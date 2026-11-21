#!/usr/bin/env python3
"""
parking_replan_watchdog.py — 泊车超时看门狗

作用
----
当车辆在 Parking 场景下长时间卡住（速度持续为 0 且未报完成）时，
自动清空并重新下发同一条路线，让 freespace_planner 从【当前自车位姿】
重新规划到【原 goal】。

原理
----
autoware_freespace_planner 的 onRoute() 收到新路线时会调用 reset()，
清空 trajectory_ 并以上一帧 odometry 为起点重新做 A* 搜索。所以只要
重发同一条 route，规划器就会从当前位置重规划，终点仍是原 goal。

用法
----
    # 终端 1：启动仿真与 Autoware
    # 终端 2：
    source /opt/ros/humble/setup.bash
    source ~/autoware_2.1.0/install/setup.bash
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    python3 ~/parking_replan_watchdog.py

参数（命令行）
--------------
    --stuck-sec    速度持续为 0 多少秒判定为卡住（默认 20）
    --max-retry    最多重规划几次（默认 5）
    --watchdog-only 只告警不重规划（调试用）

注意
----
重规划只能解决"规划器卡住"，不能解决"定位/诊断异常导致模式抖动"。
如果 emergency 频繁出现，请先排查 sensor_fusion_status。
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from autoware_adapi_v1_msgs.srv import ClearRoute, SetRoutePoints
from autoware_internal_planning_msgs.msg import Scenario
from autoware_planning_msgs.msg import LaneletRoute
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class ParkingReplanWatchdog(Node):
    def __init__(self, stuck_sec: float, max_retry: int, dry_run: bool):
        super().__init__('parking_replan_watchdog')
        self.stuck_sec = stuck_sec
        self.max_retry = max_retry
        self.dry_run = dry_run

        self.goal = None          # (x, y, qz, qw)
        self.speed = 0.0
        self.scenario = ''
        self.completed = None

        self.stuck_since = None
        self.retry_count = 0
        self.last_action = 0.0

        # ---- 订阅 ----
        self.create_subscription(LaneletRoute, '/planning/mission_planning/route',
                                 self.on_route, QoSProfile(depth=1))
        self.create_subscription(Odometry, '/localization/kinematic_state',
                                 self.on_odom, QoSProfile(depth=10))
        self.create_subscription(Scenario, '/planning/scenario_planning/scenario',
                                 self.on_scenario, QoSProfile(depth=10))
        self.create_subscription(Bool, '/planning/scenario_planning/parking/is_completed',
                                 self.on_completed, QoSProfile(depth=10))

        # ---- 服务客户端 ----
        self.cli_clear = self.create_client(ClearRoute, '/api/routing/clear_route')
        self.cli_set = self.create_client(SetRoutePoints, '/api/routing/set_route_points')

        # ---- 定时检查（1 Hz）----
        self.create_timer(1.0, self.tick)

        self.get_logger().info(
            f'watchdog started: stuck_sec={stuck_sec}s, max_retry={max_retry}, '
            f'dry_run={dry_run}')

    # ---------------- 回调 ----------------
    def on_route(self, msg: LaneletRoute):
        p = msg.goal_pose.position
        q = msg.goal_pose.orientation
        if self.goal is None:
            self.get_logger().info(
                f'捕获原始 goal: ({p.x:.3f}, {p.y:.3f}) '
                f'q=({q.z:.5f}, {q.w:.5f})')
        self.goal = (p.x, p.y, q.z, q.w)

    def on_odom(self, msg: Odometry):
        v = msg.twist.twist.linear
        self.speed = math.sqrt(v.x * v.x + v.y * v.y)

    def on_scenario(self, msg: Scenario):
        self.scenario = getattr(msg, 'current_scenario', '')

    def on_completed(self, msg: Bool):
        self.completed = msg.data

    # ---------------- 主逻辑 ----------------
    def tick(self):
        now = time.time()

        # 只在 Parking 场景、未完成、有 goal 时判定
        if self.scenario != 'Parking' or self.completed is True or self.goal is None:
            self.stuck_since = None
            return

        stopped = self.speed < 0.05
        if not stopped:
            # 车动了 → 清空计时并允许下一轮重试
            if self.stuck_since is not None:
                self.get_logger().info('车辆重新移动，计时清零')
            self.stuck_since = None
            self.retry_count = 0
            return

        if self.stuck_since is None:
            self.stuck_since = now
            return

        stuck = now - self.stuck_since
        if stuck < self.stuck_sec:
            return

        if self.retry_count >= self.max_retry:
            self.get_logger().warn(
                f'已重规划 {self.max_retry} 次仍卡住，停止尝试。请检查 '
                f'sensor_fusion_status / emergency 相关诊断。',
                throttle_duration_sec=10.0)
            return

        if now - self.last_action < 5.0:
            return

        self.last_action = now
        self.retry_count += 1
        gx, gy, qz, qw = self.goal
        self.get_logger().warn(
            f'车辆已静止 {stuck:.0f}s（speed={self.speed:.3f}），'
            f'第 {self.retry_count}/{self.max_retry} 次重规划 → '
            f'goal=({gx:.3f}, {gy:.3f})')

        if self.dry_run:
            self.get_logger().info('[dry-run] 跳过实际服务调用')
            self.stuck_since = now
            return

        self.do_replan(gx, gy, qz, qw)
        self.stuck_since = now   # 重新计时，给规划器时间

    def do_replan(self, gx, gy, qz, qw):
        if not self.cli_clear.service_is_ready():
            self.get_logger().warn('clear_route 服务未就绪，跳过本次重规划')
            return
        if not self.cli_set.service_is_ready():
            self.get_logger().warn('set_route_points 服务未就绪，跳过本次重规划')
            return

        # 1) 清空旧路线（否则 set_route_points 会返回 ERROR_INVALID_STATE）
        fut = self.cli_clear.call_async(ClearRoute.Request())
        fut.add_done_callback(lambda f: self._on_clear_done(f, gx, gy, qz, qw))

    def _on_clear_done(self, fut, gx, gy, qz, qw):
        try:
            res = fut.result()
            code = getattr(getattr(res, 'status', None), 'code', None)
            self.get_logger().info(f'clear_route 完成, code={code}')
        except Exception as e:
            self.get_logger().warn(f'clear_route 失败: {e}')

        req = SetRoutePoints.Request()
        req.header.frame_id = 'map'
        req.option.allow_goal_modification = False
        req.goal.position.x = float(gx)
        req.goal.position.y = float(gy)
        req.goal.position.z = 0.0
        req.goal.orientation.z = float(qz)
        req.goal.orientation.w = float(qw)
        req.waypoints = []

        fut2 = self.cli_set.call_async(req)
        fut2.add_done_callback(self._on_set_done)

    def _on_set_done(self, fut):
        try:
            res = fut.result()
            code = getattr(getattr(res, 'status', None), 'code', None)
            if code == 0:
                self.get_logger().info('重规划已下发，路线重建成功')
            else:
                self.get_logger().warn(f'set_route_points 返回 code={code}')
        except Exception as e:
            self.get_logger().warn(f'set_route_points 失败: {e}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--stuck-sec', type=float, default=20.0)
    ap.add_argument('--max-retry', type=int, default=5)
    ap.add_argument('--watchdog-only', action='store_true')
    args = ap.parse_args()

    rclpy.init()
    node = ParkingReplanWatchdog(args.stuck_sec, args.max_retry, args.watchdog_only)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
