#!/usr/bin/env python3
"""Autoware 自动 engage 工具。

监听 /autoware/state，当状态进入 WAITING_FOR_ENGAGE 且自动驾驶模式可用时，
自动调用 /api/operation_mode/change_to_autonomous，让车辆免去手动点 AUTO。

用法（必须与 Autoware 使用相同的 DDS 环境）：
    source ~/.nvidia_env.sh
    source /opt/ros/humble/setup.bash
    source ~/autoware_2.1.0/install/setup.bash
    python3 ~/auto_engage.py

按 Ctrl-C 退出。
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_adapi_v1_msgs.srv import ChangeOperationMode
from autoware_system_msgs.msg import AutowareState
from rclpy.executors import ExternalShutdownException

# AutowareState 枚举
STATE_WAITING_FOR_ENGAGE = 4
STATE_DRIVING = 5
STATE_AUTONOMOUS = 6
STATE_NAMES = {
    0: "UNKNOWN",
    1: "INITIALIZING",
    2: "WAITING_FOR_ROUTE",
    3: "PLANNING",
    4: "WAITING_FOR_ENGAGE",
    5: "DRIVING",
    6: "AUTONOMOUS",
    7: "FINALIZING",
}

# OperationModeState.mode 枚举
MODE_STOP = 1
MODE_AUTONOMOUS = 2


def _as_int(value, default=-1):
    """兼容不同 RMW 下 uint8 被解析成 bytes 或 int 的情况。"""
    if isinstance(value, (bytes, bytearray)):
        return value[0] if value else default
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


class AutoEngage(Node):
    def __init__(self, retry_interval_s: float = 2.0):
        super().__init__("auto_engage")
        self._state = None
        self._mode = None
        self._available = False
        self._last_attempt = 0.0
        self._retry_interval = retry_interval_s
        self._pending = False

        qos_latched = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(AutowareState, "/autoware/state", self._on_state, 10)
        self.create_subscription(
            OperationModeState, "/api/operation_mode/state", self._on_mode, qos_latched
        )
        self._client = self.create_client(
            ChangeOperationMode, "/api/operation_mode/change_to_autonomous"
        )
        self.create_timer(0.5, self._tick)

        self.get_logger().info("auto_engage 已启动，等待状态进入 WAITING_FOR_ENGAGE ...")

    def _on_state(self, msg: AutowareState) -> None:
        new_state = _as_int(msg.state)
        if new_state != self._state:
            self._state = new_state
            self.get_logger().info(
                f"AutowareState -> {STATE_NAMES.get(new_state, new_state)}"
            )

    def _on_mode(self, msg: OperationModeState) -> None:
        self._mode = _as_int(msg.mode)
        self._available = bool(msg.is_autonomous_mode_available)

    def _tick(self) -> None:
        if self._state != STATE_WAITING_FOR_ENGAGE:
            return
        if self._mode != MODE_STOP or not self._available:
            return
        if self._pending:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_attempt < self._retry_interval:
            return
        if not self._client.service_is_ready():
            self.get_logger().warn("等待 engage 服务就绪 ...")
            return

        self._last_attempt = now
        self._pending = True
        self.get_logger().info("检测到 WAITING_FOR_ENGAGE，正在自动 engage ...")
        future = self._client.call_async(ChangeOperationMode.Request())
        future.add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        self._pending = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"engage 调用异常: {exc}")
            return
        if response is None:
            self.get_logger().warn("engage 无返回结果")
            return
        if response.status.code == 0:
            self.get_logger().info("engage 成功，已进入 AUTONOMOUS")
        else:
            self.get_logger().warn(
                f"engage 失败 (code={response.status.code}): {response.status.message}"
            )


def main() -> int:
    rclpy.init()
    node = AutoEngage()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到 Ctrl-C，退出 auto_engage")
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
