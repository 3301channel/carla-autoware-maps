#!/usr/bin/env bash
# 一键启动 Autoware + CARLA 仿真，并自动 engage（免去手动点 AUTO）。
#
# 用法：
#   1) 先在另一个终端启动 CARLA 并等待就绪：
#        source ~/.nvidia_env.sh
#        cd ~/carla && ./CarlaUE4.sh -prefernvidia -quality-level=Low -RenderOffScreen -nosound
#   2) 运行本脚本：
#        bash ~/start_autoware_sim.sh
#   3) 在 RViz 里放 2D Goal Pose，车辆会在进入 WaitingForEngage 时自动起步。
#
# 按 Ctrl-C 会同时停止 Autoware 与 auto_engage。

set -euo pipefail

MAP_PATH="${MAP_PATH:-$HOME/autoware_data/maps/Town04}"
VEHICLE_MODEL="${VEHICLE_MODEL:-sample_vehicle}"
SENSOR_MODEL="${SENSOR_MODEL:-carla_sensor_kit}"
SPAWN_POINT="${SPAWN_POINT:-}"   # 留空则使用 launch 默认值（车库旁车道）

# ROS 的 setup.bash 会引用未定义变量，临时关闭 set -u 以免报错
set +u
source "$HOME/.nvidia_env.sh" 2>/dev/null || true
source /opt/ros/humble/setup.bash
source "$HOME/autoware_2.1.0/install/setup.bash"
set -u

# 后台启动 auto_engage，退出时一并清理
python3 "$HOME/auto_engage.py" &
AUTO_ENGAGE_PID=$!
cleanup() {
  kill "$AUTO_ENGAGE_PID" 2>/dev/null || true
  wait "$AUTO_ENGAGE_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

LAUNCH_ARGS=(
  "map_path:=$MAP_PATH"
  "vehicle_model:=$VEHICLE_MODEL"
  "sensor_model:=$SENSOR_MODEL"
  "simulator_type:=carla"
)
if [[ -n "$SPAWN_POINT" ]]; then
  LAUNCH_ARGS+=("spawn_point:=$SPAWN_POINT")
fi

ros2 launch autoware_launch e2e_simulator.launch.xml "${LAUNCH_ARGS[@]}"
