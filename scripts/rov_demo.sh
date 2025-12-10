#!/usr/bin/env bash
set -euo pipefail

echo "=============================================="
echo "🎬 DEMO: Implementasi Kendali ROV (SITL)"
echo "=============================================="

ROS_DISTRO=${ROS_DISTRO:-jazzy}
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/rov_ws/install/setup.bash

# Bersihkan
pkill -f sim_vehicle.py || true
pkill -f mavproxy.py || true
pkill -f mavros_node || true
sleep 1

# Log folder
mkdir -p ~/rov_logs/demo

# Jalankan SITL
gnome-terminal --title="SITL ArduSub" -- bash -c "
cd ~/ardupilot/ArduSub;
sim_vehicle.py -v ArduSub -L RATBeach --no-mavproxy \
  --out=tcp:127.0.0.1:5760 > ~/rov_logs/demo/sitl.log 2>&1"

sleep 10

# MAVProxy bridge
gnome-terminal --title="MAVProxy Bridge" -- bash -c "
mavproxy.py --master=tcp:127.0.0.1:5760 \
  --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14560 \
  > ~/rov_logs/demo/mavproxy.log 2>&1"

sleep 5

# MAVROS
gnome-terminal --title="MAVROS Node" -- bash -c "
ros2 run mavros mavros_node --ros-args \
  -r __ns:=/mavros/mavros_uas1 \
  -p fcu_url:=udp://:14560@ \
  -p system_id:=255 -p component_id:=191 \
  > ~/rov_logs/demo/mavros.log 2>&1"

sleep 10

# Mode & arming
echo "🟡 Setting mode ALT_HOLD..."
ros2 service call /mavros/mavros_uas1/set_mode mavros_msgs/srv/SetMode "{custom_mode: ALT_HOLD}"
sleep 3
echo "🟢 Arming..."
ros2 service call /mavros/mavros_uas1/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

sleep 5
echo "🚀 Jalankan algoritma ForwardRightStop"
gnome-terminal --title="Algorithm: ForwardRightStop" -- bash -c "
ros2 run rov_ctrl_alg_template alg_runner --ros-args \
  -p algorithm_class:='rov_ctrl_alg_template.algorithms.forward_right_stop:ForwardRightStop' \
  -p params:='{\"surge\":0.6,\"sway\":0.5,\"t_forward\":3.0,\"t_right\":3.0}'"

sleep 12

echo "🔁 Ganti algoritma: PIDHoldDepth (maintain depth)"
pkill -f forward_right_stop || true
sleep 2
gnome-terminal --title="Algorithm: PIDHoldDepth" -- bash -c "
ros2 run rov_ctrl_alg_template alg_runner --ros-args \
  -p algorithm_class:='rov_ctrl_alg_template.algorithms.pid_hold_depth:PIDHoldDepth' \
  -p params:='{\"target_depth\":0.5,\"Kp\":1.0,\"Ki\":0.05,\"Kd\":0.02}'"

sleep 15

echo "🛑 Disarm ROV..."
ros2 service call /mavros/mavros_uas1/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"
echo "✅ Demo selesai! Semua log tersimpan di ~/rov_logs/demo/"
