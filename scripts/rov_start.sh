#!/usr/bin/env bash
set -euo pipefail
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-84}
ROS_SETUP="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
WS_SETUP="$HOME/rov_ws/install/setup.bash"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
SIM_TOOL="$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py"
LOC="${LOC:-RATBeach}"
QGC_DIR="${QGC_DIR:-$HOME/QGroundControl}"
QGC_APP=""
[[ -x "$QGC_DIR/QGroundControl.AppImage" ]] && QGC_APP="$QGC_DIR/QGroundControl.AppImage"
[[ -z "$QGC_APP" && -x "$QGC_DIR/QGroundControl-x86_64.AppImage" ]] && QGC_APP="$QGC_DIR/QGroundControl-x86_64.AppImage"
LOG_DIR="${LOG_DIR:-$HOME/rov_logs}"
mkdir -p "$LOG_DIR"
TCP_SITL=${TCP_SITL:-5760}
UDP_QGC=${UDP_QGC:-14550}
UDP_MAVROS=${UDP_MAVROS:-14560}
NS="${NS:-/mavros/mavros_uas1}"
HEADLESS=${HEADLESS:-0}
QGC_BIND_ALL=${QGC_BIND_ALL:-0}

has_cmd(){ command -v "$1" >/dev/null 2>&1; }
open_term(){ local title="$1"; shift; local cmd="$*";
  if has_cmd gnome-terminal; then gnome-terminal --title="$title" -- bash -lc "$cmd; exec bash";
  else nohup bash -lc "$cmd" >/dev/null 2>&1 & fi; }
wait_port(){ local proto="$1" port="$2" sec="$3"; for _ in $(seq 1 "$sec"); do ss -lntp 2>/dev/null | grep -qE "LISTEN .*:${port}\b" && return 0; sleep 1; done; return 1; }
wait_topic_pub(){ local t="$1" sec="$2"; for _ in $(seq 1 "$sec"); do ros2 topic info "$t" --verbose 2>/dev/null | grep -qE 'Publisher count:\s*[1-9]' && return 0; sleep 1; done; return 1; }
relax_source(){ set +u; source "$1" >/dev/null 2>&1 || true; set -u; }

[[ -x "$SIM_TOOL" ]] || { echo "❌ sim_vehicle.py tidak ditemukan: $SIM_TOOL"; exit 1; }
export PATH="$ARDUPILOT_DIR/Tools/autotest:$HOME/.local/bin:$PATH"
pkill -f sim_vehicle.py 2>/dev/null || true; pkill -f mavproxy.py 2>/dev/null || true; pkill -f mavros_node 2>/dev/null || true; pkill -f QGroundControl 2>/dev/null || true
relax_source "$ROS_SETUP"; relax_source "$WS_SETUP"

open_term "SITL ArduSub" "cd $ARDUPILOT_DIR/ArduSub || exit 1; $SIM_TOOL -v ArduSub -L $LOC --no-mavproxy --out=tcp:127.0.0.1:$TCP_SITL 2>&1 | tee '$LOG_DIR/sitl.log'"
wait_port tcp "$TCP_SITL" 45 || { echo '❌ SITL belum siap'; exit 1; }

OUT_QGC="udp:127.0.0.1:$UDP_QGC"; [[ "$QGC_BIND_ALL" = "1" ]] && OUT_QGC="udp:0.0.0.0:$UDP_QGC"
open_term "MAVProxy Bridge" "mavproxy.py --master=tcp:127.0.0.1:$TCP_SITL --out=$OUT_QGC --out=udp:127.0.0.1:$UDP_MAVROS 2>&1 | tee '$LOG_DIR/mavproxy.log'"
sleep 4

open_term "MAVROS" "source $ROS_SETUP; source $WS_SETUP 2>/dev/null || true; ros2 run mavros mavros_node --ros-args -r __ns:=$NS --params-file $(pwd)/config/mavros_params.yaml -p fcu_url:=udp://:$UDP_MAVROS@ -p system_id:=255 -p component_id:=191 2>&1 | tee '$LOG_DIR/mavros.log'"
if [[ -n "$QGC_APP" && "$HEADLESS" != "1" ]]; then nohup "$QGC_APP" >/dev/null 2>&1 & fi

STATE_TOPIC="$NS/state"
if wait_topic_pub "$STATE_TOPIC" 30; then echo "✅ Heartbeat di $STATE_TOPIC"; else echo "⚠️  Belum terlihat publisher $STATE_TOPIC"; fi

cat <<EOF
Siap. Coba:
ros2 service call $NS/set_mode mavros_msgs/srv/SetMode "{custom_mode: ALT_HOLD}"
ros2 service call $NS/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
ros2 topic pub --rate 10 $NS/manual_control/send mavros_msgs/msg/ManualControl '{x: 700, y: 0, z: 500, r: 0, buttons: 0}'
EOF
