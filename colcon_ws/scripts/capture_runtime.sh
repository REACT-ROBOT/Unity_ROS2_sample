#!/bin/bash
# Record a servo_demo run for the runtime validation figure. Runs INSIDE the
# container. Usage: capture_runtime.sh <sim_dir> <out_csv> [log_seconds]
#
# Same bring-up order as scripts/bringup_test.sh (endpoint -> simulator ->
# start -> launch); the difference is that it then just logs, long enough to
# cover a full commander cycle (two 18 s sweeps plus the steps ~= 46 s).
SIM_DIR=${1:-$HOME/simbuild}
OUT=${2:-/home/unity/colcon_ws/runtime_capture.csv}
DUR=${3:-70}

rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/unity/colcon_ws/scripts/fastdds_udp_only.xml

for p in 'Simulator.x8[6]_64' 'servo_demo_spaw[n]' 'default_server_endpoin[t]' \
         'ros2_control_nod[e]' 'robot_state_publishe[r]' 'servo_demo_commande[r]' \
         '[s]pawner joint_' 'set_sim_stat[e]'; do
  pkill -f "$p" 2>/dev/null
done
sleep 2
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source ~/colcon_ws/install/setup.sh

nohup ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 \
  > /tmp/endpoint.log 2>&1 &
sleep 5

export DISPLAY=:1 XAUTHORITY=/.Xauthority
cd "$SIM_DIR" && nohup ./Unity_ROS2_Robot_Simulator.x86_64 > /tmp/sim_bringup.log 2>&1 &
sleep 14

timeout 30 ros2 run simulation_ros2_utils set_sim_state --ros-args -p set_state:=start 2>&1 | tail -1
sleep 2

cd ~/colcon_ws
nohup ros2 launch servo_demo_description servo_demo_spawn.launch.py > /tmp/demo_launch.log 2>&1 &
# controller spawners + the commander's first publish need ~20 s
sleep 20

python3 ~/colcon_ws/scripts/log_servo_demo.py "$OUT" "$DUR"

for p in 'Simulator.x8[6]_64' 'servo_demo_spaw[n]' 'default_server_endpoin[t]' \
         'ros2_control_nod[e]' 'robot_state_publishe[r]' 'servo_demo_commande[r]'; do
  pkill -f "$p" 2>/dev/null
done
echo "capture done: $OUT"
wc -l "$OUT"
