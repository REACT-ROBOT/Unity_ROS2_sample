#!/bin/bash
# Demo bring-up + health check, run INSIDE the container.
# Usage: bringup_test.sh <sim_dir>
# Prints CLEAN if both demo joints are near zero shortly after startup.
SIM_DIR=${1:-$HOME/Unity_ROS2_Robot_Simulator_v0.9.3_Linux_amd64}

# UDP-only DDS (stale /dev/shm/fastrtps_* from killed processes breaks
# discovery; clean them and avoid creating new ones)
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/unity/colcon_ws/scripts/fastdds_udp_only.xml

# kill the whole previous stack INCLUDING launch children (ros2 launch's
# children survive a pkill of the launch process itself)
pkill -f 'Simulator.x8[6]_64' 2>/dev/null
pkill -f 'servo_demo_spaw[n]' 2>/dev/null
pkill -f 'default_server_endpoin[t]' 2>/dev/null
pkill -f 'ros2_control_nod[e]' 2>/dev/null
pkill -f 'robot_state_publishe[r]' 2>/dev/null
pkill -f 'servo_demo_commande[r]' 2>/dev/null
pkill -f '[s]pawner joint_' 2>/dev/null
pkill -f 'set_sim_stat[e]' 2>/dev/null
sleep 2
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source ~/colcon_ws/install/setup.sh

# endpoint FIRST so Unity's initial connection (and all topic registrations)
# happen against a live endpoint
nohup ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 > /tmp/endpoint.log 2>&1 &
sleep 5

export DISPLAY=:1 XAUTHORITY=/.Xauthority
cd "$SIM_DIR" && nohup ./Unity_ROS2_Robot_Simulator.x86_64 > /tmp/sim_bringup.log 2>&1 &
sleep 14

timeout 30 ros2 run simulation_ros2_utils set_sim_state --ros-args -p set_state:=start 2>&1 | tail -1
sleep 2

nohup ros2 launch servo_demo_description servo_demo_spawn.launch.py > /tmp/demo_launch.log 2>&1 &
sleep 22

sample() {
  timeout 6 ros2 topic echo /ServoDemo/joint_states --once 2>/dev/null | sed 's/\x1b\[[0-9;]*m//g' | python3 -c "
import sys
lines = [l.rstrip() for l in sys.stdin]
try:
    ni = lines.index('name:')
    names = [lines[ni+1].strip('- '), lines[ni+2].strip('- ')]
    pi = next(i for i, l in enumerate(lines) if l == 'position:')
    vals = [float(lines[pi+1].strip('- ')), float(lines[pi+2].strip('- '))]
    m = dict(zip(names, vals))
    print(f\"{m.get('ideal_joint')} {m.get('cheap_joint')}\")
except Exception as e:
    print('nan nan')
"
}
S1=$(sample)
sleep 4
S2=$(sample)
python3 -c "
i1, c1 = map(float, '$S1'.split())
i2, c2 = map(float, '$S2'.split())
moving = abs(i1 - i2) > 0.02 or abs(c1 - c2) > 0.02
sane = all(abs(v) < 1.6 for v in (i1, c1, i2, c2))
print(f'ideal={i2:+.3f} cheap={c2:+.3f} moved={moving}')
print('CLEAN' if moving and sane else 'BAD')
"
