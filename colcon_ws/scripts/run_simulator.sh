#!/bin/bash
# バージョンの定義は simulator_version.txt の 1 か所だけ。
source "$(dirname "$0")/simulator_version.sh"
"${SIM_DIR_DEFAULT}"/Unity_ROS2_Robot_Simulator.x86_64
