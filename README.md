# Unity_ROS2_sample
English | [日本語](README-ja.md)

## Overview
This repository is a sample implementation of a ROS2-integrated simulator using Unity. It combines Unity's real-time rendering with ROS2's communication capabilities to provide an evaluation environment for robot development and algorithm verification.

## Prerequisites
- Unity 2022.3 LTS or higher
- ROS2 Humble or higher
- Ubuntu 22.04 LTS (recommended)

## Installation
1. Clone this repository:
```
git clone https://github.com/yourusername/Unity_ROS2_sample.git
```

2. Build the Docker image:
```
cd Unity_ROS2_sample/docker
./build-docker-image.sh
```

3. Run the Docker container:
```
./run-docker-container.sh
```

4. Build the ROS2 packages:
```
colcon build
source install/setup.bash
```

## Usage
1. Run the simulation scene in Unity:
```
./scripts/run_simulator.sh
```

2. Run the TCP connector from a separate terminal:
```
docker exec -it ros-humble-unity-sample /bin/bash
```
```
./scripts/run_tcp_connector.sh
```

3. Spawn the robot from another terminal:
```
docker exec -it ros-humble-unity-sample /bin/bash
```
```
ros2 launch unity_diffbot_sim diffbot_spawn.launch.py
```

4. Run teleop_twist_keyboard from another terminal:
```
docker exec -it ros-humble-unity-sample /bin/bash
```
```
./scripts/start_sim.sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Key Features
- Bidirectional communication with Unity via ROS2 topics
- Physics simulation environment
- Sensor data simulation
- Customizable robot models

## Acknowledgements

This project reuses the KHR3-HV biped robot model provided in MasutaniLab’s [choreonoid_ros_khr3](https://github.com/MasutaniLab/choreonoid_ros_khr3).
We appreciate the authors for making it available to the community.

