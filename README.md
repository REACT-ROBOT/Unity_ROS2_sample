# Unity_ROS2_sample
English | [日本語](README-ja.md)

## Overview
This repository is a sample implementation of a ROS2-integrated simulator using Unity. It combines Unity's real-time rendering with ROS2's communication capabilities to provide an evaluation environment for robot development and algorithm verification.

## Prerequisites
- Unity 2022.3 LTS or higher
- ROS 2 Humble (Ubuntu 22.04) or Jazzy (Ubuntu 24.04)
- Docker (either distro runs entirely inside the container)

## Installation
1. Clone this repository:
```
git clone https://github.com/yourusername/Unity_ROS2_sample.git
```

2. Build the Docker image. The ROS distro is chosen by argument (humble by default):
```
cd Unity_ROS2_sample/docker
./build-dokcer-image.bash          # ROS 2 Humble / Ubuntu 22.04
./build-dokcer-image.bash jazzy    # ROS 2 Jazzy  / Ubuntu 24.04
```

3. Run the Docker container. Pass the same distro you built:
```
./run-docker-container.bash
./run-docker-container.bash jazzy
```

Containers are named `ros-<distro>-unity-sample`, so a humble and a jazzy one can coexist.

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
docker exec -it ros-humble-unity-sample /bin/bash   # or ros-jazzy-unity-sample
```
```
./scripts/run_tcp_connector.sh
```

3. Spawn the robot from another terminal:
```
docker exec -it ros-humble-unity-sample /bin/bash   # or ros-jazzy-unity-sample
```
```
ros2 launch unity_diffbot_sim diffbot_spawn.launch.py
```

4. Run teleop_twist_keyboard from another terminal:
```
docker exec -it ros-humble-unity-sample /bin/bash   # or ros-jazzy-unity-sample
```
```
./scripts/start_sim.sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Service conformance test
An automated suite checks that the features the simulator exposes through
`simulation_interfaces` services (`spawn_entity` / `set_simulation_state` /
`get_simulation_state` / `reset_simulation` / `step_simulation`) behave as specified.
It exists in particular to reproduce and isolate the class of bug where the robot stops
accepting commands after `reset_simulation` is called.

Run it inside the container:
```
cd ~/colcon_ws
colcon build --packages-select simulation_service_tests simulation_ros2_utils
source install/setup.bash
./scripts/service_conformance_test.sh
```

The script brings up the ROS-TCP-Endpoint and the simulator, runs the suite, and tears it down.
Exit codes: 0 = everything as expected, 1 = defects found, 2 = could not run.

See [colcon_ws/src/simulation_service_tests/README.md](colcon_ws/src/simulation_service_tests/README.md) for details.

## Key Features
- Bidirectional communication with Unity via ROS2 topics
- Physics simulation environment
- Sensor data simulation
- Customizable robot models

## Acknowledgements

This project reuses the KHR3-HV biped robot model provided in MasutaniLab’s [choreonoid_ros_khr3](https://github.com/MasutaniLab/choreonoid_ros_khr3).
We appreciate the authors for making it available to the community.

