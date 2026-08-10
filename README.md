# Unity_ROS2_sample
English | [日本語](README-ja.md)

## Overview
This repository is a sample implementation of a ROS2-integrated simulator using Unity. It combines Unity's real-time rendering with ROS2's communication capabilities to provide an evaluation environment for robot development and algorithm verification.

## Branches

| Branch | Purpose |
|---|---|
| `main` | Development line targeting **ROS 2 Jazzy**; the default distro is jazzy |
| `humble` | For ROS 2 Humble. A snapshot of the point where Humble was verified |

The scripts read `${ROS_DISTRO}` on either branch, so `main` still runs on Humble if you
pass `./build-dokcer-image.bash humble`. The `humble` branch is for changes that Humble
needs and Jazzy does not.

> **The conformance suite on `humble` is frozen where it was.** Checks added to `main`
> afterwards — the later H scenarios from `WORLD_TAGS` onward, the I group, and G6 / F3 /
> H2b — are not on it. The checks that are there gate on the advertised features, so they do
> not fail against a simulator that has the newer ones. Cherry-pick from `main` if you want
> them aligned. This is also recorded as deferred work in the simulator's
> [Known-Limitations.md](https://github.com/hijimasa/Unity_ROS2_Robot_Simulator/blob/main/docs/Known-Limitations.md).

## Prerequisites
- Unity 2022.3 LTS or higher
- ROS 2 Jazzy (Ubuntu 24.04) or Humble (Ubuntu 22.04)
- Docker (either distro runs entirely inside the container)

## Installation
1. Clone this repository:
```
git clone https://github.com/yourusername/Unity_ROS2_sample.git
```

2. Build the Docker image. The ROS distro is chosen by argument (jazzy by default):
```
cd Unity_ROS2_sample/docker
./build-dokcer-image.bash          # ROS 2 Jazzy  / Ubuntu 24.04
./build-dokcer-image.bash humble   # ROS 2 Humble / Ubuntu 22.04
```

3. Run the Docker container. Pass the same distro you built:
```
./run-docker-container.bash
./run-docker-container.bash humble
```

Containers are named `ros-<distro>-unity-sample`, so a humble and a jazzy one can coexist.

4. Build the ROS2 packages:
```
colcon build
source install/setup.bash
```

> **Note**: `colcon_ws` cannot be shared between distros. Wipe the build products before
> switching between humble and jazzy — the Python versions differ (3.10 / 3.12), and leftovers
> make message type support fail to load with
> `UnsupportedTypeSupport: Could not import 'rosidl_typesupport_c'`.
> ```
> rm -rf build install log && colcon build
> ```

## Usage
1. Run the simulation scene in Unity:
```
./scripts/run_simulator.sh
```

2. Run the TCP connector from a separate terminal:
```
docker exec -it ros-jazzy-unity-sample /bin/bash   # or ros-humble-unity-sample
```
```
./scripts/run_tcp_connector.sh
```

3. Spawn the robot from another terminal:
```
docker exec -it ros-jazzy-unity-sample /bin/bash   # or ros-humble-unity-sample
```
```
ros2 launch unity_diffbot_sim diffbot_spawn.launch.py
```

4. Run teleop_twist_keyboard from another terminal:
```
docker exec -it ros-jazzy-unity-sample /bin/bash   # or ros-humble-unity-sample
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

