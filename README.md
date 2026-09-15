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
1. Clone this repository, submodules included:
```
git clone --recursive https://github.com/yourusername/Unity_ROS2_sample.git
```
An existing clone needs `git submodule update --init --recursive`: the simulator's
services and the `MagneticGuide` message come from `simulation_interfaces` and
`simulation_ros2_utils`, which are pinned there.

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

## Trying the features added in simulator v1.4.0

The simulator version is pinned in `colcon_ws/scripts/simulator_version.txt`, now
**v1.4.0**. Three of its additions are wired into this sample: objects a lidar sees but
never collides with, an AGV magnetic line sensor with the tape it follows, and a GNSS
receiver whose fix degrades the way the buildings around it say it should.

The diffbot carries the new sensors by default. Drop them with
`use_magnetic_guide:=false` / `use_gnss:=false` on the xacro if you do not want the extra
work per frame.

Start from the four terminals of [Usage](#usage), then add a prop in another one. The
props live in `sim_props_description`; each is a URDF entity, so it is listed by
`get_entities`, removed by `delete_entity`, and cleared by `reset_simulation` with
`SCOPE_SPAWNED`.

> The `MagneticGuide` message comes from `simulation_extra_interfaces`, built from source
> in this workspace. Build it and **restart the TCP connector before the simulator
> connects**, or the type will not resolve.

### Objects the lidar sees but drives through

```
ros2 launch sim_props_description spawn_prop.launch.py prop:=weeds
```

Weeds stand 1.5–4 m ahead of the spawn point, taller than the diffbot's lidar plane.
Drive into them with `teleop_twist_keyboard`: `/diffbot/lidar_link/scan` returns them at
their true range, the robot passes straight through, and `get_contact_events` records
nothing.

What makes them see-through is `<collision_material><sensor_only value="true"/>`, which
turns the collision shape into a Unity trigger — raycasts hit it, contact resolution
ignores it. A link with **no** `<collision>` is not the same thing: the lidar is a physics
raycast, so such a link is simply invisible.

### Magnetic line sensor

```
ros2 launch sim_props_description spawn_prop.launch.py prop:=magnetic_course
ros2 run unity_diffbot_sim magnetic_line_follower
```

The course is a 21 m oval of magnetic tape with three markers beside it, and the robot
spawns on it. `magnetic_line_follower` is a plain proportional controller on the reported
lateral offset — the same first stage a real AGV uses — and logs each marker it passes.

```
ros2 topic echo /diffbot/magnetic_guide_link/magnetic_guide
```

`position` is the tape's lateral offset in metres, positive to the robot's left, and
`track_positions` lists every track under the 160 mm bar, so a fork shows two.

The tape is `<collision_material><magnetic_tape polarity="track|marker"/>` on thin boxes;
`magnetic_tape` implies `sensor_only`, so the robot drives over it. `courses/*.json` plus
`scripts/gen_tape_urdf.py` generate the URDF, so a course of your own is a polyline away —
see [sim_props_description](colcon_ws/src/sim_props_description/README.md).

### GNSS in an urban canyon

```
ros2 launch sim_props_description spawn_prop.launch.py prop:=gnss_canyon
```

Buildings line an 8 m street from x=4 to x=26, with two 2 m side streets. Driving along it
takes the antenna from open sky into the canyon and back out:

```
ros2 topic echo /diffbot/gnss_antenna_link/extended_fix --field status
```

`GPSStatus.status` separates an RTK fix (19) from an RTK float (20), which `NavSatFix`
structurally cannot — on `/diffbot/gnss_antenna_link/fix` the quality travels in
`position_covariance` instead, which is what `navsat_transform_node` reads.
`/diffbot/gnss_antenna_link/nmea` carries GGA/RMC, so an NMEA driver from real hardware
works here unchanged.

The error is not noise sprinkled on the truth. A blocked satellite drops out of the
solution and a reflected one enters it carrying the excess length of its detour, so the
error **points somewhere the buildings explain and repeats at the same place**. Turn on
`<link> gnss rays` in the entity panel to see the paths: green is direct, thick amber is a
reflection.

## Key Features
- Bidirectional communication with Unity via ROS2 topics
- Physics simulation environment
- Sensor data simulation
- Customizable robot models

## Acknowledgements

This project reuses the KHR3-HV biped robot model provided in MasutaniLab’s [choreonoid_ros_khr3](https://github.com/MasutaniLab/choreonoid_ros_khr3).
We appreciate the authors for making it available to the community.

