#!/bin/bash
# Start the container.
#
#   ./run-docker-container.bash [humble|jazzy]
#
# Defaults to humble. Container name is ros-<distro>-unity-sample, so a humble
# and a jazzy container can coexist.

file_dir=`dirname $0`

# select ROS distro
distro=${1:-humble}
case "${distro}" in
  humble) codename=jammy ;;
  jazzy)  codename=noble ;;
  *)
    echo "Unsupported ROS distro '${distro}'. Use humble or jazzy." >&2
    exit 1
    ;;
esac

# start sharing xhost
xhost +local:root
user=`id -un`

if type nvidia-container-runtime >/dev/null 2>&1; then
  GPU_OPT="--gpus all"
fi

# run docker
docker run -it --rm \
  --net=host \
  --ipc=host \
  ${GPU_OPT} \
  --privileged \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $HOME/.Xauthority:$docker/.Xauthority \
  -v ${file_dir}/../colcon_ws:/home/unity/colcon_ws \
  -e XAUTHORITY=$home_folder/.Xauthority \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /run/dbus/system_bus_socket:/run/dbus/system_bus_socket \
  -it --name "ros-${distro}-unity-sample" ${user}/ros-${distro}-${codename}-unity-sample
