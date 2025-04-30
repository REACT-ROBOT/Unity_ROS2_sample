#!/bin/bash

file_dir=`dirname $0`

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
  -it --name "ros-humble-unity-sample" ${user}/ros-humble-jammy-unity-sample
