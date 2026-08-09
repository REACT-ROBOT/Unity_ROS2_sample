#!/bin/bash
# Build the container image.
#
#   ./build-dokcer-image.bash [humble|jazzy]
#
# Defaults to jazzy. The image is tagged <user>/ros-<distro>-<codename>-unity-sample.

set -e

file_dir=`dirname $0`

# select ROS distro
distro=${1:-jazzy}
case "${distro}" in
  humble) codename=jammy ;;
  jazzy)  codename=noble ;;
  *)
    echo "Unsupported ROS distro '${distro}'. Use humble or jazzy." >&2
    exit 1
    ;;
esac

# get parameter from system
user=`id -un`
group=`id -gn`
uid=`id -u`
gid=`id -g`

echo "Building ${user}/ros-${distro}-${codename}-unity-sample (ROS 2 ${distro} / Ubuntu ${codename})"

# build docker images
docker build -t ${user}/ros-${distro}-${codename}-unity-sample \
    --build-arg ROS_DISTRO_ARG=${distro} \
    --build-arg UBUNTU_CODENAME=${codename} \
    --build-arg USER=unity \
    --build-arg UID=${uid} \
    --build-arg GROUP=${group} \
    --build-arg GID=${gid} \
    ${file_dir}
