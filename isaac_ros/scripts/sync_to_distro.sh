#!/usr/bin/env bash
# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.
#
# Copy this directory into the Isaac ROS distro, where the container mounts it:
#   ~/workspaces/isaac_ros-dev/src/rover_perception/isaac_ros    (distro Ubuntu-24.04)
#   /workspaces/isaac_ros-dev/src/rover_perception/isaac_ros     (inside the container)
# and point the Isaac ROS CLI at docker/ for the `rover_perception` image layer.
# Run from the rover distro after editing anything here. The two distros do not share a
# filesystem, so this copy is one-way: edit here, not there.
set -euo pipefail

DISTRO="${ISAAC_ROS_DISTRO:-Ubuntu-24.04}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST='workspaces/isaac_ros-dev/src/rover_perception'

cd /mnt/c # wsl.exe cannot translate a \\wsl.localhost working directory
tar -C "$HERE/.." -c isaac_ros \
    | wsl.exe -d "$DISTRO" -- bash -c "mkdir -p ~/$DEST && tar -x -C ~/$DEST"
wsl.exe -d "$DISTRO" -- bash -c "mkdir -p ~/workspaces/isaac_ros-dev/scripts \
    && cp ~/$DEST/isaac_ros/config/isaac_ros_common-config ~/workspaces/isaac_ros-dev/scripts/.isaac_ros_common-config"
echo "synced to $DISTRO:~/$DEST/isaac_ros"
