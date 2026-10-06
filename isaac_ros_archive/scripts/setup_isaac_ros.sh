#!/usr/bin/env bash
# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.
#
# Stage 3: Isaac ROS 5.0 CLI in Docker mode, with the RealSense image layer selected.
# Follows https://nvidia-isaac-ros.github.io/getting_started/index.html (x86_64, Docker) and
# .../getting_started/sensors/realsense_setup.html.
#
#   wsl.exe -d Ubuntu-24.04 -u root -- bash -s -- [user] < setup_isaac_ros.sh
#
# Afterwards: sync_to_distro.sh (from the rover distro), then as the user in a terminal of that
# distro (builds the image locally; ~10+ GB of downloads, hours on a slow uplink):
#   isaac-ros activate --build-local
set -euo pipefail

DEV_USER="${1:-rover-a1}"
ISAAC_ROS_RELEASE=release-5.0
LIBREALSENSE_VERSION=v2.56.3 # pinned by Isaac ROS 5.0, with camera firmware 5.16.0.1

if [[ $EUID -ne 0 ]]; then
    echo "run as root: wsl.exe -d Ubuntu-24.04 -u root -- bash -s < $0" >&2
    exit 1
fi
DEV_HOME="$(getent passwd "$DEV_USER" | cut -d: -f6)"

export DEBIAN_FRONTEND=noninteractive

# Isaac ROS apt repository (only a `noble` suite exists) and the CLI.
key=/usr/share/keyrings/nvidia-isaac-ros.gpg
curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key | gpg --dearmor --yes -o "$key"
echo "deb [signed-by=$key] https://isaac.download.nvidia.com/isaac-ros/$ISAAC_ROS_RELEASE noble main" \
    > /etc/apt/sources.list.d/nvidia-isaac-ros.list
apt-get update
apt-get install -y isaac-ros-cli git-lfs jq

# RealSense udev rule on the host side of the container: the RSUSB backend opens the camera
# through libusb, so the usbipd-attached device must be accessible to plugdev.
curl -fsSL -o /etc/udev/rules.d/99-realsense-libusb.rules \
    "https://raw.githubusercontent.com/realsenseai/librealsense/$LIBREALSENSE_VERSION/config/99-realsense-libusb.rules"
udevadm control --reload-rules
udevadm trigger

isaac-ros init docker

runuser -u "$DEV_USER" -- bash -s << 'EOF'
set -euo pipefail
mkdir -p "$HOME/workspaces/isaac_ros-dev/src" "$HOME/.config/isaac-ros-cli"
line='export ISAAC_ROS_WS="${ISAAC_ROS_WS:-${HOME}/workspaces/isaac_ros-dev/}"'
grep -qxF "$line" "$HOME/.bashrc" || echo "$line" >> "$HOME/.bashrc"
git lfs install
# The CLI bind-mounts this file into the container; if it is missing, Docker creates a
# root-owned directory in its place. WSLg needs no X authority, so an empty file is enough.
touch "$HOME/.Xauthority"
# realsense: librealsense + realsense-ros (RealSense is supported in Docker mode only).
# rover_perception: our layer with the FoundationStereo / FoundationPose packages; the CLI
# finds its Dockerfile through the config that sync_to_distro.sh installs.
cat > "$HOME/.config/isaac-ros-cli/config.yaml" << 'YAML'
docker:
  image:
    additional_image_keys:
      - realsense
      - rover_perception
YAML
EOF

echo "Isaac ROS CLI ready for '$DEV_USER' ($DEV_HOME/workspaces/isaac_ros-dev)"
