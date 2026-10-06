#!/usr/bin/env bash
# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.
#
# Stage 2: Docker Engine + NVIDIA Container Toolkit inside the Isaac ROS distro. Isaac ROS
# expects a native Docker Engine under systemd; Docker Desktop containers do not reliably see
# USB devices attached with usbipd.
#
#   wsl.exe -d Ubuntu-24.04 -u root -- bash -s -- [user] < setup_docker_gpu.sh
#   wsl.exe --terminate Ubuntu-24.04        # so the docker group membership takes effect
#
# Check afterwards, as the user:
#   docker run --rm --gpus all ubuntu:24.04 nvidia-smi
set -euo pipefail

DEV_USER="${1:-rover-a1}"

if [[ $EUID -ne 0 ]]; then
    echo "run as root: wsl.exe -d Ubuntu-24.04 -u root -- bash -s < $0" >&2
    exit 1
fi
if [[ "$(ps -p 1 -o comm=)" != systemd ]]; then
    echo "systemd is not PID 1: run setup_distro.sh, then wsl.exe --terminate Ubuntu-24.04" >&2
    exit 1
fi

export DEBIAN_FRONTEND=noninteractive
install -m 0755 -d /etc/apt/keyrings

# Docker Engine, official Ubuntu instructions.
curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
chmod a+r /etc/apt/keyrings/docker.asc
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] \
https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" \
    > /etc/apt/sources.list.d/docker.list

# NVIDIA Container Toolkit.
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
    | gpg --dearmor --yes -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
    | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
    > /etc/apt/sources.list.d/nvidia-container-toolkit.list

apt-get update
apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin \
    docker-compose-plugin nvidia-container-toolkit

nvidia-ctk runtime configure --runtime=docker
usermod -aG docker "$DEV_USER"
systemctl daemon-reload
systemctl restart docker

docker info | grep -E 'Runtimes|Default Runtime'
echo "docker ready; now: wsl.exe --terminate Ubuntu-24.04"
