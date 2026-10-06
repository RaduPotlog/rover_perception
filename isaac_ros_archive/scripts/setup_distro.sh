#!/usr/bin/env bash
# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.
#
# Stage 1: prepare the Isaac ROS WSL distro (Ubuntu 24.04). Isaac ROS 5.0 only ships a `noble`
# apt suite, so it cannot live in the rover's Ubuntu 26.04 distro.
#
# Install the distro from Windows first (no admin needed):
#   wsl.exe --install Ubuntu-24.04 --no-launch --location 'D:\WSL\isaac-ros'
# then run this script in it as root, from the rover distro:
#   wsl.exe -d Ubuntu-24.04 -u root -- bash -s -- [user] < setup_distro.sh
#   wsl.exe --terminate Ubuntu-24.04        # so systemd and the default user take effect
#
# The user is created without a password; privileged steps of the other scripts run with
# `wsl.exe -u root`. For interactive sudo, set one: wsl.exe -d Ubuntu-24.04 -u root passwd <user>.
# Do NOT enable Docker Desktop's WSL integration for this distro: it uses its own Docker Engine.
set -euo pipefail

DEV_USER="${1:-rover-a1}"

if [[ $EUID -ne 0 ]]; then
    echo "run as root: wsl.exe -d Ubuntu-24.04 -u root -- bash -s < $0" >&2
    exit 1
fi

if ! id "$DEV_USER" > /dev/null 2>&1; then
    useradd --create-home --shell /bin/bash --groups sudo,plugdev,video "$DEV_USER"
fi

cat > /etc/wsl.conf << EOF
[boot]
systemd=true

[user]
default=$DEV_USER
EOF

# binfmt_misc registrations are shared by every WSL distro. Both systemd-binfmt (start and stop)
# and systemd-shutdown (every idle stop of this distro, ~15 s after its last shell exits) clear
# them, which unregisters WSLInterop and breaks *.exe launching in the rover distro ("cannot
# execute binary file: Exec format error"). So: no systemd-binfmt here, and binfmt_misc is
# unmounted in this distro at boot, which makes systemd-shutdown skip it. *.exe still runs here.
# Recovery if it happens anyway, from the rover distro:
#   /init /mnt/c/WINDOWS/system32/wsl.exe wsl.exe -d Ubuntu-24.04 -u root -- true
cat > /etc/systemd/system/wsl-keep-binfmt.service << 'EOF'
[Unit]
Description=Keep the binfmt_misc registrations shared by all WSL distros out of reach of systemd-shutdown
DefaultDependencies=no
Before=sysinit.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/bin/sh -c "test -e /proc/sys/fs/binfmt_misc/WSLInterop || echo :WSLInterop:M::MZ::/init:PF > /proc/sys/fs/binfmt_misc/register; umount /proc/sys/fs/binfmt_misc"

[Install]
WantedBy=sysinit.target
EOF
systemctl mask systemd-binfmt.service proc-sys-fs-binfmt_misc.automount proc-sys-fs-binfmt_misc.mount
systemctl daemon-reload
systemctl enable --now wsl-keep-binfmt.service

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-install-recommends \
    ca-certificates curl gnupg locales software-properties-common usbutils wget
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
add-apt-repository -y universe

echo "distro ready for user '$DEV_USER'; now: wsl.exe --terminate Ubuntu-24.04"
