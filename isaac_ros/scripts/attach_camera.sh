#!/usr/bin/env bash
# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.
#
# Stage 5: hand the RealSense D435i to WSL with usbipd-win. Run from any WSL distro, BEFORE
# `isaac-ros activate` (the container only sees USB devices present when it starts).
#
#   attach_camera.sh            # attach and keep re-attaching (blocks; leave the terminal open)
#   attach_camera.sh --once     # attach once and return
#   attach_camera.sh --detach   # give the camera back to Windows
#
# One-time, from an elevated Windows prompt (this laptop has USBPcap, hence --force):
#   usbipd bind --busid <BUSID> --force
# The attachment is lost on unplug and on `wsl --shutdown`; while attached, Windows cannot use
# the camera. Every WSL distro sees the device, but only one process can open it.
set -euo pipefail

DISTRO="${ISAAC_ROS_DISTRO:-Ubuntu-24.04}"
VID_PID=8086:0b3a # Intel RealSense D435i
USBIPD="/mnt/c/Program Files/usbipd-win/usbipd.exe"

BUSID="$("$USBIPD" list 2> /dev/null | tr -d '\r' | awk -v id="$VID_PID" '$2 == id { print $1; exit }')"
if [[ -z "$BUSID" ]]; then
    echo "no D435i ($VID_PID) connected to Windows" >&2
    exit 1
fi

case "${1:-}" in
    --detach) exec "$USBIPD" detach --busid "$BUSID" ;;
    --once) exec "$USBIPD" attach --wsl "$DISTRO" --busid "$BUSID" ;;
    "") exec "$USBIPD" attach --wsl "$DISTRO" --busid "$BUSID" --auto-attach ;;
    *) sed -n '5,16p' "$0" | sed 's/^# \{0,1\}//'; exit 1 ;;
esac
