#!/usr/bin/env bash
# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.
#
# FoundationStereo (320x736, FP16 engine). Run INSIDE the Isaac ROS container.
# The FP32 engine NVIDIA's installer builds needs ~7.1 GB of GPU memory and crashes the pipeline
# on this 8 GB GPU; the FP16 one (built by install_models.sh) needs ~3.3 GB and is twice as fast.
#
#   run_foundationstereo.sh rosbag       # then, in a second shell of the container:
#       ros2 bag play -l ${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_foundationstereo/rosbags/foundationstereo_rosbag \
#           --remap /left/camera_info:=/left/camera_info_rect /right/camera_info:=/right/camera_info_rect
#   run_foundationstereo.sh realsense    # D435i IR pair, 640x360 @ 15 fps, emitter off
#
# Extra arguments are passed to the launch file. Output: /disparity (stereo_msgs/DisparityImage).
# Do not run together with FoundationPose: the 8 GB GPU does not hold both.
set -euo pipefail

MODE="${1:-}"
shift || true
: "${ISAAC_ROS_WS:?run inside the Isaac ROS container}"
ENGINE="${FOUNDATIONSTEREO_ENGINE:-${ISAAC_ROS_WS%/}/isaac_ros_assets/models/foundationstereo/deployable_v2.0/foundationstereo_320x736_fp16.engine}"
[[ -e "$ENGINE" ]] || { echo "missing $ENGINE: run install_models.sh stereo" >&2; exit 1; }

ARGS=(engine_file_path:="$ENGINE" model_input_width:=736 model_input_height:=320)
case "$MODE" in
    rosbag)
        FRAGMENTS=foundationstereo
        ;;
    realsense)
        FRAGMENTS=realsense_stereo_rect,foundationstereo
        # The default 90 fps outruns inference and stalls the disparity output within a minute.
        ARGS+=(realsense_config_file:="$(ros2 pkg prefix isaac_ros_foundationstereo --share)/config/realsense_stereo_low_fps.yaml")
        ;;
    *)
        sed -n '5,15p' "$0" | sed 's/^# \{0,1\}//'
        exit 1
        ;;
esac

exec ros2 launch isaac_ros_examples isaac_ros_examples.launch.py \
    launch_fragments:="$FRAGMENTS" "${ARGS[@]}" "$@"
