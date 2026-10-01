#!/usr/bin/env bash
# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.
#
# FoundationPose with the RT-DETR (sdetr_grasp) initial mask. Run INSIDE the Isaac ROS container.
#
#   run_foundationpose.sh rosbag             # then: ros2 bag play -l .../isaac_ros_foundationpose/quickstart.bag/
#   run_foundationpose.sh rosbag-tracking    # then: ros2 bag play -l .../isaac_ros_foundationpose/foundationpose_tracking.bag/
#   run_foundationpose.sh realsense          # D435i colour + aligned depth, 640x480 @ 15 fps
#   run_foundationpose.sh realsense-tracking
#
# MESH=<file.obj> selects the object (default: the quickstart mesh of each mode). Extra
# arguments are passed to the launch file. Output: pose_estimation/output (Detection3DArray)
# and the TF frame fp_object. Peak VRAM is ~7 GB: close GPU apps on Windows first, and do not
# run together with FoundationStereo.
set -euo pipefail

MODE="${1:-}"
shift || true
: "${ISAAC_ROS_WS:?run inside the Isaac ROS container}"
ASSETS="${ISAAC_ROS_WS%/}/isaac_ros_assets"
QUICKSTART="$ASSETS/isaac_ros_foundationpose"

ARGS=(
    score_engine_file_path:="$ASSETS/models/foundationpose/score_trt_engine.plan"
    refine_engine_file_path:="$ASSETS/models/foundationpose/refine_trt_engine.plan"
    rt_detr_engine_file_path:="$ASSETS/models/synthetica_detr/sdetr_grasp.plan"
)
case "$MODE" in
    rosbag | rosbag-tracking)
        FRAGMENTS=foundationpose
        MESH="${MESH:-$QUICKSTART/Mustard/textured_simple.obj}"
        ARGS+=(interface_specs_file:="$QUICKSTART/quickstart_interface_specs.json")
        ;;
    realsense | realsense-tracking)
        FRAGMENTS=realsense_mono_rect_depth,foundationpose
        MESH="${MESH:-$QUICKSTART/Mac_and_cheese_0_1/Mac_and_cheese_0_1.obj}"
        ;;
    *)
        sed -n '5,15p' "$0" | sed 's/^# \{0,1\}//'
        exit 1
        ;;
esac
[[ $MODE == *-tracking ]] && FRAGMENTS="${FRAGMENTS}_tracking"
for f in "$MESH" "$ASSETS/models/foundationpose/score_trt_engine.plan" \
    "$ASSETS/models/synthetica_detr/sdetr_grasp.plan"; do
    [[ -e "$f" ]] || { echo "missing $f: run install_models.sh pose" >&2; exit 1; }
done

exec ros2 launch isaac_ros_examples isaac_ros_examples.launch.py \
    launch_fragments:="$FRAGMENTS" mesh_file_path:="$MESH" "${ARGS[@]}" "$@"
