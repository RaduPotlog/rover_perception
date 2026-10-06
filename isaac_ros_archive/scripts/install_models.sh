#!/usr/bin/env bash
# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.
#
# Stage 4: quickstart assets, models and TensorRT engines, all under
# ${ISAAC_ROS_WS}/isaac_ros_assets, a host directory the --rm container mounts. Idempotent:
# finished pieces are skipped, and interrupted downloads resume.
#
#   install_models.sh download               # assets + ONNX only; also runs on the host distro
#   install_models.sh [stereo|pose|all]      # in the Isaac ROS container (default: all)
#
# The NVIDIA installers show each model's EULA and ask y/n; this script answers "y", i.e. running
# it accepts the FoundationStereo and SyntheticaDETR model licences.
# FoundationStereo is installed in low_res (320x736): high_res needs more than 16 GB of VRAM.
# FoundationPose engines are FP32 (it loses accuracy in FP16); FoundationStereo also gets FP16.
set -euo pipefail

WHAT="${1:-all}"
ISAAC_ROS_WS="${ISAAC_ROS_WS:-$HOME/workspaces/isaac_ros-dev/}"
ASSETS="${ISAAC_ROS_WS%/}/isaac_ros_assets"
DOWNLOADS="$ASSETS/.downloads"
mkdir -p "$ASSETS" "$DOWNLOADS"

fetch() { # <url> <file>; resumes a partial file
    curl -L --fail --retry 20 --retry-all-errors --retry-delay 5 -C - -o "$2" "$1"
}

# Latest asset version that matches Isaac ROS 5.0, as in the quickstart pages.
ngc_asset() { # <resource> <file> <marker path that exists once extracted>
    local resource="$1" file="$2" marker="$3"
    [[ -e "$ASSETS/$marker" ]] && return 0
    local base="https://api.ngc.nvidia.com/v2/resources/nvidia/isaac/$resource/versions"
    local version
    version="$(curl -s -H 'Accept: application/json' "$base" | jq -r '
        .recipeVersions[].versionId
        | select(test("^\\d+\\.\\d+\\.\\d+$"))
        | select((split(".") | map(tonumber)) as $v | $v[0] == 5 and $v[1] <= 0)' \
        | sort -V | tail -n 1)"
    [[ -n "$version" ]] || { echo "no 5.0 version of $resource on NGC" >&2; return 1; }
    echo "downloading $resource $version/$file"
    local archive="$DOWNLOADS/${resource}_${version}_$file"
    fetch "$base/$version/files/$file" "$archive"
    tar -xf "$archive" -C "$ASSETS"
    rm "$archive"
}

FP_DIR="$ASSETS/models/foundationpose"
FP_URL=https://api.ngc.nvidia.com/v2/models/nvidia/isaac/foundationpose/versions/1.0.1_onnx/files

download() {
    ngc_asset isaac_ros_foundationstereo_assets quickstart.tar.gz isaac_ros_foundationstereo
    ngc_asset isaac_ros_rtdetr_assets quickstart.tar.gz isaac_ros_rtdetr
    ngc_asset isaac_ros_foundationpose_assets quickstart.tar.gz isaac_ros_foundationpose/quickstart.bag
    ngc_asset isaac_ros_foundationpose_assets foundationpose_tracking.tar.gz \
        isaac_ros_foundationpose/foundationpose_tracking.bag
    mkdir -p "$FP_DIR"
    local name
    for name in refine score; do
        [[ -e "$FP_DIR/${name}_model.onnx" ]] && continue
        fetch "$FP_URL/${name}_model.onnx" "$DOWNLOADS/${name}_model.onnx"
        mv "$DOWNLOADS/${name}_model.onnx" "$FP_DIR/${name}_model.onnx"
    done
}

install_stereo() {
    local dir="$ASSETS/models/foundationstereo/deployable_v2.0"
    [[ -e "$dir/foundationstereo_320x736.engine" ]] || echo y | ros2 run \
        isaac_ros_foundationstereo_models_install install_foundationstereo_models.sh --eula --model_res low_res
    # NVIDIA's installer builds FP32, which needs ~7.1 GB of GPU memory: too much for 8 GB.
    # FP16 needs ~3.3 GB. Its build takes ~25 min and ~10 GB of host RAM (WSL is capped at 16 GB).
    [[ -e "$dir/foundationstereo_320x736_fp16.engine" ]] || /usr/src/tensorrt/bin/trtexec \
        --onnx="$dir/foundationstereo_320x736.onnx" \
        --saveEngine="$dir/foundationstereo_320x736_fp16.engine" --fp16
}

install_pose() {
    # Initial-mask detector (SyntheticaDETR sdetr_grasp): only knows a fixed set of objects.
    [[ -e "$ASSETS/models/synthetica_detr/sdetr_grasp.plan" ]] \
        || echo y | ros2 run isaac_ros_rtdetr_models_install install_rtdetr_models.sh --eula

    local name batch
    for name in refine score; do
        [[ -e "$FP_DIR/${name}_trt_engine.plan" ]] && continue
        batch=42
        [[ $name == score ]] && batch=252
        /usr/src/tensorrt/bin/trtexec \
            --onnx="$FP_DIR/${name}_model.onnx" \
            --saveEngine="$FP_DIR/${name}_trt_engine.plan" \
            --minShapes=input1:1x160x160x6,input2:1x160x160x6 \
            --optShapes=input1:1x160x160x6,input2:1x160x160x6 \
            --maxShapes=input1:${batch}x160x160x6,input2:${batch}x160x160x6
    done
}

# One step per line: `set -e` does not apply inside functions called from an && list.
case "$WHAT" in
    download) download ;;
    stereo)
        download
        install_stereo
        ;;
    pose)
        download
        install_pose
        ;;
    all)
        download
        install_stereo
        install_pose
        ;;
    *) sed -n '5,15p' "$0" | sed 's/^# \{0,1\}//'; exit 1 ;;
esac
find "$ASSETS" -maxdepth 4 \( -name '*.engine' -o -name '*.plan' -o -name '*.onnx' \) -exec ls -lh {} +
