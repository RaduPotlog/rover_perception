# Isaac ROS FoundationStereo + FoundationPose on the WSL2 laptop

NVIDIA Isaac ROS 5.0 (ROS 2 Lyrical) FoundationStereo (stereo depth) and FoundationPose
(6D object pose), fed by an Intel RealSense D435i, on this Windows laptop (RTX 4070 Laptop,
8 GB VRAM) under WSL2. WSL2 is not an NVIDIA-supported Isaac ROS platform; everything below
was verified on this machine on 2026-09-30.

## Layout

```
Windows ── usbipd-win ──► WSL2 VM (one kernel, shared by all distros)
                             ├─ Ubuntu        rover workspace (26.04, ROS Lyrical)   ← edit here
                             └─ Ubuntu-24.04  Isaac ROS distro (D:\WSL\isaac-ros)
                                  Docker Engine + NVIDIA runtime
                                  └─ isaac_ros_dev_container  (isaac_ros + realsense + rover_perception)
                                       /workspaces/isaac_ros-dev = ~/workspaces/isaac_ros-dev
```

Why a second distro: Isaac ROS 5.0 only publishes a `noble` (24.04) apt suite, and RealSense
is supported only in its Docker mode, which expects Docker Engine (not Docker Desktop).

| Path (this repo) | Purpose |
|---|---|
| `scripts/setup_distro.sh` | Stage 1: user, systemd, binfmt protection (see Pitfalls) |
| `scripts/setup_docker_gpu.sh` | Stage 2: Docker Engine + NVIDIA Container Toolkit |
| `scripts/setup_isaac_ros.sh` | Stage 3: Isaac ROS apt repo + CLI, RealSense udev rule, CLI config |
| `scripts/sync_to_distro.sh` | Copy this directory into the Isaac ROS distro (one-way) |
| `docker/Dockerfile.rover_perception` | Image layer: FoundationStereo/Pose, RT-DETR, RViz, rmw_zenoh |
| `config/isaac_ros_common-config` | Points the Isaac ROS CLI at `docker/` |
| `scripts/attach_camera.sh` | usbipd attach/detach of the D435i |
| `scripts/isaac_container.sh` | Start / exec / stop the dev container without a terminal |
| `scripts/install_models.sh` | Assets, ONNX, TensorRT engines |
| `scripts/run_foundationstereo.sh`, `scripts/run_foundationpose.sh` | Launch wrappers (inside the container) |

## Setup from zero

From a terminal in the rover distro (`Ubuntu`), in this directory:

```bash
# 1. Distro (no admin needed)
wsl.exe --install Ubuntu-24.04 --no-launch --location 'D:\WSL\isaac-ros'
wsl.exe -d Ubuntu-24.04 -u root -- bash -s -- rover-a1 < scripts/setup_distro.sh
wsl.exe --terminate Ubuntu-24.04
# 2. Docker + GPU
wsl.exe -d Ubuntu-24.04 -u root -- bash -s -- rover-a1 < scripts/setup_docker_gpu.sh
wsl.exe --terminate Ubuntu-24.04
# 3. Isaac ROS CLI, then copy this directory over
wsl.exe -d Ubuntu-24.04 -u root -- bash -s -- rover-a1 < scripts/setup_isaac_ros.sh
scripts/sync_to_distro.sh
```

Run `wsl.exe` from a Windows-side directory (`cd /mnt/c`) if it warns that it cannot translate
the working directory. Then, in a terminal of the Isaac ROS distro (`wsl -d Ubuntu-24.04`):

```bash
isaac-ros activate --build-local --build-only   # builds the image locally: ~50 GB, 1–2 h
```

Camera, once, from an **elevated** Windows prompt (this laptop has USBPcap, hence `--force`):

```
usbipd bind --busid <BUSID of the D435i> --force
```

The bind is per port, so keep the camera on the same port. Use a USB 3 port **and cable**: with
a USB 2 cable it enumerates at 480 Mbit/s on every port (`lsusb -v` shows `bcdUSB 2.10`).

## Daily use

```bash
# rover distro: attach the camera BEFORE the container starts
scripts/attach_camera.sh --once           # or without --once: keeps re-attaching, blocks
# Isaac ROS distro
isaac-ros activate                        # interactive shell; run it again for a second shell
```

Inside the container:

```bash
cd /workspaces/isaac_ros-dev/src/rover_perception/isaac_ros/scripts
./install_models.sh all                   # once; engines take tens of minutes
./run_foundationstereo.sh rosbag|realsense
./run_foundationpose.sh rosbag|rosbag-tracking|realsense|realsense-tracking
```

Only one of the two models at a time: the 8 GB GPU does not hold both.

## Measured on this laptop (2026-10-01)

RTX 4070 Laptop, 8 GB. **The GPU was held at a 30 W power limit** (default 115 W), so these
are lower bounds; check `nvidia-smi -q -d POWER` and the laptop's power/thermal profile first.

| Pipeline | Input | Output rate | Peak GPU memory |
|---|---|---|---|
| FoundationStereo 320x736 **FP32** | rosbag | — crashes (`cuMemSetAccess … CUDA_ERROR_NOT_READY`) | ~7.7 GB (context alone 7.1 GB) |
| FoundationStereo 320x736 FP16 | rosbag | `/disparity` 1.2 Hz | 5.8 GB |
| FoundationStereo 320x736 FP16 | D435i IR 640x360 @ 15 fps | `/disparity` 1.2 Hz | 4.7 GB |
| FoundationPose estimation | rosbag | `/output` 0.15 Hz (one pose / 6.7 s) | 7.9 GB |
| FoundationPose tracking | rosbag | `/tracking/output` 1.7 Hz | 7.9 GB |
| FoundationPose + RT-DETR | D435i colour + aligned depth @ 15 fps | detections 1 Hz; no pose (no known object in view) | 5.1 GB |

TensorRT benchmark (`trtexec`, same power cap): FoundationStereo FP32 1.86 s, FP16 0.82 s per
frame; RT-DETR `sdetr_grasp` 28 ms; FoundationPose refine 5.3 ms, score 4.9 ms per batch.

FoundationPose needs an object the RT-DETR `sdetr_grasp` model knows (the quickstart meshes:
`Mac_and_cheese_0_1`, `Mustard`, `soup_can`, …) for its first-frame mask. A custom object needs
its own `.obj` mesh (`MESH=...`) and a different mask source.

## Pitfalls found on this machine

- **Windows interop breaks when the second distro stops.** binfmt_misc is shared by all WSL
  distros, and systemd (systemd-binfmt, and systemd-shutdown on every idle stop) clears it:
  `*.exe` then fails in the rover distro with "Exec format error". `setup_distro.sh` masks the
  binfmt units and unmounts binfmt_misc in the Isaac ROS distro (`wsl-keep-binfmt.service`).
  Recovery: `/init /mnt/c/WINDOWS/system32/wsl.exe wsl.exe -d Ubuntu-24.04 -u root -- true`.
- **FastDDS discovery does not work between processes in the container** (nor on the rover
  distro). The image starts a private rmw_zenoh router on `127.0.0.1:7448` and every shell uses
  `rmw_zenoh_cpp` (`/etc/profile.d/50-rover-zenoh.sh`). Port 7447 is left for the rover / sim.
- **Do not put files into `/etc/isaac-ros-cli`.** The base layer bind-mounts it as build
  context; any new file there rebuilds the base and RealSense layers.
- **Slow or flaky downloads.** The layer keeps apt downloads in a cache mount (the Ubuntu base's
  `docker-clean` is removed) and retries; pip gets long timeouts, retries and a cache for the
  torch/onnx "pip shim" packages that `isaac_ros_foundationstereo` pulls in.
- **librealsense in this image is built with CUDA:** run RealSense tools with `--gpus all`.
- **Camera firmware** is 5.17.3.10; Isaac ROS pins 5.16.0.1. Streams work; not downgraded.
- **The camera detaches when the laptop sleeps.** Re-run `attach_camera.sh --once`, then restart
  the container (`isaac_container.sh stop && isaac_container.sh start`, or exit and re-run
  `isaac-ros activate`): a running container never sees a re-attached device ("No RealSense
  devices were found!").
- **FoundationStereo FP32 does not fit in 8 GB**; `install_models.sh` builds an FP16 engine and
  `run_foundationstereo.sh` uses it. That build needs ~10 GB of host RAM.
- **`timeout` does not stop ROS 2 CLI tools under rmw_zenoh** (rclpy ignores SIGTERM): use
  `timeout -s INT -k 5 <secs> ros2 topic hz ...`.
- The Isaac ROS CLI cannot start the container without a TTY; use `scripts/isaac_container.sh`.
