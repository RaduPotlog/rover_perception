# rover_perception

Lightweight, CPU-first perception for the rover. It replaces the earlier Isaac ROS
FoundationStereo / FoundationPose kit (a ~50 GB Docker image, 5-8 GB of VRAM, 0.15-1.7 Hz,
x86 only). That kit is gone from the tree; it stays in the git history (commit `facba89`).

Everything runs natively in the normal colcon workspace: no Docker image of its own, no private
zenoh router, no CUDA requirement.

| Package | What it does | Cost |
|---------|--------------|------|
| `rover_perception_bringup` | One launch file starting the parts below behind switches | - |
| `rover_perception_detection` | YOLO nano (ONNX Runtime) -> `detections` (`vision_msgs/Detection2DArray`) | CPU 2 threads, or about 0.3 GB VRAM with CUDA |
| `rover_perception_terrain` | RANSAC ground plane on `rslidar_points` -> `terrain/incline` (forward / lateral / slope in degrees) | one core at 5 Hz |
| *(bringup)* depth cloud | `depth_image_proc` turns the D435i depth (424x240 at 15 Hz) into `camera/depth/points`, a local-costmap source | a few percent of a core |
| *(bringup)* fiducials | `apriltag_ros` on the colour stream | one core at 15 Hz |

Camera: RealSense D435i via `realsense2_camera`. Its frames hang off `camera_link`, which the URDF
(`rover_description`) provides at an **assumed** mount pose until the real one is measured.

## Switches

All are environment variables, so on the rover they are balenaCloud fleet or device variables
(see `rover_docker/README.md`). The same names work as launch arguments (`use_camera:=true`).

| Variable | Default | Effect |
|----------|---------|--------|
| `ROVER_USE_CAMERA` | `false` | RealSense driver and everything fed by it; also adds the depth cloud to the local costmap |
| `ROVER_CAMERA_DEPTH_CLOUD` | `true` | depth -> `camera/depth/points` |
| `ROVER_CAMERA_FIDUCIALS` | `false` | AprilTag |
| `ROVER_CAMERA_DETECTION` | `false` | object detection (needs `ROVER_CAMERA_DETECTION_MODEL`) |
| `ROVER_USE_TERRAIN` | `false` | ground slope from the lidar |

## Run

```bash
sudo apt install ros-lyrical-realsense2-camera ros-lyrical-depth-image-proc \
  ros-lyrical-apriltag-ros ros-lyrical-vision-msgs ros-lyrical-cv-bridge
pip install onnxruntime        # only for detection; onnxruntime-gpu for CUDA
colcon build --symlink-install --packages-up-to rover_perception_bringup
ros2 launch rover_perception_bringup rover_perception.launch.py use_camera:=true use_terrain:=true
```

A D435i inside WSL2 needs a usbipd attach first: `rover_perception_bringup/scripts/attach_camera.sh`
(`--once`, `--detach`). It then needs `sudo chmod a+rw /dev/video* && sudo chmod -R a+rw /dev/bus/usb`;
both reset on every re-attach.

Detection model, once, not committed (`models/*.onnx` is git-ignored):

```bash
yolo export model=yolo11n.pt format=onnx imgsz=640
```

## Cost on your machine

`rover_perception_bringup/scripts/benchmark.sh [seconds] [launch args]` launches the stack, then prints
per-process CPU (percent of one core) and memory plus the real topic rates. Run the same command on the
dev laptop and on the rover, ideally with the rest of the rover's stacks running.

Dev laptop (i9-13900HX), camera + depth cloud + AprilTag + terrain, 15 fps: about 21 % of one core in
total and 330 MB. With `camera_fps:=6 fiducials_decimate:=4.0`: 7 %. A Raspberry Pi 5 core is roughly
3-4x slower (an estimate, not measured). Knobs: `ROVER_CAMERA_FPS` (6, 15 or 30), `ROVER_CAMERA_DEPTH_PROFILE`,
`ROVER_CAMERA_FIDUCIALS_DECIMATE`, `ROVER_CAMERA_DETECTION_MAX_RATE`.

## Topics

| Topic | Type | From |
|-------|------|------|
| `camera/color/image_raw`, `camera/depth/image_rect_raw` | `sensor_msgs/Image` | realsense2_camera |
| `camera/depth/points` | `sensor_msgs/PointCloud2` | depth_image_proc |
| `detections` | `vision_msgs/Detection2DArray` | detection_node |
| `tag_detections` | `apriltag_msgs/AprilTagDetectionArray` | apriltag_node |
| `terrain/incline` | `geometry_msgs/Vector3Stamped`: x forward, y lateral, z total slope, degrees, positive = ground rising along +x / +y | terrain_node |
| `terrain/ground_confidence` | `std_msgs/Float32` (inlier ratio) | terrain_node |
| `tf` | AprilTag poses | apriltag_node |

Camera and lidar streams are best-effort sensor data, per the workspace QoS rules.

## Tests

```bash
cd rover_perception_terrain && PYTHONPATH=. python3 -m pytest test
cd rover_perception_detection && PYTHONPATH=. python3 -m pytest test
```

The domain and application layers import no ROS, so these run without a sourced workspace.
