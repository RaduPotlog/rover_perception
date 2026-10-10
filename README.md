# rover_perception

Lightweight, CPU-first perception for the rover. It replaces the earlier Isaac ROS
FoundationStereo / FoundationPose kit (a ~50 GB Docker image, 5-8 GB of VRAM, 0.15-1.7 Hz,
x86 only). That kit is gone from the tree; it stays in the git history (commit `facba89`).

Everything runs natively in the normal colcon workspace: no Docker image of its own, no private
zenoh router, no CUDA requirement. It runs in `rover-a1-sensors`, next to the drivers it consumes:
`rover_sensors_bringup` starts the drivers (`rover_sensors`, the RealSense in `rover_realsense`),
then includes `rover_perception.launch.py`. Perception starts no driver itself.

| Package | What it does | Cost |
|---------|--------------|------|
| `rover_perception_bringup` | One launch file starting the parts below behind switches | - |
| `rover_perception_detection` | YOLO nano (ONNX Runtime) -> `detections` (`vision_msgs/Detection2DArray`) | CPU 2 threads, or about 0.3 GB VRAM with CUDA |
| `rover_perception_terrain` | RANSAC ground plane on `rslidar_points` -> `terrain/incline` (forward / lateral / slope in degrees) | one core at 5 Hz |
| `rover_perception_fmoc` | fmoc (follow-me on camera): Intel's ADBSCAN person clustering on `camera/depth/points` plus a tracker -> `tracked_person` (`rover_msgs/TrackedPerson`) | about a quarter of a Pi 5 core at 5 Hz (estimate) |
| *(bringup)* fiducials | `apriltag_ros` on the colour stream | one core at 15 Hz |

Camera: RealSense D435i, driven by `rover_sensors/rover_realsense` (colour, depth and the depth
point cloud). Its frames hang off `<namespace>/camera_link`, which the URDF (`rover_description`)
provides at an **assumed** mount pose until the real one is measured.

## Switches

All are environment variables, so on the rover they are balenaCloud fleet or device variables
(see `rover_docker/README.md`). The same names work as launch arguments (`use_camera:=true`).

| Variable | Default | Effect |
|----------|---------|--------|
| `ROVER_SYSTEM_USE_CAMERA` | `false` | the camera-fed nodes below may run (the driver itself is `rover_sensors`') |
| `ROVER_SENSORS_CAMERA_FIDUCIALS` | `false` | AprilTag |
| `ROVER_SENSORS_CAMERA_DETECTION` | `false` | object detection (needs `ROVER_SENSORS_CAMERA_DETECTION_MODEL`) |
| `ROVER_SENSORS_TERRAIN` | `false` | ground slope from the lidar |
| `ROVER_SYSTEM_FOLLOW_ME_ENABLE` | `false` | fmoc person tracking (`use_person_tracking`); the same variable starts follow-me in `rover-a1-orchestrator` |

Person tracking is not gated by `ROVER_SYSTEM_USE_CAMERA`: it only needs `camera/depth/points`, which
Gazebo publishes without the driver. Driver knobs (`ROVER_SENSORS_CAMERA_DEPTH_CLOUD`, `ROVER_SENSORS_CAMERA_FPS`,
`ROVER_SENSORS_CAMERA_DEPTH_PROFILE`) are `rover_realsense` arguments now.

## Run

`rover_perception_fmoc` uses `rover_msgs` (`rover_ros`), so clone that next to this repo.

```bash
sudo apt install ros-lyrical-apriltag-ros ros-lyrical-vision-msgs ros-lyrical-cv-bridge
pip install onnxruntime        # only for detection; onnxruntime-gpu for CUDA
colcon build --symlink-install --packages-up-to rover_perception_bringup rover_realsense
# the driver (rover_sensors), then perception on top of it
ros2 launch rover_realsense rover_realsense.launch.py
ros2 launch rover_perception_bringup rover_perception.launch.py use_camera:=true use_terrain:=true
```

On the rover both come from one launch: `rover_sensors_bringup/rover_sensors.launch.py`.

A D435i inside WSL2 needs a usbipd attach first: `rover_realsense/scripts/attach_camera.sh` (in
`rover_sensors`; `--once`, `--detach`). It then needs
`sudo chmod a+rw /dev/video* && sudo chmod -R a+rw /dev/bus/usb`; both reset on every re-attach.

Detection model, once, not committed (`models/*.onnx` is git-ignored):

```bash
yolo export model=yolo11n.pt format=onnx imgsz=640
```

## Cost on your machine

`rover_perception_bringup/scripts/benchmark.sh [seconds] [launch args]` launches the stack (with
`use_camera:=true` also the `rover_realsense` driver), then prints
per-process CPU (percent of one core) and memory plus the real topic rates. Run the same command on the
dev laptop and on the rover, ideally with the rest of the rover's stacks running.

Dev laptop (i9-13900HX), camera + depth cloud + AprilTag + terrain, 15 fps: about 21 % of one core in
total and 330 MB. With `camera_fps:=6 fiducials_decimate:=4.0`: 7 %. A Raspberry Pi 5 core is roughly
3-4x slower (an estimate, not measured). Knobs: `ROVER_SENSORS_CAMERA_FPS` (6, 15 or 30), `ROVER_SENSORS_CAMERA_DEPTH_PROFILE`,
`ROVER_SENSORS_CAMERA_FIDUCIALS_DECIMATE`, `ROVER_SENSORS_CAMERA_DETECTION_MAX_RATE`.

## Topics

| Topic | Type | From |
|-------|------|------|
| `camera/color/image_raw`, `camera/depth/image_rect_raw` | `sensor_msgs/Image` | realsense2_camera (`rover_realsense`, input) |
| `camera/depth/points` | `sensor_msgs/PointCloud2` | depth_image_proc (`rover_realsense`, input) |
| `detections` | `vision_msgs/Detection2DArray` | detection_node |
| `tag_detections` | `apriltag_msgs/AprilTagDetectionArray` | apriltag_node |
| `terrain/incline` | `geometry_msgs/Vector3Stamped`: x forward, y lateral, z total slope, degrees, positive = ground rising along +x / +y | terrain_node |
| `terrain/ground_confidence` | `std_msgs/Float32` (inlier ratio) | terrain_node |
| `tf` | AprilTag poses | apriltag_node |
| `tracked_person` | `rover_msgs/TrackedPerson`: state NONE/TRACKING/COASTING/LOST, position and velocity in `<namespace>/odom` | fmoc |
| `tracked_person/markers` | `visualization_msgs/MarkerArray`: clusters, acquire zone, target | fmoc |
| `tracked_person/reset` (service) | `std_srvs/Trigger`: drop the target | fmoc |

Camera and lidar streams are best-effort sensor data, per the workspace QoS rules.

## Person tracking (fmoc)

`fmoc` only perceives: it never drives. `rover_orchestrator`'s `rover_follow_me` turns a tracked
person into a Nav 2 `FollowObject` goal when asked to (`follow_me/start`); any other consumer can
use `tracked_person` too.

- **Acquire.** A person-sized cluster (0.15-1.0 m wide, at least 0.6 m tall) standing 1.5 m in
  front of the rover, within 0.6 m, while the rover stands still. `tracked_person/markers` shows
  the clusters, the acquire zone and the target.
- **Track.** In the global frame (odom), nearest cluster to a constant-velocity prediction, so the
  rover's own motion does not move the target.
- **Lost and found.** A briefly hidden person stays COASTING; a person-sized cluster near the last
  sighting is re-identified for 10 s.

Parameters: `rover_perception_fmoc/config/fmoc.yaml` (`/**/fmoc:`, the node runs in the rover
namespace).

### ADBSCAN

`rover_perception_fmoc/domain/adbscan.py` ports `adaptive_parameters()` (`doDBSCAN.cpp`), the
KD-tree `dbscan_adaptiveK()` (`dbscan_adaptiveK.cpp`) and the centroid/box helpers (`Util.cpp`)
from Intel's [follow-me component](https://github.com/open-edge-platform/edge-ai-suites/tree/main/robotics-ai-suite/components/adbscan)
(Apache-2.0, see `NOTICE`), keeping its semantics: per-point K and eps from the forward distance,
asymmetric neighbourhoods, border points that expand. Departures, each deliberate:

| Intel | Here | Why |
|-------|------|-----|
| Cloud rotated with a hardcoded optical-frame swap, mount ignored | TF `camera frame → base_link` | The real mount pose, any camera frame. |
| Ground filter off for RealSense (`Z_based_ground_removal: 0.05` truncates to 0) | Height band 0.15–1.9 m above `base_link` | The floor otherwise clusters with the feet. |
| Target kept in the camera frame; nearest centroid to the last one | Global frame, nearest to a constant-velocity prediction; acquisition needs a person-sized cluster and a still rover | The rover's own motion does not move the target; a wall or box is not picked up. |
| eps from the cloud's density only | Optional floor `min_eps + min_eps_per_m · x` (on in `fmoc.yaml`) | Alone in an open space a person at 3–4 m turned into noise: eps shrank below the subsampled point spacing. |
| PCL KD-tree | SciPy `cKDTree` (numpy fallback, same result) | Pure Python, arm64. |
| `cmd_vel` straight to the base | `TrackedPerson` → `rover_follow_me` → Nav 2 Following server → `cmd_vel_nav` | Nav 2's safety chain stays in the loop. |

Cost: with `filter.*_stride: 5` and `max_points: 2500`, a frame with a wall and two people
(about 2k points) takes 14 ms on a dev laptop with SciPy, 33 ms with the numpy fallback. A Pi 5
core is estimated (not measured) at 3–4× slower: about a quarter of a core at
`process_rate: 5.0`. Lower it, or raise the strides, if `fmoc` takes too much.

Limits: anyone person-sized is a person (no classification; YOLO's `person` class from
`rover_perception_detection` would fit as an option), and the camera pose comes from TF, whose
mount is still an ASSUMPTION in the URDF.

## Tests

```bash
cd rover_perception_terrain && PYTHONPATH=. python3 -m pytest test
cd rover_perception_detection && PYTHONPATH=. python3 -m pytest test
cd rover_perception_fmoc && PYTHONPATH=. python3 -m pytest test
```

`rover_perception_fmoc/test/unit/synthetic_scene.py` ray-casts a D435i-like depth cloud of people,
walls and floor; one ADBSCAN test needs SciPy.

The domain and application layers import no ROS, so these run without a sourced workspace.
