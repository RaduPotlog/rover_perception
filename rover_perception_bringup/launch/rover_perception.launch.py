# Copyright 2026 Mechatronics Academy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Lightweight perception: one switch per part, balenaCloud variables as defaults.

The sensor drivers are not started here: rover_sensors does that (the RealSense in
rover_realsense), and rover_sensors_bringup includes this file on top of them. In Gazebo the
simulator publishes the same topics.

ROVER_SYSTEM_USE_CAMERA         camera-fed nodes below may run (default false)
ROVER_SENSORS_CAMERA_FIDUCIALS  AprilTag detection on the colour stream (default false)
ROVER_SENSORS_CAMERA_DETECTION  YOLO object detection on the colour stream (default false)
ROVER_SENSORS_TERRAIN           ground slope from the lidar cloud, no camera needed (default false)
ROVER_SYSTEM_FOLLOW_ME_ENABLE   fmoc person tracking on camera/depth/points -> tracked_person,
                                for rover_orchestrator's follow-me (default false)

Cost knobs, for a loaded controller (all optional):
ROVER_SENSORS_CAMERA_FIDUCIALS_DECIMATE  AprilTag decimation, higher = cheaper (default 2.0)
ROVER_SENSORS_CAMERA_DETECTION_MAX_RATE  detections per second (default 10.0)

The camera sub-switches only matter while ROVER_SYSTEM_USE_CAMERA is true. Person tracking is not
tied to it: it only needs a depth cloud, which Gazebo provides without the RealSense driver.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def env_flag(value):
    """True for true/1/yes/on (any case): the values usually come straight from balena."""
    return PythonExpression(["'", value, "'.strip().lower() in ('true', '1', 'yes', 'on')"])


def flag_arg(name, env, default):
    return DeclareLaunchArgument(
        name, default_value=EnvironmentVariable(env, default_value=default),
        description=f'true/false, default from ${env}.')


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_camera = LaunchConfiguration('use_camera')
    use_fiducials = LaunchConfiguration('use_fiducials')
    use_detection = LaunchConfiguration('use_detection')
    use_terrain = LaunchConfiguration('use_terrain')
    use_person_tracking = LaunchConfiguration('use_person_tracking')
    use_sim_time = LaunchConfiguration('use_sim_time')

    def camera_and(flag):
        return IfCondition(PythonExpression([env_flag(use_camera), ' and ', env_flag(flag)]))

    apriltag = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',  # must match the key in config/apriltag.yaml
        namespace=namespace,
        condition=camera_and(use_fiducials),
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare('rover_perception_bringup'), 'config', 'apriltag.yaml']),
            {'use_sim_time': use_sim_time,
             'detector.decimate': LaunchConfiguration('fiducials_decimate')},
        ],
        remappings=[
            ('image_rect', 'camera/color/image_raw'),
            ('camera_info', 'camera/color/camera_info'),
            # `detections` is the YOLO node's vision_msgs topic; keep the tag array apart.
            ('detections', 'tag_detections'),
        ],
        output='screen',
    )

    detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('rover_perception_detection'), 'launch', 'detection.launch.py'])),
        condition=camera_and(use_detection),
        launch_arguments={
            'namespace': namespace,
            'image_topic': 'camera/color/image_raw',
            'model_path': LaunchConfiguration('detection_model'),
            'use_gpu': LaunchConfiguration('detection_use_gpu'),
            'max_rate_hz': LaunchConfiguration('detection_max_rate'),
            'use_sim_time': use_sim_time,
        }.items(),
    )

    terrain = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('rover_perception_terrain'), 'launch', 'terrain.launch.py'])),
        condition=IfCondition(env_flag(use_terrain)),
        launch_arguments={'namespace': namespace, 'use_sim_time': use_sim_time}.items(),
    )

    person_tracking = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('rover_perception_fmoc'), 'launch', 'fmoc.launch.py'])),
        condition=IfCondition(env_flag(use_person_tracking)),
        launch_arguments={'namespace': namespace, 'use_sim_time': use_sim_time}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value=EnvironmentVariable('ROVER_SYSTEM_NAMESPACE', default_value='')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        flag_arg('use_camera', 'ROVER_SYSTEM_USE_CAMERA', 'false'),
        flag_arg('use_fiducials', 'ROVER_SENSORS_CAMERA_FIDUCIALS', 'false'),
        flag_arg('use_detection', 'ROVER_SENSORS_CAMERA_DETECTION', 'false'),
        flag_arg('use_terrain', 'ROVER_SENSORS_TERRAIN', 'false'),
        flag_arg('use_person_tracking', 'ROVER_SYSTEM_FOLLOW_ME_ENABLE', 'false'),
        DeclareLaunchArgument(
            'detection_model',
            default_value=EnvironmentVariable(
                'ROVER_SENSORS_CAMERA_DETECTION_MODEL', default_value=''),
            description='Path of the YOLO .onnx file.'),
        DeclareLaunchArgument(
            'fiducials_decimate',
            default_value=EnvironmentVariable(
                'ROVER_SENSORS_CAMERA_FIDUCIALS_DECIMATE', default_value='2.0')),
        DeclareLaunchArgument(
            'detection_max_rate',
            default_value=EnvironmentVariable(
                'ROVER_SENSORS_CAMERA_DETECTION_MAX_RATE', default_value='10.0')),
        DeclareLaunchArgument(
            'detection_use_gpu',
            default_value=EnvironmentVariable(
                'ROVER_SENSORS_CAMERA_DETECTION_GPU', default_value='false')),
        apriltag,
        detection,
        terrain,
        person_tracking,
    ])
