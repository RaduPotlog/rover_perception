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

ROVER_USE_CAMERA             RealSense D435i driver and everything fed by it (default false)
ROVER_CAMERA_DEPTH_CLOUD     depth -> PointCloud2 for the Nav 2 costmaps (default true)
ROVER_CAMERA_FIDUCIALS       AprilTag detection on the colour stream (default false)
ROVER_CAMERA_DETECTION       YOLO object detection on the colour stream (default false)
ROVER_USE_TERRAIN            ground slope from the lidar cloud, no camera needed (default false)

Cost knobs, for a loaded controller (all optional):
ROVER_CAMERA_FPS                  colour and depth frame rate (default 15)
ROVER_CAMERA_DEPTH_PROFILE        depth resolution WxH (default 424x240)
ROVER_CAMERA_FIDUCIALS_DECIMATE   AprilTag decimation, higher = cheaper (default 2.0)
ROVER_CAMERA_DETECTION_MAX_RATE   detections per second (default 10.0)

The camera sub-switches only matter while ROVER_USE_CAMERA is true.
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
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
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
    use_depth_cloud = LaunchConfiguration('use_depth_cloud')
    use_fiducials = LaunchConfiguration('use_fiducials')
    use_detection = LaunchConfiguration('use_detection')
    use_terrain = LaunchConfiguration('use_terrain')
    use_sim_time = LaunchConfiguration('use_sim_time')
    camera_fps = LaunchConfiguration('camera_fps')
    depth_profile = LaunchConfiguration('depth_profile')

    def camera_and(flag):
        return IfCondition(PythonExpression([env_flag(use_camera), ' and ', env_flag(flag)]))

    # Depth at 424x240 / 15 Hz is plenty for a 3 m costmap source and keeps USB and CPU load low.
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])),
        condition=IfCondition(env_flag(use_camera)),
        launch_arguments={
            'camera_name': 'camera',
            'camera_namespace': namespace,
            'depth_module.depth_profile': [depth_profile, 'x', camera_fps],
            'rgb_camera.color_profile': ['640x480x', camera_fps],
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'pointcloud.enable': 'false',  # depth_image_proc below does it, only when wanted
            'align_depth.enable': 'false',
        }.items(),
    )

    depth_cloud = ComposableNodeContainer(
        name='depth_cloud_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container',
        condition=camera_and(use_depth_cloud),
        composable_node_descriptions=[ComposableNode(
            package='depth_image_proc',
            plugin='depth_image_proc::PointCloudXyzNode',
            name='depth_points',
            # A composable node does not inherit its container's namespace.
            namespace=namespace,
            remappings=[
                ('image_rect', 'camera/depth/image_rect_raw'),
                ('camera_info', 'camera/depth/camera_info'),
                ('points', 'camera/depth/points'),
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        )],
        output='screen',
    )

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

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value=EnvironmentVariable('ROVER_NAMESPACE', default_value='')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        flag_arg('use_camera', 'ROVER_USE_CAMERA', 'false'),
        flag_arg('use_depth_cloud', 'ROVER_CAMERA_DEPTH_CLOUD', 'true'),
        flag_arg('use_fiducials', 'ROVER_CAMERA_FIDUCIALS', 'false'),
        flag_arg('use_detection', 'ROVER_CAMERA_DETECTION', 'false'),
        flag_arg('use_terrain', 'ROVER_USE_TERRAIN', 'false'),
        DeclareLaunchArgument(
            'detection_model',
            default_value=EnvironmentVariable('ROVER_CAMERA_DETECTION_MODEL', default_value=''),
            description='Path of the YOLO .onnx file.'),
        DeclareLaunchArgument(
            'camera_fps',
            default_value=EnvironmentVariable('ROVER_CAMERA_FPS', default_value='15')),
        DeclareLaunchArgument(
            'depth_profile',
            default_value=EnvironmentVariable(
                'ROVER_CAMERA_DEPTH_PROFILE', default_value='424x240'),
            description='Depth resolution WxH.'),
        DeclareLaunchArgument(
            'fiducials_decimate',
            default_value=EnvironmentVariable(
                'ROVER_CAMERA_FIDUCIALS_DECIMATE', default_value='2.0')),
        DeclareLaunchArgument(
            'detection_max_rate',
            default_value=EnvironmentVariable(
                'ROVER_CAMERA_DETECTION_MAX_RATE', default_value='10.0')),
        DeclareLaunchArgument(
            'detection_use_gpu',
            default_value=EnvironmentVariable('ROVER_CAMERA_DETECTION_GPU', default_value='false')),
        realsense,
        depth_cloud,
        apriltag,
        detection,
        terrain,
    ])
