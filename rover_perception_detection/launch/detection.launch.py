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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value=EnvironmentVariable('ROVER_SYSTEM_NAMESPACE', default_value='')),
        DeclareLaunchArgument('image_topic', default_value='camera/color/image_raw'),
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('use_gpu', default_value='false'),
        DeclareLaunchArgument('max_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='rover_perception_detection',
            executable='detection_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                PathJoinSubstitution(
                    [FindPackageShare('rover_perception_detection'), 'config', 'detection.yaml']),
                {'model_path': LaunchConfiguration('model_path'),
                 'use_gpu': LaunchConfiguration('use_gpu'),
                 'max_rate_hz': LaunchConfiguration('max_rate_hz'),
                 'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[('image_raw', LaunchConfiguration('image_topic'))],
            output='screen',
        ),
    ])
