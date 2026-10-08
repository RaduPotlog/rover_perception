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
"""
fmoc person tracking: camera/depth/points -> tracked_person (rover_msgs/TrackedPerson).

  ros2 launch rover_perception_fmoc fmoc.launch.py namespace:=rover

The rover's TF frames carry a <namespace>/ prefix, so global_frame and base_frame get it too.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _frame(namespace: str, frame: str) -> str:
    """Prefix <namespace>/ unless the frame already names one."""
    return f'{namespace}/{frame}' if namespace and '/' not in frame else frame


def _launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    namespace = arg('namespace').strip('/')
    return [Node(
        package='rover_perception_fmoc',
        executable='fmoc_node',
        name='fmoc',
        namespace=namespace,
        output='screen',
        parameters=[
            PathJoinSubstitution([FindPackageShare('rover_perception_fmoc'), 'config', 'fmoc.yaml']),
            {
                'global_frame': _frame(namespace, arg('global_frame')),
                'base_frame': _frame(namespace, arg('base_frame')),
                'use_sim_time': arg('use_sim_time').strip().lower() in ('true', '1', 'yes', 'on'),
            },
        ],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value=EnvironmentVariable('ROVER_NAMESPACE', default_value='')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'global_frame', default_value='odom',
            description='Frame the tracked person is published in. odom always exists, and '
                        "Nav 2's Following server transforms the pose into odom anyway."),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        OpaqueFunction(function=_launch_setup),
    ])
