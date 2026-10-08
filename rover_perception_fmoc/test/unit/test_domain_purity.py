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
"""domain/ and application/ stay free of ROS, MQTT and outer layers (Clean Architecture)."""

import pathlib
import re

import pytest

PKG = pathlib.Path(__file__).resolve().parents[2] / 'rover_perception_fmoc'
FORBIDDEN = re.compile(
    r'^\s*(import|from)\s+(rclpy|paho|launch|tf2_ros|[a-z0-9_]*_msgs|'
    r'\.\.?(infrastructure|presentation)|rover_perception_fmoc\.(infrastructure|presentation))\b',
    re.MULTILINE)


@pytest.mark.parametrize('layer', ['domain', 'application'])
def test_layer_has_no_ros_or_outer_imports(layer):
    offenders = []
    for path in sorted((PKG / layer).rglob('*.py')):
        for match in FORBIDDEN.finditer(path.read_text()):
            offenders.append(f'{path.relative_to(PKG)}: {match.group(0).strip()}')
    assert not offenders, '\n'.join(offenders)
