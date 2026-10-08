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
import numpy as np

from rover_perception_fmoc.application.track_person import TrackPerson
from rover_perception_fmoc.domain.adbscan import AdbscanConfig
from rover_perception_fmoc.domain.cloud_filter import CloudFilterConfig
from rover_perception_fmoc.domain.target_tracker import RobotMotion, RobotPose, TrackerConfig, TrackState
from synthetic_scene import camera_to_base, render


def test_follows_a_person_walking_away_past_a_bystander():
    use_case = TrackPerson(CloudFilterConfig(), AdbscanConfig(), TrackerConfig())
    robot = RobotPose(0.0, 0.0, 0.0)
    result = use_case.process(render(people=[(1.6, 0.0), (2.0, -1.2)], wall_x=4.5),
                              camera_to_base(), robot, RobotMotion(0.0, 0.0), 0.0)
    assert result.track.state == TrackState.TRACKING
    assert abs(result.track.y) < 0.1
    xs = []
    for i in range(1, 12):
        px = 1.6 + 0.15 * i
        result = use_case.process(render(people=[(px, 0.0), (2.0, -1.2)], wall_x=4.5),
                                  camera_to_base(), robot, RobotMotion(0.5, 0.0), 0.2 * i)
        assert result.track.state == TrackState.TRACKING
        xs.append(result.track.x)
        assert abs(result.track.y) < 0.15  # never jumps to the bystander
    assert np.all(np.diff(xs) > 0)


def test_max_points_caps_the_cloud():
    use_case = TrackPerson(CloudFilterConfig(row_stride=1, col_stride=1), AdbscanConfig(),
                           TrackerConfig(), max_points=500)
    result = use_case.process(render(people=[(1.6, 0.0)]), camera_to_base(),
                              RobotPose(0.0, 0.0, 0.0), RobotMotion(0.0, 0.0), 0.0)
    assert result.points_used == 500
