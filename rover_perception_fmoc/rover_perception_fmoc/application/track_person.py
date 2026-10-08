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

"""One depth frame -> the tracked person: filter, ADBSCAN, associate."""

from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from ..domain.adbscan import adbscan, AdbscanConfig, Cluster, clusters_from_labels
from ..domain.cloud_filter import CloudFilterConfig, filter_cloud
from ..domain.target_tracker import (
    Candidate, RobotMotion, RobotPose, TargetTracker, Track, TrackerConfig)


@dataclass(frozen=True)
class FrameResult:
    track: Track
    clusters: List[Cluster]   # base frame, for visualization
    points_used: int


class TrackPerson:

    def __init__(self, filter_config: CloudFilterConfig, adbscan_config: AdbscanConfig,
                 tracker_config: TrackerConfig, max_points: int = 6000, seed: int = 0):
        self._filter = filter_config
        self._adbscan = adbscan_config
        self._tracker = TargetTracker(tracker_config)
        self._max_points = max_points
        self._rng = np.random.default_rng(seed)

    def reset(self) -> None:
        self._tracker.reset()

    def process(self, points: np.ndarray, sensor_to_base: np.ndarray, robot: RobotPose,
                motion: Optional[RobotMotion], stamp: float) -> FrameResult:
        """
        points: (H, W, 3) organized or (N, 3) cloud in the sensor frame.
        sensor_to_base: 4x4 transform sensor -> base frame.
        robot: the base frame's pose in the global frame at the cloud's time.
        motion: the rover's velocity, None when unknown (no recent odometry).
        """
        base_points = filter_cloud(points, sensor_to_base, self._filter)
        if len(base_points) > self._max_points:
            keep = self._rng.choice(len(base_points), self._max_points, replace=False)
            base_points = base_points[np.sort(keep)]
        # Intel computes K from the distance to the camera.
        sensor_x = float(sensor_to_base[0, 3])
        labels = adbscan(base_points, self._adbscan, x_offset=sensor_x)
        clusters = clusters_from_labels(base_points, labels)
        candidates = []
        for c in clusters:
            gx, gy = robot.to_global(c.centroid[0], c.centroid[1])
            candidates.append(Candidate(gx, gy, c.width, c.height_span))
        track = self._tracker.update(candidates, robot, motion, stamp)
        return FrameResult(track, clusters, len(base_points))
