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

"""Use case: turn a gravity-aligned point set into a smoothed terrain estimate."""

from dataclasses import dataclass
from typing import Optional

import numpy as np

from rover_perception_terrain.domain.ground_plane import fit_ground_plane, select_ground_candidates


@dataclass(frozen=True)
class TerrainEstimate:
    slope_deg: float
    incline_forward_deg: float
    incline_lateral_deg: float
    confidence: float


class EstimateTerrain:
    """Fit the ground plane and low-pass the angles so a single bad frame cannot jerk them."""

    def __init__(
        self,
        min_x: float = 0.5,
        max_x: float = 4.0,
        max_abs_y: float = 1.5,
        max_abs_z: float = 0.6,
        max_points: int = 2000,
        distance_threshold: float = 0.05,
        min_inlier_ratio: float = 0.3,
        smoothing: float = 0.5,
        seed: Optional[int] = None,
    ):
        self._roi = {'min_x': min_x, 'max_x': max_x, 'max_abs_y': max_abs_y,
                     'max_abs_z': max_abs_z, 'max_points': max_points}
        self._distance_threshold = distance_threshold
        self._min_inlier_ratio = min_inlier_ratio
        self._alpha = min(1.0, max(0.0, smoothing))  # 1.0 = no smoothing
        self._rng = np.random.default_rng(seed)
        self._last: Optional[TerrainEstimate] = None

    def execute(self, points: np.ndarray) -> Optional[TerrainEstimate]:
        """Return the smoothed estimate, or None when no ground plane was found this frame."""
        candidates = select_ground_candidates(points, rng=self._rng, **self._roi)
        plane = fit_ground_plane(
            candidates,
            distance_threshold=self._distance_threshold,
            min_inlier_ratio=self._min_inlier_ratio,
            rng=self._rng,
        )
        if plane is None:
            return None
        raw = TerrainEstimate(
            plane.slope_deg, plane.incline_forward_deg, plane.incline_lateral_deg,
            plane.inlier_ratio)
        if self._last is None:
            self._last = raw
        else:
            a, p = self._alpha, self._last
            self._last = TerrainEstimate(
                a * raw.slope_deg + (1 - a) * p.slope_deg,
                a * raw.incline_forward_deg + (1 - a) * p.incline_forward_deg,
                a * raw.incline_lateral_deg + (1 - a) * p.incline_lateral_deg,
                raw.confidence)
        return self._last
