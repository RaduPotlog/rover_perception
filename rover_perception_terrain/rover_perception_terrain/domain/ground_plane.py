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


"""Ground-plane fit and incline maths. Pure numpy: no ROS imports (domain layer)."""

from dataclasses import dataclass
import math
from typing import Optional, Tuple

import numpy as np


@dataclass(frozen=True)
class GroundPlane:
    """Plane n . p + d = 0 with a unit normal pointing up (nz >= 0)."""

    normal: Tuple[float, float, float]
    d: float
    inlier_ratio: float

    @property
    def slope_deg(self) -> float:
        """Angle between the plane and the horizontal, 0 on flat ground."""
        return math.degrees(math.acos(max(-1.0, min(1.0, self.normal[2]))))

    @property
    def incline_forward_deg(self) -> float:
        """Rise along +x: positive when the ground climbs ahead of the rover."""
        return math.degrees(math.atan2(-self.normal[0], self.normal[2]))

    @property
    def incline_lateral_deg(self) -> float:
        """Rise along +y: positive when the ground climbs to the rover's left."""
        return math.degrees(math.atan2(-self.normal[1], self.normal[2]))


def fit_ground_plane(
    points: np.ndarray,
    iterations: int = 60,
    distance_threshold: float = 0.05,
    min_inlier_ratio: float = 0.3,
    rng: Optional[np.random.Generator] = None,
) -> Optional[GroundPlane]:
    """RANSAC plane fit on an (N, 3) array, refined by least squares on the inliers.

    The points must be in a gravity-aligned frame (z up) for the incline angles to be
    meaningful. Returns None when too few points or no plane covers min_inlier_ratio.
    """
    n = len(points)
    if n < 10:
        return None
    rng = rng or np.random.default_rng()
    best_mask = None
    best_count = 0
    for _ in range(iterations):
        sample = points[rng.choice(n, 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal /= norm
        mask = np.abs((points - sample[0]) @ normal) < distance_threshold
        count = int(mask.sum())
        if count > best_count:
            best_count, best_mask = count, mask
    if best_mask is None or best_count / n < min_inlier_ratio:
        return None

    inliers = points[best_mask]
    centroid = inliers.mean(axis=0)
    # Smallest singular vector of the centred inliers is the plane normal.
    normal = np.linalg.svd(inliers - centroid, full_matrices=False)[2][-1]
    if normal[2] < 0:
        normal = -normal
    d = -float(normal @ centroid)
    return GroundPlane(
        normal=(float(normal[0]), float(normal[1]), float(normal[2])),
        d=d,
        inlier_ratio=best_count / n,
    )


def select_ground_candidates(
    points: np.ndarray,
    min_x: float,
    max_x: float,
    max_abs_y: float,
    max_abs_z: float,
    max_points: int,
    rng: Optional[np.random.Generator] = None,
) -> np.ndarray:
    """Keep the region ahead of the rover near ground height, randomly capped to max_points."""
    keep = (
        (points[:, 0] >= min_x)
        & (points[:, 0] <= max_x)
        & (np.abs(points[:, 1]) <= max_abs_y)
        & (np.abs(points[:, 2]) <= max_abs_z)
    )
    out = points[keep]
    if len(out) > max_points:
        rng = rng or np.random.default_rng()
        out = out[rng.choice(len(out), max_points, replace=False)]
    return out
