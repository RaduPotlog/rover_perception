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
Depth cloud -> the points a person can be made of, in the robot's base frame.

Intel's RealSense preprocessing (doDBSCAN.cpp, dimension 4) made explicit: subsample, rotate the
optical frame to body axes, crop x and y, remove the ground. Here the rotation is the real
sensor -> base transform from TF (Intel hardcodes x'=z, y'=-x, z'=-y and ignores the mount), and
the ground is removed with a height band in the base frame (Intel's Z_based_ground_removal is a
UINT32, so its RS value 0.05 truncates to 0 and the ground filter never runs).
"""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CloudFilterConfig:
    # Organized clouds keep every row_stride-th row and col_stride-th column; unorganized ones
    # every (row_stride * col_stride)-th point.
    row_stride: int = 4
    col_stride: int = 4
    min_x: float = 0.5
    max_x: float = 4.0
    y_half_width: float = 2.0
    # Height band above base_link's origin: drops the floor and anything over head height.
    z_min: float = 0.15
    z_max: float = 1.9

    def __post_init__(self):
        if self.row_stride < 1 or self.col_stride < 1:
            raise ValueError('strides must be >= 1')
        if not (self.min_x < self.max_x and self.z_min < self.z_max and self.y_half_width > 0):
            raise ValueError(f'empty filter box: {self}')


def subsample(points: np.ndarray, config: CloudFilterConfig) -> np.ndarray:
    """(H, W, 3) organized or (N, 3) points -> (M, 3)."""
    if points.ndim == 3:
        return points[::config.row_stride, ::config.col_stride].reshape(-1, 3)
    return points.reshape(-1, 3)[::config.row_stride * config.col_stride]


def filter_cloud(points: np.ndarray, sensor_to_base: np.ndarray,
                 config: CloudFilterConfig) -> np.ndarray:
    """
    Subsample, drop non-finite points, transform to the base frame and crop.

    points: (H, W, 3) organized or (N, 3), in the sensor frame.
    sensor_to_base: 4x4 homogeneous transform taking sensor-frame points to the base frame.
    Returns (M, 3) float64 points in the base frame.
    """
    pts = subsample(np.asarray(points), config).astype(np.float64, copy=False)
    pts = pts[np.isfinite(pts).all(axis=1)]
    if pts.size == 0:
        return np.empty((0, 3))
    rotation = sensor_to_base[:3, :3]
    translation = sensor_to_base[:3, 3]
    pts = pts @ rotation.T + translation
    keep = (
        (pts[:, 0] >= config.min_x) & (pts[:, 0] <= config.max_x)
        & (np.abs(pts[:, 1]) <= config.y_half_width)
        & (pts[:, 2] >= config.z_min) & (pts[:, 2] <= config.z_max)
    )
    return pts[keep]
