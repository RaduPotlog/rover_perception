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
import math

import numpy as np
import pytest

from rover_perception_fmoc.domain.cloud_filter import CloudFilterConfig, filter_cloud, subsample


def test_subsample_organized_and_flat():
    cfg = CloudFilterConfig(row_stride=2, col_stride=3)
    organized = np.arange(4 * 6 * 3, dtype=float).reshape(4, 6, 3)
    assert subsample(organized, cfg).shape == (4, 3)
    assert subsample(organized.reshape(-1, 3), cfg).shape == (4, 3)


def test_crop_box_height_band_and_nans():
    cfg = CloudFilterConfig(row_stride=1, col_stride=1, min_x=0.3, max_x=4.0, y_half_width=2.0,
                            z_min=0.15, z_max=1.9)
    pts = np.array([
        [1.0, 0.0, 1.0],     # kept
        [1.0, 0.0, 0.05],    # floor
        [1.0, 0.0, 2.5],     # above head
        [0.1, 0.0, 1.0],     # too close
        [5.0, 0.0, 1.0],     # too far
        [1.0, 2.5, 1.0],     # too far sideways
        [np.nan, 0.0, 1.0],  # no return
    ])
    out = filter_cloud(pts, np.eye(4), cfg)
    np.testing.assert_allclose(out, [[1.0, 0.0, 1.0]])


def test_optical_frame_is_rotated_into_the_base_frame():
    # REP-103 optical frame (z forward, x right, y down) mounted 0.25 m ahead, 0.5 m up.
    to_base = np.eye(4)
    to_base[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
    to_base[:3, 3] = [0.25, 0.0, 0.5]
    cfg = CloudFilterConfig(row_stride=1, col_stride=1)
    # 2 m ahead, 0.5 m to the right, 0.3 m below the camera.
    out = filter_cloud(np.array([[0.5, 0.3, 2.0]]), to_base, cfg)
    np.testing.assert_allclose(out, [[2.25, -0.5, 0.2]])


def test_empty_box_is_refused():
    with pytest.raises(ValueError):
        CloudFilterConfig(min_x=2.0, max_x=1.0)
    assert math.isfinite(CloudFilterConfig().max_x)
