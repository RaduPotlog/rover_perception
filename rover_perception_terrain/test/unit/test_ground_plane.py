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

from rover_perception_terrain.application.estimate_terrain import EstimateTerrain
from rover_perception_terrain.domain.ground_plane import fit_ground_plane


def _plane(pitch_deg, roll_deg=0.0, n=3000, noise=0.005, seed=0, outliers=0):
    """Ground rising along +x at pitch_deg (and +y at roll_deg), plus optional clutter."""
    rng = np.random.default_rng(seed)
    x = rng.uniform(0.5, 4.0, n)
    y = rng.uniform(-1.5, 1.5, n)
    z = x * math.tan(math.radians(pitch_deg)) + y * math.tan(math.radians(roll_deg))
    pts = np.stack([x, y, z + rng.normal(0, noise, n)], axis=1)
    if outliers:
        clutter = np.stack([rng.uniform(0.5, 4, outliers), rng.uniform(-1.5, 1.5, outliers),
                            rng.uniform(0.2, 0.55, outliers)], axis=1)
        pts = np.vstack([pts, clutter])
    return pts


@pytest.mark.parametrize('pitch', [0.0, 5.0, -5.0, 12.0])
def test_forward_incline_recovered(pitch):
    plane = fit_ground_plane(_plane(pitch), rng=np.random.default_rng(1))
    assert plane is not None
    assert plane.incline_forward_deg == pytest.approx(pitch, abs=0.5)
    assert plane.slope_deg == pytest.approx(abs(pitch), abs=0.5)
    assert plane.incline_lateral_deg == pytest.approx(0.0, abs=0.5)


def test_lateral_incline_recovered():
    plane = fit_ground_plane(_plane(0.0, roll_deg=4.0), rng=np.random.default_rng(1))
    assert plane.incline_lateral_deg == pytest.approx(4.0, abs=0.5)


def test_robust_to_clutter():
    plane = fit_ground_plane(_plane(5.0, outliers=600), rng=np.random.default_rng(2))
    assert plane is not None
    assert plane.incline_forward_deg == pytest.approx(5.0, abs=0.7)


def test_too_few_points_returns_none():
    assert fit_ground_plane(np.zeros((5, 3))) is None


def test_no_dominant_plane_returns_none():
    pts = np.random.default_rng(3).uniform(-3, 3, (500, 3))
    assert fit_ground_plane(pts, min_inlier_ratio=0.6, rng=np.random.default_rng(3)) is None


def test_use_case_smooths_and_filters_roi():
    uc = EstimateTerrain(smoothing=0.5, seed=0)
    first = uc.execute(_plane(0.0))
    second = uc.execute(_plane(10.0))
    assert first.incline_forward_deg == pytest.approx(0.0, abs=0.5)
    # Halfway between 0 and 10 with alpha 0.5.
    assert second.incline_forward_deg == pytest.approx(5.0, abs=0.7)


def test_use_case_ignores_points_outside_roi():
    behind = _plane(0.0)
    behind[:, 0] *= -1  # everything behind the rover
    assert EstimateTerrain(seed=0).execute(behind) is None
