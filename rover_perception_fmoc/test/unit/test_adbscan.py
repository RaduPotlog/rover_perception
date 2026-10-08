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

from rover_perception_fmoc.domain import adbscan as adbscan_module
from rover_perception_fmoc.domain.adbscan import (
    adaptive_parameters, adbscan, AdbscanConfig, cluster_labels, clusters_from_labels,
    radius_neighbors)
from rover_perception_fmoc.domain.cloud_filter import CloudFilterConfig, filter_cloud
from synthetic_scene import camera_to_base, CAMERA_X, render

CFG = AdbscanConfig()


def test_adaptive_parameters_match_intel_formula():
    points = np.array([[0.0, 0.0, 0.0], [2.0, 1.0, 0.5], [1.0, 0.5, 1.0], [2.0, 0.0, 0.0]])
    k_floor, eps = adaptive_parameters(points, CFG)
    # x = 2: K = coeff_2 * 4 + coeff_1 * 2 + base = 4.8939 -> 4
    assert k_floor[1] == 4.0
    # x = 0: K = base = 5.295 -> 5
    assert k_floor[0] == 5.0
    prod, m = 2.0 * 1.0 * 1.0, 4
    expected = (prod * 4 * 1.3293 / (m * math.sqrt(3.1416 ** 3))) ** (1.0 / 3.0) * 0.9
    assert eps[1] == pytest.approx(expected, rel=1e-6)
    assert eps[1] == pytest.approx(0.7035, abs=1e-3)


def test_k_is_never_below_one_and_x_offset_shifts_it():
    far = np.array([[30.0, 0.0, 0.0], [31.0, 1.0, 1.0]])
    k_floor, _ = adaptive_parameters(far, CFG)
    assert (k_floor == 1.0).all()
    near = np.array([[2.25, 0.0, 0.0], [3.0, 1.0, 1.0]])
    k_shifted, _ = adaptive_parameters(near, CFG, x_offset=0.25)
    k_plain, _ = adaptive_parameters(near - [0.25, 0.0, 0.0], CFG)
    np.testing.assert_array_equal(k_shifted, k_plain)


def test_flat_cloud_falls_back_to_two_percent_of_x_range():
    flat = np.array([[1.0, 0.0, 0.0], [3.0, 1.0, 0.0], [2.0, 0.5, 0.0]])
    _, eps = adaptive_parameters(flat, CFG)
    # max(0, max x) - min(0, min x) = 3 - 0
    np.testing.assert_allclose(eps, 0.06)


def test_neighbourhoods_use_each_points_own_radius_and_include_itself():
    points = np.array([[0.0, 0.0, 0.0], [0.5, 0.0, 0.0]])
    neighbors = radius_neighbors(points, np.array([1.0, 0.1]))
    assert neighbors[0].tolist() == [0, 1]
    assert neighbors[1].tolist() == [1]


def test_cluster_labels_core_border_and_noise():
    # 0..3 a tight group (core with K=2), 4 reachable only from the group, 5 alone.
    neighbors = [np.array(n) for n in ([0, 1, 2, 3], [0, 1, 2, 3, 4], [0, 1, 2], [0, 1, 3],
                                       [1, 4], [5])]
    labels = cluster_labels(neighbors, np.full(6, 2.0))
    assert labels.tolist() == [1, 1, 1, 1, 1, -1]


def _scene_clusters(**scene):
    base = filter_cloud(render(**scene), camera_to_base(), CloudFilterConfig())
    labels = adbscan(base, CFG, x_offset=CAMERA_X)
    return clusters_from_labels(base, labels)


def _near(clusters, x, y, tol=0.4):
    return [c for c in clusters if math.hypot(c.centroid[0] - x, c.centroid[1] - y) < tol]


def test_a_person_is_one_person_sized_cluster():
    clusters = _scene_clusters(people=[(1.6, 0.0)])
    person = _near(clusters, 1.45, 0.0)
    assert len(person) == 1
    assert 0.15 < person[0].width < 1.0
    assert person[0].height_span > 0.6


def test_a_person_next_to_a_wall_is_separate_from_it():
    clusters = _scene_clusters(people=[(2.0, -0.9)], side_wall_y=-1.5)
    person = _near(clusters, 1.85, -0.9, tol=0.3)
    assert len(person) == 1
    assert person[0].width < 1.0
    walls = [c for c in clusters if c.centroid[1] < -1.4]
    assert walls and all(c.label != person[0].label for c in walls)


def test_two_people_are_two_clusters():
    clusters = _scene_clusters(people=[(1.6, 0.0), (2.5, 1.2)])
    assert len(_near(clusters, 1.45, 0.0)) == 1
    assert len(_near(clusters, 2.35, 1.15)) == 1


def test_scattered_outliers_are_noise():
    # eps adapts to the cloud's own density, so "sparse" is relative: a few stray returns in a
    # scene with a person in it are noise.
    base = filter_cloud(render(people=[(1.6, 0.0)], wall_x=3.5), camera_to_base(),
                        CloudFilterConfig())
    rng = np.random.default_rng(3)
    strays = np.column_stack([rng.uniform(0.5, 3.0, 8), rng.uniform(-1.8, -1.0, 8),
                              rng.uniform(0.2, 1.8, 8)])
    cloud = np.vstack([base, strays])
    labels = adbscan(cloud, CFG, x_offset=CAMERA_X)
    assert (labels[-len(strays):] == -1).all()


def test_empty_cloud():
    assert adbscan(np.empty((0, 3)), CFG).size == 0


def test_numpy_fallback_matches_kdtree():
    if adbscan_module.cKDTree is None:
        pytest.skip('scipy not installed; only the numpy path exists')
    base = filter_cloud(render(people=[(1.6, 0.0)], wall_x=3.5, noise=0.01), camera_to_base(),
                        CloudFilterConfig())
    _, eps = adaptive_parameters(base, CFG, CAMERA_X)
    with_tree = radius_neighbors(base, eps)
    saved = adbscan_module.cKDTree
    adbscan_module.cKDTree = None
    try:
        without = radius_neighbors(base, eps)
    finally:
        adbscan_module.cKDTree = saved
    assert all(np.array_equal(a, b) for a, b in zip(with_tree, without))


def test_eps_floor_keeps_a_lone_distant_person_a_cluster():
    # Only the person in view: Intel's eps follows the tiny cloud and drops below the point
    # spacing at 4 m; the floor keeps the person together.
    base = filter_cloud(render(people=[(4.2, 0.0)]), camera_to_base(),
                        CloudFilterConfig(row_stride=5, col_stride=5, max_x=5.0))
    intel = clusters_from_labels(base, adbscan(base, CFG, x_offset=CAMERA_X))
    floored = AdbscanConfig(min_eps=0.05, min_eps_per_m=0.04)
    with_floor = clusters_from_labels(base, adbscan(base, floored, x_offset=CAMERA_X))
    assert not _near(intel, 4.0, 0.0)
    person = _near(with_floor, 4.0, 0.0)
    assert len(person) == 1 and person[0].height_span > 0.6


def test_eps_floor_is_off_by_default():
    points = np.array([[0.0, 0.0, 0.0], [2.0, 1.0, 0.5], [1.0, 0.5, 1.0], [2.0, 0.0, 0.0]])
    _, plain = adaptive_parameters(points, CFG)
    _, floored = adaptive_parameters(points, AdbscanConfig(min_eps=1.0))
    np.testing.assert_allclose(floored, np.maximum(plain, 1.0))
