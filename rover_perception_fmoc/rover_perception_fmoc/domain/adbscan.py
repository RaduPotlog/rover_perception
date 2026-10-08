# Copyright 2026 Mechatronics Academy
# Copyright (C) 2025 Intel Corporation
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
Adaptive DBSCAN (ADBSCAN): a Python port of Intel's follow-me clustering.

Ported from open-edge-platform/edge-ai-suites, robotics-ai-suite/components/adbscan/
Follow_me_RS_2D/src/adbscan_ros2/src: adaptive_parameters() in doDBSCAN.cpp, the KD-tree
dbscan_adaptiveK() in dbscan_adaptiveK.cpp (the variant compiled without USE_LINKED_LISTS), and
calc_centroid() / get_obstacle() in Util.cpp. See NOTICE.

Each point gets its own minimum-points K and radius eps from its forward distance x:

    K_i   = max(1, coeff_2 * x_i^2 + coeff_1 * x_i + base),   Kf_i = floor(K_i)
    eps_i = ((dx * dy * dz) * Kf_i * GAMMA / (m * sqrt(PI^3)))^(1/3) * scale_factor

where dx, dy, dz are the extents of the whole cloud and m its point count, so eps follows the
cloud's density. If that is not positive (a flat cloud), eps falls back to
0.02 * (max(0, max x) - min(0, min x)), as in the C++.

One addition, off by default: a floor min_eps + min_eps_per_m * x on eps. With little else in view
(a person in an open space) the cloud's extents are small, eps shrinks below the spacing of the
subsampled depth points at a few metres, and the person turns into noise; the floor keeps eps
above that spacing (about stride * FOV / width rad per metre of range).

The clustering keeps Intel's semantics, which differ from textbook DBSCAN: neighbourhoods use each
point's own eps (so they are not symmetric) and include the point itself; a seed is core when it
has at least Kf + 1 neighbours; during expansion every reached point with more than one neighbour
adds its neighbours, core or not; the active set is processed lowest index first. Labels are
1, 2, ... per cluster and -1 for noise.
"""

from dataclasses import dataclass
import heapq
import math
from typing import List, Sequence, Tuple

import numpy as np

try:  # scipy's KD-tree when available (the container installs python3-scipy).
    from scipy.spatial import cKDTree
except ImportError:  # pragma: no cover - exercised where scipy is missing
    cKDTree = None

# dbscan_adaptiveK.h
ADBSCAN_PI = 3.1416
ADBSCAN_GAMMA = 1.3293  # gamma(0.5 * n + 1) for n = 3, as Intel hardcodes it
DIMENSIONS = 3


@dataclass(frozen=True)
class AdbscanConfig:
    # Intel's RealSense coefficients (adbscan_sub_RS.yaml).
    base: float = 5.29545454
    coeff_1: float = -0.164835164
    coeff_2: float = -0.017982017
    scale_factor: float = 0.9
    # Not in Intel's ADBSCAN (0 = as Intel): eps >= min_eps + min_eps_per_m * x.
    min_eps: float = 0.0
    min_eps_per_m: float = 0.0


@dataclass(frozen=True)
class Cluster:
    label: int
    centroid: Tuple[float, float, float]   # mean of the points (Intel's calc_centroid)
    box_min: Tuple[float, float, float]
    box_max: Tuple[float, float, float]
    count: int

    @property
    def center(self) -> Tuple[float, float, float]:
        """Bounding-box centre (Intel's get_obstacle)."""
        return tuple((lo + hi) / 2.0 for lo, hi in zip(self.box_min, self.box_max))

    @property
    def size(self) -> Tuple[float, float, float]:
        return tuple(hi - lo for lo, hi in zip(self.box_min, self.box_max))

    @property
    def width(self) -> float:
        """Horizontal extent: the larger of the x and y box sides."""
        sx, sy, _ = self.size
        return max(sx, sy)

    @property
    def height_span(self) -> float:
        return self.size[2]


def adaptive_parameters(points: np.ndarray, config: AdbscanConfig,
                        x_offset: float = 0.0) -> Tuple[np.ndarray, np.ndarray]:
    """
    Per-point (K_floor, eps) for an (m, 3) cloud, as doDBSCAN.cpp's adaptive_parameters().

    x_offset: the sensor's x in the points' frame. Intel's x is the distance from the camera;
    with points in the base frame, K is computed from x - x_offset.
    """
    m = len(points)
    if m == 0:
        return np.empty(0), np.empty(0)
    # Intel keeps base/coeff_1/coeff_2 as float32 (Adaptive_params_t).
    base = float(np.float32(config.base))
    coeff_1 = float(np.float32(config.coeff_1))
    coeff_2 = float(np.float32(config.coeff_2))

    extent = points.max(axis=0) - points.min(axis=0)
    prod = float(np.prod(extent))
    denominator = m * math.sqrt(ADBSCAN_PI ** DIMENSIONS)

    x = points[:, 0] - x_offset
    k = coeff_2 * x * x + coeff_1 * x + base
    k = np.where(k < 1.0, 1.0, k)
    k_floor = np.floor(k)
    eps = np.power(prod * k_floor * ADBSCAN_GAMMA / denominator, 1.0 / DIMENSIONS)
    eps = eps * config.scale_factor
    # max_x / min_x start at 0 in the C++, so they are max(0, max x) and min(0, min x).
    fallback = 0.02 * (max(0.0, float(x.max())) - min(0.0, float(x.min())))
    eps = np.where(eps <= 0.0, fallback, eps)
    if config.min_eps > 0.0 or config.min_eps_per_m > 0.0:
        eps = np.maximum(eps, config.min_eps + config.min_eps_per_m * np.maximum(x, 0.0))
    return k_floor, eps


def radius_neighbors(points: np.ndarray, eps: np.ndarray) -> List[np.ndarray]:
    """For each point i, the indices within eps[i] of it (itself included), sorted."""
    n = len(points)
    if n == 0:
        return []
    if cKDTree is not None:
        # All pairs within the largest eps in one call, then each point's own eps.
        tree = cKDTree(points)
        pairs = tree.sparse_distance_matrix(tree, float(eps.max()), output_type='ndarray')
        i, j, d = pairs['i'], pairs['j'], pairs['v']
        keep = (d <= eps[i]) & (i != j)
        i, j = i[keep], j[keep]
        # Every point is its own neighbour (PCL's radiusSearch returns the query point too).
        i = np.concatenate([i, np.arange(n)])
        j = np.concatenate([j, np.arange(n)])
        order = np.lexsort((j, i))
        i, j = i[order], j[order]
        return np.split(j.astype(np.int64), np.searchsorted(i, np.arange(1, n)))
    # numpy fallback: points sorted on x, processed in chunks against the band of points whose
    # x is within the largest eps of the chunk, then the exact per-point distance test.
    order = np.argsort(points[:, 0], kind='stable')
    xs = points[order, 0]
    reach = float(eps.max())
    neighbors: List[np.ndarray] = [None] * n
    chunk = 256
    for start in range(0, n, chunk):
        idx = order[start:start + chunk]
        lo = np.searchsorted(xs, xs[start] - reach, side='left')
        hi = np.searchsorted(xs, xs[min(start + chunk, n) - 1] + reach, side='right')
        window = order[lo:hi]
        d2 = np.sum((points[idx, None, :] - points[None, window, :]) ** 2, axis=2)
        inside = d2 <= (eps[idx] ** 2)[:, None]
        for row, i in enumerate(idx):
            neighbors[i] = np.sort(window[inside[row]])
    return neighbors


def cluster_labels(neighbors: Sequence[np.ndarray], k_floor: np.ndarray) -> np.ndarray:
    """dbscan_adaptiveK() (KD-tree variant): labels 1.. per cluster, -1 for noise."""
    n = len(neighbors)
    labels = np.full(n, -1, dtype=np.int32)
    touched = np.zeros(n, dtype=bool)
    active = np.zeros(n, dtype=bool)
    cluster = 1
    for i in range(n):
        if touched[i]:
            continue
        seed = neighbors[i]
        length = len(seed)
        if length == 1:
            touched[i] = True  # only itself: noise
            continue
        if length < k_floor[i] + 1:
            continue  # not core; may still be reached from a core point later
        labels[seed] = cluster
        active[seed] = True
        heap = [int(j) for j in seed]
        heapq.heapify(heap)
        while heap:
            ind = heapq.heappop(heap)
            active[ind] = False
            touched[ind] = True
            reached = neighbors[ind]
            if len(reached) > 1:
                labels[reached] = cluster
                new = reached[~touched[reached]]
                touched[new] = True
                for j in new[~active[new]]:
                    active[j] = True
                    heapq.heappush(heap, int(j))
        cluster += 1
    return labels


def adbscan(points: np.ndarray, config: AdbscanConfig, x_offset: float = 0.0) -> np.ndarray:
    """Cluster an (m, 3) cloud; returns per-point labels (1.. clusters, -1 noise)."""
    if len(points) == 0:
        return np.empty(0, dtype=np.int32)
    k_floor, eps = adaptive_parameters(points, config, x_offset)
    return cluster_labels(radius_neighbors(points, eps), k_floor)


def clusters_from_labels(points: np.ndarray, labels: np.ndarray) -> List[Cluster]:
    clusters = []
    for label in np.unique(labels):
        if label < 1:
            continue
        member = points[labels == label]
        clusters.append(Cluster(
            label=int(label),
            centroid=tuple(float(v) for v in member.mean(axis=0)),
            box_min=tuple(float(v) for v in member.min(axis=0)),
            box_max=tuple(float(v) for v in member.max(axis=0)),
            count=len(member)))
    return clusters
