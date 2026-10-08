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
"""A tiny ray caster standing in for the D435i: floor, walls and person-sized cylinders."""

import math

import numpy as np

CAMERA_HEIGHT = 0.5
CAMERA_X = 0.25


def render(people=(), wall_x=None, side_wall_y=None, width=424, height=240,
           hfov_deg=87.0, vfov_deg=58.0, max_range=6.0, noise=0.0, seed=0):
    """
    Organized (height, width, 3) cloud in a camera frame with x forward, y left, z up, placed at
    (CAMERA_X, 0, CAMERA_HEIGHT) in base_link with no rotation. Misses are NaN.

    people: (x, y) in base_link of 0.22 m radius, 1.7 m tall cylinders.
    """
    h = math.tan(math.radians(hfov_deg) / 2.0)
    v = math.tan(math.radians(vfov_deg) / 2.0)
    us = np.linspace(h, -h, width)          # left .. right
    vs = np.linspace(v, -v, height)         # up .. down
    dy, dz = np.meshgrid(us, vs)
    dirs = np.stack([np.ones_like(dy), dy, dz], axis=-1)
    dirs /= np.linalg.norm(dirs, axis=-1, keepdims=True)
    origin = np.array([CAMERA_X, 0.0, CAMERA_HEIGHT])
    best = np.full(dy.shape, np.inf)

    def hit(t):
        nonlocal best
        t = np.where((t > 0.05) & (t < best), t, np.inf)
        best = np.minimum(best, t)

    with np.errstate(divide='ignore', invalid='ignore'):
        hit(-origin[2] / dirs[..., 2])                              # floor z = 0
        if wall_x is not None:
            hit((wall_x - origin[0]) / dirs[..., 0])
        if side_wall_y is not None:
            hit((side_wall_y - origin[1]) / dirs[..., 1])
        for px, py in people:
            # Vertical cylinder radius r: |(o + t d)_xy - c|^2 = r^2.
            r = 0.22
            ox, oy = origin[0] - px, origin[1] - py
            a = dirs[..., 0] ** 2 + dirs[..., 1] ** 2
            b = 2.0 * (ox * dirs[..., 0] + oy * dirs[..., 1])
            c = ox * ox + oy * oy - r * r
            disc = b * b - 4.0 * a * c
            t = (-b - np.sqrt(np.where(disc >= 0.0, disc, np.nan))) / (2.0 * a)
            z = origin[2] + t * dirs[..., 2]
            hit(np.where((disc >= 0.0) & (z >= 0.0) & (z <= 1.7), t, np.inf))
    best = np.where(best <= max_range, best, np.nan)
    points = dirs * best[..., None]
    if noise:
        points = points + np.random.default_rng(seed).normal(0.0, noise, points.shape)
    return points


def camera_to_base():
    t = np.eye(4)
    t[:3, 3] = [CAMERA_X, 0.0, CAMERA_HEIGHT]
    return t
