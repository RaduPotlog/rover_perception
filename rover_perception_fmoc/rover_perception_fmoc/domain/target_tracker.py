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
Which cluster is the person: acquire once in front of the rover, then follow frame to frame.

Intel's rule (doDBSCAN.cpp): start from a target location in front of the robot, take the
cluster centroid nearest the previous target within tracking_radius, hold the previous location
for max_frame_blocked frames when none is found, then give up. Here the target lives in the
global frame, so the rover's own motion does not move it, and the search is centred on a
constant-velocity prediction. Acquisition also asks for a person-sized cluster and a rover at
standstill, so a passing box or a wall is not picked up while driving.

Once lost, the person is looked for again near where they were last seen, for reacquire_time,
with the rover allowed to move: following lags behind a walking person, and the usual way to lose
them is that they get out of range or out of view while the rover is still driving towards them.
"""

from dataclasses import dataclass
from enum import IntEnum
import math
from typing import Iterable, Optional, Tuple


class TrackState(IntEnum):
    # Values match rover_msgs/TrackedPerson.
    NONE = 0
    TRACKING = 1
    COASTING = 2
    LOST = 3


@dataclass(frozen=True)
class TrackerConfig:
    # Where the person stands to be picked up, in the rover's base frame.
    acquire_x: float = 1.5
    acquire_y: float = 0.0
    acquire_radius: float = 0.6
    # Person-sized: horizontal extent and visible height of the cluster.
    min_width: float = 0.15
    max_width: float = 1.0
    min_height_span: float = 0.6
    # Frame-to-frame association radius around the prediction (Intel recommends 1.0; 0.2 loses
    # the target in turns).
    tracking_radius: float = 1.0
    max_frames_blocked: int = 5
    # Low-pass on the velocity estimate (0 = raw, 1 = frozen).
    velocity_smoothing: float = 0.6
    # A prediction never extrapolates further than this many seconds.
    max_prediction: float = 1.0
    # The rover counts as standing still (acquisition allowed) below these (m/s, rad/s).
    still_linear: float = 0.05
    still_angular: float = 0.1
    # After a loss: a person-sized cluster this close to where the person was last seen is
    # taken to be them again, for this long.
    reacquire_radius: float = 1.5
    reacquire_time: float = 10.0

    def __post_init__(self):
        if self.acquire_radius <= 0.0 or self.tracking_radius <= 0.0:
            raise ValueError('radii must be positive')
        if self.max_frames_blocked < 0:
            raise ValueError('max_frames_blocked must be >= 0')
        if not 0.0 <= self.velocity_smoothing < 1.0:
            raise ValueError('velocity_smoothing must be in [0, 1)')
        if self.reacquire_radius < 0.0 or self.reacquire_time < 0.0:
            raise ValueError('reacquire_radius and reacquire_time must not be negative')


@dataclass(frozen=True)
class Candidate:
    """A cluster as the tracker sees it: centroid in the global frame plus its shape."""

    x: float
    y: float
    width: float
    height_span: float


@dataclass(frozen=True)
class RobotMotion:
    """The rover's own velocity (odometry twist)."""

    linear: float
    angular: float


@dataclass(frozen=True)
class RobotPose:
    x: float
    y: float
    theta: float

    def to_global(self, bx: float, by: float) -> Tuple[float, float]:
        c, s = math.cos(self.theta), math.sin(self.theta)
        return self.x + c * bx - s * by, self.y + s * bx + c * by


@dataclass(frozen=True)
class Track:
    state: TrackState
    x: float = 0.0
    y: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    confidence: float = 0.0

    @property
    def has_position(self) -> bool:
        return self.state != TrackState.NONE


class TargetTracker:

    def __init__(self, config: TrackerConfig):
        self._config = config
        self.reset()

    def reset(self) -> None:
        self._state = TrackState.NONE
        self._x = self._y = 0.0
        self._vx = self._vy = 0.0
        self._stamp: Optional[float] = None
        self._blocked = 0
        self._lost_at: Optional[float] = None

    @property
    def state(self) -> TrackState:
        return self._state

    def acquire_point(self, robot: RobotPose) -> Tuple[float, float]:
        return robot.to_global(self._config.acquire_x, self._config.acquire_y)

    def still(self, motion: Optional[RobotMotion]) -> bool:
        """Standing still, as far as acquisition goes; unknown motion (no odometry) is not."""
        return (motion is not None and abs(motion.linear) <= self._config.still_linear
                and abs(motion.angular) <= self._config.still_angular)

    def update(self, candidates: Iterable[Candidate], robot: RobotPose,
               motion: Optional[RobotMotion], stamp: float) -> Track:
        candidates = list(candidates)
        if self._state in (TrackState.TRACKING, TrackState.COASTING):
            return self._follow(candidates, stamp)
        if self._state == TrackState.LOST and self._lost_at is not None \
                and stamp - self._lost_at <= self._config.reacquire_time:
            track = self._reacquire(candidates, stamp)
            if track is not None:
                return track
        return self._acquire(candidates, robot, self.still(motion), stamp)

    # --- internals ------------------------------------------------------------------------------

    def _person_sized(self, c: Candidate) -> bool:
        cfg = self._config
        return cfg.min_width <= c.width <= cfg.max_width and c.height_span >= cfg.min_height_span

    def _acquire(self, candidates, robot: RobotPose, still: bool, stamp: float) -> Track:
        if still:
            ax, ay = self.acquire_point(robot)
            best = _nearest(
                (c for c in candidates if self._person_sized(c)), ax, ay,
                self._config.acquire_radius)
            if best is not None:
                return self._lock_on(best, stamp)
        if self._state == TrackState.LOST:
            return Track(TrackState.LOST, self._x, self._y)
        return Track(TrackState.NONE)

    def _reacquire(self, candidates, stamp: float) -> Optional[Track]:
        best = _nearest((c for c in candidates if self._person_sized(c)), self._x, self._y,
                        self._config.reacquire_radius)
        return None if best is None else self._lock_on(best, stamp)

    def _lock_on(self, c: Candidate, stamp: float) -> Track:
        self._state = TrackState.TRACKING
        self._x, self._y = c.x, c.y
        self._vx = self._vy = 0.0
        self._stamp = stamp
        self._blocked = self._config.max_frames_blocked
        self._lost_at = None
        return self._track(1.0)

    def _follow(self, candidates, stamp: float) -> Track:
        cfg = self._config
        dt = 0.0 if self._stamp is None else min(max(stamp - self._stamp, 0.0),
                                                 cfg.max_prediction)
        px, py = self._x + self._vx * dt, self._y + self._vy * dt
        # Partially visible people (at the edge of the image) lose height but keep their width.
        best = _nearest((c for c in candidates if c.width <= cfg.max_width), px, py,
                        cfg.tracking_radius)
        if best is not None:
            if dt > 0.0:
                a = cfg.velocity_smoothing
                self._vx = a * self._vx + (1.0 - a) * (best.x - self._x) / dt
                self._vy = a * self._vy + (1.0 - a) * (best.y - self._y) / dt
            self._x, self._y = best.x, best.y
            self._stamp = stamp
            self._blocked = cfg.max_frames_blocked
            self._state = TrackState.TRACKING
            miss = math.hypot(best.x - px, best.y - py) / cfg.tracking_radius
            return self._track(max(0.0, 1.0 - miss))
        if self._blocked > 0:
            self._blocked -= 1
            self._state = TrackState.COASTING
            # Report the prediction; keep the last measurement as the base for the next one.
            return Track(TrackState.COASTING, px, py, self._vx, self._vy,
                         self._blocked / max(cfg.max_frames_blocked, 1))
        self._state = TrackState.LOST
        self._lost_at = stamp
        self._vx = self._vy = 0.0
        return Track(TrackState.LOST, self._x, self._y)

    def _track(self, confidence: float) -> Track:
        return Track(self._state, self._x, self._y, self._vx, self._vy, confidence)


def _nearest(candidates, x: float, y: float, radius: float) -> Optional[Candidate]:
    best, best_d = None, radius
    for c in candidates:
        d = math.hypot(c.x - x, c.y - y)
        if d < best_d:
            best, best_d = c, d
    return best
