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

import pytest

from rover_perception_fmoc.domain.target_tracker import (
    Candidate, RobotMotion, RobotPose, TargetTracker, TrackerConfig, TrackState)

CFG = TrackerConfig(acquire_x=1.5, acquire_radius=0.6, tracking_radius=1.0,
                    max_frames_blocked=3, reacquire_radius=1.5, reacquire_time=10.0)
ORIGIN = RobotPose(0.0, 0.0, 0.0)
STILL = RobotMotion(0.0, 0.0)
MOVING = RobotMotion(0.5, 0.0)


def person(x, y, width=0.4, height=1.2):
    return Candidate(x, y, width, height)


def test_acquires_only_a_person_sized_cluster_near_the_acquire_point():
    tracker = TargetTracker(CFG)
    assert tracker.update([person(3.0, 0.0)], ORIGIN, STILL, 0.0).state == TrackState.NONE
    assert tracker.update([person(1.5, 0.0, width=2.0)], ORIGIN, STILL, 0.1).state ==         TrackState.NONE  # a wall
    assert tracker.update([person(1.5, 0.0, height=0.3)], ORIGIN, STILL, 0.2).state ==         TrackState.NONE  # a box
    track = tracker.update([person(3.0, 0.0), person(1.6, 0.1)], ORIGIN, STILL, 0.3)
    assert track.state == TrackState.TRACKING
    assert (track.x, track.y) == (1.6, 0.1)


def test_does_not_acquire_while_the_rover_moves():
    tracker = TargetTracker(CFG)
    assert tracker.update([person(1.5, 0.0)], ORIGIN, MOVING, 0.0).state == TrackState.NONE


def test_acquire_point_follows_the_rover_pose():
    tracker = TargetTracker(CFG)
    robot = RobotPose(10.0, 5.0, math.pi / 2)
    track = tracker.update([person(10.0, 6.5)], robot, STILL, 0.0)
    assert track.state == TrackState.TRACKING


def test_tracks_a_walking_person_and_ignores_a_crossing_one():
    tracker = TargetTracker(CFG)
    tracker.update([person(1.5, 0.0)], ORIGIN, STILL, 0.0)
    t = 0.0
    x = 1.5
    for _ in range(30):
        t += 0.2
        x += 0.2  # 1 m/s
        other = person(x + 1.3, 0.0)  # someone walking further ahead, out of reach
        track = tracker.update([other, person(x, 0.0)], RobotPose(x - 2.0, 0.0, 0.0), MOVING, t)
        assert track.state == TrackState.TRACKING
        assert track.x == x
    assert track.vx == pytest.approx(1.0, rel=0.05)


def test_coasts_then_loses():
    tracker = TargetTracker(CFG)
    tracker.update([person(1.5, 0.0)], ORIGIN, STILL, 0.0)
    states = [tracker.update([], ORIGIN, STILL, 0.1 * i).state for i in range(1, 6)]
    assert states == [TrackState.COASTING] * 3 + [TrackState.LOST, TrackState.LOST]


def _lost_at(x, stamp=0.4):
    tracker = TargetTracker(CFG)
    tracker.update([person(1.5, 0.0)], ORIGIN, STILL, 0.0)
    tracker.update([person(x, 0.0)], ORIGIN, MOVING, 0.1)
    for i in range(2, 6):
        tracker.update([], ORIGIN, MOVING, 0.1 * i)
    assert tracker.state == TrackState.LOST
    return tracker


def test_reacquires_near_the_last_sighting_while_moving():
    tracker = _lost_at(2.0)
    track = tracker.update([person(3.2, 0.3)], ORIGIN, MOVING, 2.0)
    assert track.state == TrackState.TRACKING and (track.x, track.y) == (3.2, 0.3)


def test_does_not_reacquire_far_away_or_too_late():
    tracker = _lost_at(2.0)
    assert tracker.update([person(4.0, 0.0)], ORIGIN, MOVING, 2.0).state == TrackState.LOST
    assert tracker.update([person(2.5, 0.0, width=2.0)], ORIGIN, MOVING, 2.1).state == \
        TrackState.LOST  # a wall near the last sighting
    assert tracker.update([person(2.5, 0.0)], ORIGIN, MOVING, 20.0).state == TrackState.LOST
    # Later, only the acquire zone with the rover stopped.
    assert tracker.update([person(1.5, 0.0)], ORIGIN, STILL, 21.0).state == TrackState.TRACKING


def test_no_odometry_means_no_acquisition():
    tracker = TargetTracker(CFG)
    assert tracker.update([person(1.5, 0.0)], ORIGIN, None, 0.0).state == TrackState.NONE
    assert not tracker.still(RobotMotion(0.0, 0.5))
    assert tracker.still(RobotMotion(0.01, -0.05))


def test_partially_visible_person_keeps_being_tracked():
    tracker = TargetTracker(CFG)
    tracker.update([person(1.5, 0.0)], ORIGIN, STILL, 0.0)
    track = tracker.update([person(1.6, 0.5, height=0.2)], ORIGIN, STILL, 0.1)
    assert track.state == TrackState.TRACKING


def test_reset_forgets_the_target():
    tracker = TargetTracker(CFG)
    tracker.update([person(1.5, 0.0)], ORIGIN, STILL, 0.0)
    tracker.reset()
    assert tracker.update([person(2.5, 0.0)], ORIGIN, STILL, 0.1).state == TrackState.NONE
