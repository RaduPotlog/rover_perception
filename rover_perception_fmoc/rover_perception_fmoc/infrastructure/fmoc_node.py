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
Lifecycle node: camera/depth/points (+ odom, TF) -> tracked_person. No logic here.

fmoc (follow-me on camera): Intel's ADBSCAN person tracking on the RealSense depth cloud. It only
perceives; rover_orchestrator's rover_follow_me decides whether the rover follows the person.
"""

import math
import time
from typing import Optional

import numpy as np
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray

from rover_perception_fmoc.application.track_person import FrameResult, TrackPerson
from rover_perception_fmoc.domain.adbscan import AdbscanConfig
from rover_perception_fmoc.domain.cloud_filter import CloudFilterConfig
from rover_perception_fmoc.domain.target_tracker import RobotMotion, RobotPose, TrackerConfig, TrackState
from rover_msgs.msg import TrackedPerson


def transform_to_matrix(transform) -> np.ndarray:
    """geometry_msgs/Transform -> 4x4 homogeneous matrix."""
    q = transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    t = transform.translation
    matrix[:3, 3] = [t.x, t.y, t.z]
    return matrix


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class FmocNode(Node):

    def __init__(self):
        super().__init__('fmoc')
        self.declare_parameter('autostart', True)
        self.declare_parameter('base_frame', 'rover/base_link')
        self.declare_parameter('global_frame', 'rover/odom')
        self.declare_parameter('process_rate', 5.0)
        self.declare_parameter('max_points', 2500)
        self.declare_parameter('odom_timeout', 0.5)
        self.declare_parameter('tf_tolerance', 0.1)
        self.declare_parameter('filter.row_stride', 5)
        self.declare_parameter('filter.col_stride', 5)
        self.declare_parameter('filter.min_x', 0.5)
        self.declare_parameter('filter.max_x', 5.0)
        self.declare_parameter('filter.y_half_width', 2.0)
        self.declare_parameter('filter.z_min', 0.15)
        self.declare_parameter('filter.z_max', 1.9)
        self.declare_parameter('adbscan.base', 5.29545454)
        self.declare_parameter('adbscan.coeff_1', -0.164835164)
        self.declare_parameter('adbscan.coeff_2', -0.017982017)
        self.declare_parameter('adbscan.scale_factor', 0.9)
        self.declare_parameter('adbscan.min_eps', 0.05)
        self.declare_parameter('adbscan.min_eps_per_m', 0.04)
        self.declare_parameter('tracker.acquire_x', 1.5)
        self.declare_parameter('tracker.acquire_y', 0.0)
        self.declare_parameter('tracker.acquire_radius', 0.6)
        self.declare_parameter('tracker.min_width', 0.15)
        self.declare_parameter('tracker.max_width', 1.0)
        self.declare_parameter('tracker.min_height_span', 0.6)
        self.declare_parameter('tracker.tracking_radius', 1.0)
        self.declare_parameter('tracker.max_frames_blocked', 5)
        self.declare_parameter('tracker.velocity_smoothing', 0.6)
        self.declare_parameter('tracker.still_linear', 0.05)
        self.declare_parameter('tracker.still_angular', 0.1)
        self.declare_parameter('tracker.reacquire_radius', 1.5)
        self.declare_parameter('tracker.reacquire_time', 10.0)
        self._use_case: Optional[TrackPerson] = None
        self._tf_buffer = None
        self._tf_listener = None
        self._cloud_sub = self._odom_sub = None
        self._target_pub = self._markers_pub = None
        self._reset_srv = None
        self._watchdog = None
        self._last_processed = 0.0
        self._last_cloud = 0.0
        self._odom: Optional[Odometry] = None
        self._odom_at = 0.0

    # --- lifecycle ------------------------------------------------------------------------------

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        p = self.get_parameter
        try:
            filter_config = CloudFilterConfig(
                row_stride=p('filter.row_stride').value, col_stride=p('filter.col_stride').value,
                min_x=p('filter.min_x').value, max_x=p('filter.max_x').value,
                y_half_width=p('filter.y_half_width').value,
                z_min=p('filter.z_min').value, z_max=p('filter.z_max').value)
            tracker_config = TrackerConfig(
                acquire_x=p('tracker.acquire_x').value, acquire_y=p('tracker.acquire_y').value,
                acquire_radius=p('tracker.acquire_radius').value,
                min_width=p('tracker.min_width').value, max_width=p('tracker.max_width').value,
                min_height_span=p('tracker.min_height_span').value,
                tracking_radius=p('tracker.tracking_radius').value,
                max_frames_blocked=p('tracker.max_frames_blocked').value,
                velocity_smoothing=p('tracker.velocity_smoothing').value,
                still_linear=p('tracker.still_linear').value,
                still_angular=p('tracker.still_angular').value,
                reacquire_radius=p('tracker.reacquire_radius').value,
                reacquire_time=p('tracker.reacquire_time').value)
        except ValueError as e:
            self.get_logger().error(f'Invalid parameters: {e}')
            return TransitionCallbackReturn.FAILURE
        adbscan_config = AdbscanConfig(
            base=p('adbscan.base').value, coeff_1=p('adbscan.coeff_1').value,
            coeff_2=p('adbscan.coeff_2').value, scale_factor=p('adbscan.scale_factor').value,
            min_eps=p('adbscan.min_eps').value, min_eps_per_m=p('adbscan.min_eps_per_m').value)
        self._use_case = TrackPerson(filter_config, adbscan_config, tracker_config,
                                     max_points=p('max_points').value)
        self._base_frame = p('base_frame').value
        self._global_frame = p('global_frame').value
        self._min_period = 1.0 / max(p('process_rate').value, 0.1)
        self._odom_timeout = p('odom_timeout').value
        self._acquire_zone = (tracker_config.acquire_x, tracker_config.acquire_y,
                              tracker_config.acquire_radius)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._tf_tolerance = p('tf_tolerance').value
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._target_pub = self.create_publisher(
            TrackedPerson, 'tracked_person',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self._markers_pub = self.create_publisher(MarkerArray, 'tracked_person/markers', 1)
        # Only the newest frame matters: depth 1 best-effort, like all camera streams; frames
        # arriving faster than process_rate are dropped.
        newest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._cloud_sub = self.create_subscription(
            PointCloud2, 'camera/depth/points', self._on_cloud, newest)
        self._odom_sub = self.create_subscription(
            Odometry, 'odom', self._on_odom, qos_profile_sensor_data)
        self._reset_srv = self.create_service(Trigger, 'tracked_person/reset', self._on_reset)
        # Also wakes the executor regularly: without any timer, an idle fmoc (no camera) did not
        # leave rclpy.spin on SIGINT under rmw_zenoh.
        self._last_cloud = time.monotonic()
        self._watchdog = self.create_timer(1.0, self._on_watchdog)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        for sub in (self._cloud_sub, self._odom_sub):
            if sub is not None:
                self.destroy_subscription(sub)
        for pub in (self._target_pub, self._markers_pub):
            if pub is not None:
                self.destroy_publisher(pub)
        if self._reset_srv is not None:
            self.destroy_service(self._reset_srv)
        if self._watchdog is not None:
            self.destroy_timer(self._watchdog)
        self._watchdog = None
        self._cloud_sub = self._odom_sub = self._target_pub = self._markers_pub = None
        self._reset_srv = None
        # A re-activated fmoc starts from scratch, not from a target seen before.
        self._use_case.reset()
        self._odom = None
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self._use_case = None
        if self._tf_listener is not None:
            self._tf_listener.unregister()
        self._tf_listener = None
        self._tf_buffer = None
        return TransitionCallbackReturn.SUCCESS

    # --- callbacks ------------------------------------------------------------------------------

    def _on_odom(self, msg: Odometry):
        self._odom = msg
        self._odom_at = time.monotonic()

    def _on_reset(self, request, response):
        self._use_case.reset()
        response.success = True
        response.message = 'target dropped; stand in the acquire zone to be picked up again'
        return response

    def _on_watchdog(self):
        silent = time.monotonic() - self._last_cloud
        if silent > 2.0:
            self.get_logger().warning(
                f'No depth cloud on camera/depth/points for {silent:.0f} s '
                '(ROVER_SYSTEM_USE_CAMERA, ROVER_SENSORS_CAMERA_DEPTH_CLOUD)',
                throttle_duration_sec=30.0)

    def _on_cloud(self, msg: PointCloud2):
        now = time.monotonic()
        self._last_cloud = now
        if now - self._last_processed < self._min_period:
            return
        stamp = Time.from_msg(msg.header.stamp)
        try:
            sensor_to_base = self._tf_buffer.lookup_transform(
                self._base_frame, msg.header.frame_id, Time())  # a fixed mount
            base_in_global = self._robot_transform(stamp)
        except tf2_ros.TransformException as e:
            self.get_logger().warning(f'No transform for the depth cloud: {e}',
                                      throttle_duration_sec=5.0)
            return
        self._last_processed = now

        points = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'))
        if msg.height > 1:
            points = points.reshape(msg.height, msg.width, 3)
        t = base_in_global.transform
        robot = RobotPose(t.translation.x, t.translation.y, yaw_from_quaternion(t.rotation))
        result = self._use_case.process(
            points, transform_to_matrix(sensor_to_base.transform), robot, self._robot_motion(),
            stamp.nanoseconds * 1e-9)
        self._publish_target(result, msg.header.stamp)
        self._publish_markers(result, msg.header.stamp)

    # --- helpers --------------------------------------------------------------------------------

    def _robot_transform(self, stamp: Time):
        """
        base_frame in global_frame at the cloud's stamp.

        The odom transform for a cloud's stamp often arrives a few ms after the cloud; rather
        than wait for it (which needs a second executor thread), take the newest one when it is
        at most tf_tolerance older than the cloud.
        """
        try:
            return self._tf_buffer.lookup_transform(self._global_frame, self._base_frame, stamp)
        except tf2_ros.ExtrapolationException:
            latest = self._tf_buffer.lookup_transform(
                self._global_frame, self._base_frame, Time())
            age = (stamp - Time.from_msg(latest.header.stamp)).nanoseconds * 1e-9
            if abs(age) > self._tf_tolerance:
                raise
            return latest

    def _robot_motion(self) -> Optional[RobotMotion]:
        """The rover's twist from odom; None when odom is missing or stale (no acquisition)."""
        if self._odom is None or time.monotonic() - self._odom_at > self._odom_timeout:
            self.get_logger().warning('No recent odom; not acquiring a target',
                                      throttle_duration_sec=10.0)
            return None
        twist = self._odom.twist.twist
        return RobotMotion(math.hypot(twist.linear.x, twist.linear.y), twist.angular.z)

    def _publish_target(self, result: FrameResult, stamp):
        track = result.track
        msg = TrackedPerson()
        msg.header.stamp = stamp
        msg.header.frame_id = self._global_frame
        msg.state = int(track.state)
        msg.position = Point(x=track.x, y=track.y, z=0.0)
        msg.velocity = Vector3(x=track.vx, y=track.vy, z=0.0)
        msg.confidence = float(track.confidence)
        msg.source = 'fmoc'
        self._target_pub.publish(msg)

    def _publish_markers(self, result: FrameResult, stamp):
        markers = MarkerArray()
        markers.markers.append(Marker(action=Marker.DELETEALL))
        for i, cluster in enumerate(result.clusters):
            box = Marker()
            box.header.stamp = stamp
            box.header.frame_id = self._base_frame
            box.ns = 'clusters'
            box.id = i
            box.type = Marker.CUBE
            cx, cy, cz = cluster.center
            box.pose.position = Point(x=cx, y=cy, z=cz)
            box.pose.orientation.w = 1.0
            sx, sy, sz = cluster.size
            box.scale.x, box.scale.y, box.scale.z = max(sx, 0.02), max(sy, 0.02), max(sz, 0.02)
            box.color.r, box.color.g, box.color.b, box.color.a = 0.6, 0.6, 0.6, 0.4
            markers.markers.append(box)

        zone = Marker()
        zone.header.stamp = stamp
        zone.header.frame_id = self._base_frame
        zone.ns = 'acquire_zone'
        zone.type = Marker.CYLINDER
        ax, ay, radius = self._acquire_zone
        zone.pose.position = Point(x=ax, y=ay, z=0.01)
        zone.pose.orientation.w = 1.0
        zone.scale.x = zone.scale.y = 2.0 * radius
        zone.scale.z = 0.02
        zone.color.r, zone.color.g, zone.color.b, zone.color.a = 0.2, 0.4, 1.0, 0.3
        markers.markers.append(zone)

        track = result.track
        if track.state in (TrackState.TRACKING, TrackState.COASTING):
            target = Marker()
            target.header.stamp = stamp
            target.header.frame_id = self._global_frame
            target.ns = 'target'
            target.type = Marker.CYLINDER
            target.pose.position = Point(x=track.x, y=track.y, z=0.9)
            target.pose.orientation.w = 1.0
            target.scale.x = target.scale.y = 0.5
            target.scale.z = 1.8
            tracking = track.state == TrackState.TRACKING
            target.color.r, target.color.g, target.color.b, target.color.a = (
                (0.1, 0.9, 0.2, 0.6) if tracking else (1.0, 0.8, 0.1, 0.6))
            markers.markers.append(target)
        self._markers_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = FmocNode()
    if node.get_parameter('autostart').value:
        if node.trigger_configure() == TransitionCallbackReturn.SUCCESS:
            node.trigger_activate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.try_shutdown()
        except KeyboardInterrupt:  # a second SIGINT (launch forwards one to the whole group)
            pass
