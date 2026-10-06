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

"""ROS 2 adapter: rslidar_points -> terrain/incline. All declared parameters, no logic here."""

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

from rover_perception_terrain.application.estimate_terrain import EstimateTerrain


class TerrainNode(Node):

    def __init__(self):
        super().__init__('terrain_node')
        self.declare_parameter('target_frame', 'base_footprint')
        self.declare_parameter('max_rate_hz', 5.0)
        self.declare_parameter('roi.min_x', 0.5)
        self.declare_parameter('roi.max_x', 4.0)
        self.declare_parameter('roi.max_abs_y', 1.5)
        self.declare_parameter('roi.max_abs_z', 0.6)
        self.declare_parameter('ransac.max_points', 2000)
        self.declare_parameter('ransac.distance_threshold', 0.05)
        self.declare_parameter('ransac.min_inlier_ratio', 0.3)
        self.declare_parameter('smoothing', 0.5)

        p = self.get_parameter
        self._target_frame = p('target_frame').value
        self._min_period = 1.0 / max(p('max_rate_hz').value, 0.1)
        self._use_case = EstimateTerrain(
            min_x=p('roi.min_x').value, max_x=p('roi.max_x').value,
            max_abs_y=p('roi.max_abs_y').value, max_abs_z=p('roi.max_abs_z').value,
            max_points=p('ransac.max_points').value,
            distance_threshold=p('ransac.distance_threshold').value,
            min_inlier_ratio=p('ransac.min_inlier_ratio').value,
            smoothing=p('smoothing').value)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._last_t = 0.0

        self._incline_pub = self.create_publisher(Vector3Stamped, 'terrain/incline', 10)
        self._confidence_pub = self.create_publisher(Float32, 'terrain/ground_confidence', 10)
        # Lidar clouds are best-effort sensor data.
        self.create_subscription(PointCloud2, 'rslidar_points', self._on_cloud,
                                 qos_profile_sensor_data)

    def _on_cloud(self, msg: PointCloud2):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_t < self._min_period:
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame, msg.header.frame_id, rclpy.time.Time.from_msg(msg.header.stamp))
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(
                f'No {msg.header.frame_id} -> {self._target_frame} transform: {exc}',
                throttle_duration_sec=5.0)
            return
        self._last_t = now
        cloud = do_transform_cloud(msg, transform)
        xyz = point_cloud2.read_points_numpy(cloud, field_names=('x', 'y', 'z'), skip_nans=True)
        estimate = self._use_case.execute(np.asarray(xyz, dtype=np.float64))
        if estimate is None:
            self.get_logger().warn('No ground plane found', throttle_duration_sec=5.0)
            return
        out = Vector3Stamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._target_frame
        out.vector.x = estimate.incline_forward_deg
        out.vector.y = estimate.incline_lateral_deg
        out.vector.z = estimate.slope_deg
        self._incline_pub.publish(out)
        self._confidence_pub.publish(Float32(data=float(estimate.confidence)))


def main(args=None):
    rclpy.init(args=args)
    node = TerrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
