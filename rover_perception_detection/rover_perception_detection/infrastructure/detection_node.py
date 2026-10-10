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

"""Lifecycle node: image topic -> vision_msgs/Detection2DArray. Owns the model, so lifecycle."""

import time

from cv_bridge import CvBridge
import rclpy
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from rover_perception_detection.application.detect_objects import DetectObjects
from rover_perception_detection.domain.coco_labels import COCO_LABELS
from rover_perception_detection.infrastructure.onnx_detector import OnnxYoloDetector


class DetectionNode(Node):

    def __init__(self):
        super().__init__('detection_node')
        self.declare_parameter('model_path', '')
        self.declare_parameter('use_gpu', False)
        self.declare_parameter('input_size', 640)
        self.declare_parameter('num_threads', 2)
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('min_score', 0.4)
        self.declare_parameter('allowed_labels', [''])  # [''] = every class
        self.declare_parameter('max_rate_hz', 3.0)
        self.declare_parameter('autostart', True)
        self._bridge = CvBridge()
        self._use_case = None
        self._sub = None
        self._pub = None
        self._last_t = 0.0

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        p = self.get_parameter
        model_path = p('model_path').value
        if not model_path:
            self.get_logger().error('model_path is empty: set it to a YOLO .onnx file')
            return TransitionCallbackReturn.FAILURE
        try:
            detector = OnnxYoloDetector(
                model_path, COCO_LABELS, use_gpu=p('use_gpu').value,
                input_size=p('input_size').value, conf_threshold=p('conf_threshold').value,
                iou_threshold=p('iou_threshold').value, num_threads=p('num_threads').value)
        except Exception as exc:  # missing file, missing onnxruntime, bad model
            self.get_logger().error(f'Cannot load detector: {exc}')
            return TransitionCallbackReturn.FAILURE
        allowed = [s for s in p('allowed_labels').value if s]
        self._use_case = DetectObjects(detector, p('min_score').value, allowed)
        self._min_period = 1.0 / max(p('max_rate_hz').value, 0.1)
        self.get_logger().info(f'Detector ready, providers={detector.providers}')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._pub = self.create_publisher(Detection2DArray, 'detections', 10)
        # Only the newest frame matters: depth 1, best-effort, like all camera streams.
        self._sub = self.create_subscription(
            Image, 'image_raw', self._on_image, qos_profile_sensor_data)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.destroy_subscription(self._sub)
        self.destroy_publisher(self._pub)
        self._sub = self._pub = None
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self._use_case = None  # frees the inference session
        return TransitionCallbackReturn.SUCCESS

    def _on_image(self, msg: Image):
        now = time.monotonic()
        if now - self._last_t < self._min_period:
            return
        self._last_t = now
        try:
            rgb = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as exc:
            self.get_logger().warn(f'Bad image: {exc}', throttle_duration_sec=5.0)
            return
        out = Detection2DArray()
        out.header = msg.header
        for d in self._use_case.execute(rgb):
            det = Detection2D()
            det.header = msg.header
            det.bbox.center.position.x, det.bbox.center.position.y = d.center
            det.bbox.size_x, det.bbox.size_y = d.width, d.height
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = d.label
            hyp.hypothesis.score = d.score
            det.results.append(hyp)
            out.detections.append(det)
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    if node.get_parameter('autostart').value:
        if node.trigger_configure() == TransitionCallbackReturn.SUCCESS:
            node.trigger_activate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
