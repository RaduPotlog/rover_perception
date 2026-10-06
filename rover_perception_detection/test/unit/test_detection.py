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

import numpy as np
import pytest

from rover_perception_detection.application.detect_objects import DetectObjects
from rover_perception_detection.domain.detection import Detection, Detector
from rover_perception_detection.infrastructure.yolo_onnx import decode, letterbox, to_input_tensor

LABELS = ('person', 'car', 'dog')


class FakeDetector(Detector):
    def __init__(self, detections):
        self._d = detections

    def detect(self, image_rgb):
        return self._d


def _det(label, score, w=50, h=50):
    return Detection(0, label, score, 0, 0, w, h)


def test_use_case_filters_by_score_label_and_area():
    dets = [_det('person', 0.9), _det('person', 0.2), _det('car', 0.9), _det('person', 0.9, 2, 2)]
    out = DetectObjects(FakeDetector(dets), min_score=0.5, allowed_labels=['person'],
                        min_area_px=100).execute(np.zeros((10, 10, 3), np.uint8))
    assert [(d.label, d.score) for d in out] == [('person', 0.9)]


def test_use_case_empty_allowed_means_all():
    dets = [_det('person', 0.9), _det('car', 0.9)]
    assert len(DetectObjects(FakeDetector(dets), min_score=0.5, allowed_labels=[]).execute(
        np.zeros((1, 1, 3), np.uint8))) == 2


def test_letterbox_geometry_and_tensor():
    img = np.zeros((480, 640, 3), np.uint8)
    boxed, scale, pad_x, pad_y = letterbox(img, 640)
    assert boxed.shape == (640, 640, 3)
    assert scale == pytest.approx(1.0) and pad_x == 0 and pad_y == 80
    tensor = to_input_tensor(boxed)
    assert tensor.shape == (1, 3, 640, 640) and tensor.dtype == np.float32


def _output(rows, num_classes=3, anchors=20):
    out = np.zeros((1, 4 + num_classes, anchors), np.float32)
    for i, (cx, cy, w, h, cls, score) in enumerate(rows):
        out[0, :4, i] = (cx, cy, w, h)
        out[0, 4 + cls, i] = score
    return out


def test_decode_maps_back_to_original_pixels():
    # 640x480 image letterboxed into 640: scale 1, pad_y 80. A box centred at (320, 320) in the
    # letterboxed frame is centred at (320, 240) in the image.
    out = _output([(320, 320, 100, 60, 1, 0.9)])
    dets = decode(out, LABELS, 1.0, 0, 80, 640, 480, 0.25, 0.45)
    assert len(dets) == 1
    d = dets[0]
    assert d.label == 'car' and d.score == pytest.approx(0.9)
    assert d.center == pytest.approx((320, 240))
    assert (d.width, d.height) == pytest.approx((100, 60))


def test_decode_nms_removes_duplicates_but_keeps_other_classes():
    out = _output([
        (300, 300, 100, 100, 0, 0.9),
        (305, 302, 100, 100, 0, 0.8),   # duplicate of the first
        (300, 300, 100, 100, 2, 0.7),   # same place, different class: kept
    ])
    dets = decode(out, LABELS, 1.0, 0, 0, 640, 640, 0.25, 0.45)
    assert sorted(d.label for d in dets) == ['dog', 'person']


def test_decode_below_threshold_is_empty():
    assert decode(_output([(100, 100, 10, 10, 0, 0.1)]), LABELS, 1, 0, 0, 640, 640,
                  0.25, 0.45) == []
