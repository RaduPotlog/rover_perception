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

"""Detector port implemented on ONNX Runtime (CPU by default, CUDA when asked and available)."""

from typing import List, Sequence

import numpy as np

from rover_perception_detection.domain.detection import Detection, Detector
from rover_perception_detection.infrastructure.yolo_onnx import (
    decode, letterbox, to_input_tensor)


class OnnxYoloDetector(Detector):

    def __init__(
        self,
        model_path: str,
        labels: Sequence[str],
        use_gpu: bool = False,
        input_size: int = 0,
        conf_threshold: float = 0.25,
        iou_threshold: float = 0.45,
        num_threads: int = 2,
    ):
        import onnxruntime as ort  # imported here so the rest of the package works without it

        options = ort.SessionOptions()
        options.intra_op_num_threads = num_threads
        available = ort.get_available_providers()
        providers = [p for p in (('CUDAExecutionProvider',) if use_gpu else ()) if p in available]
        providers.append('CPUExecutionProvider')
        self._session = ort.InferenceSession(model_path, options, providers=providers)
        self.providers = self._session.get_providers()
        inp = self._session.get_inputs()[0]
        self._input_name = inp.name
        shape_size = inp.shape[2] if isinstance(inp.shape[2], int) else 0
        self._size = shape_size or input_size or 640
        self._labels = labels
        self._conf = conf_threshold
        self._iou = iou_threshold

    def detect(self, image_rgb: np.ndarray) -> List[Detection]:
        h, w = image_rgb.shape[:2]
        boxed, scale, pad_x, pad_y = letterbox(image_rgb, self._size)
        output = self._session.run(None, {self._input_name: to_input_tensor(boxed)})[0]
        return decode(output, self._labels, scale, pad_x, pad_y, w, h, self._conf, self._iou)
