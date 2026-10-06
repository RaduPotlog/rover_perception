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

"""Use case: run a Detector and keep only the detections the rover cares about."""

from typing import Collection, List, Optional

import numpy as np

from rover_perception_detection.domain.detection import Detection, Detector


class DetectObjects:

    def __init__(
        self,
        detector: Detector,
        min_score: float = 0.4,
        allowed_labels: Optional[Collection[str]] = None,
        min_area_px: float = 0.0,
    ):
        self._detector = detector
        self._min_score = min_score
        self._allowed = set(allowed_labels) if allowed_labels else None  # empty = all classes
        self._min_area = min_area_px

    def execute(self, image_rgb: np.ndarray) -> List[Detection]:
        return [
            d for d in self._detector.detect(image_rgb)
            if d.score >= self._min_score
            and (self._allowed is None or d.label in self._allowed)
            and d.width * d.height >= self._min_area
        ]
