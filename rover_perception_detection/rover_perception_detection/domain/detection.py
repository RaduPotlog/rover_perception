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

"""Detection entity and the Detector port. No ROS, no inference runtime (domain layer)."""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List

import numpy as np


@dataclass(frozen=True)
class Detection:
    """One object in image pixels: top-left x, y and width, height."""

    class_id: int
    label: str
    score: float
    x: float
    y: float
    width: float
    height: float

    @property
    def center(self):
        return (self.x + self.width / 2.0, self.y + self.height / 2.0)


class Detector(ABC):
    """Port: anything that finds objects in an RGB HxWx3 uint8 image."""

    @abstractmethod
    def detect(self, image_rgb: np.ndarray) -> List[Detection]:
        ...
