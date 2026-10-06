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

"""Pre/post-processing for Ultralytics-style YOLO ONNX models (v8 / v11 detect heads).

Output layout is (1, 4 + num_classes, num_anchors) with boxes as centre-x, centre-y, w, h in
letterboxed-input pixels. Only numpy and OpenCV are needed, so this is testable without a model.
"""

from typing import List, Sequence, Tuple

import cv2
import numpy as np

from rover_perception_detection.domain.detection import Detection


def letterbox(image_rgb: np.ndarray, size: int) -> Tuple[np.ndarray, float, int, int]:
    """Resize keeping aspect ratio and pad to size x size. Returns (image, scale, pad_x, pad_y)."""
    h, w = image_rgb.shape[:2]
    scale = min(size / h, size / w)
    new_w, new_h = int(round(w * scale)), int(round(h * scale))
    resized = cv2.resize(image_rgb, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    pad_x, pad_y = (size - new_w) // 2, (size - new_h) // 2
    out = np.full((size, size, 3), 114, dtype=np.uint8)
    out[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = resized
    return out, scale, pad_x, pad_y


def to_input_tensor(letterboxed_rgb: np.ndarray) -> np.ndarray:
    """HxWx3 uint8 -> 1x3xHxW float32 in [0, 1]."""
    return np.ascontiguousarray(
        letterboxed_rgb.transpose(2, 0, 1)[None].astype(np.float32) / 255.0)


def decode(
    output: np.ndarray,
    labels: Sequence[str],
    scale: float,
    pad_x: int,
    pad_y: int,
    image_w: int,
    image_h: int,
    conf_threshold: float,
    iou_threshold: float,
) -> List[Detection]:
    preds = np.squeeze(output, axis=0).T  # (anchors, 4 + classes)
    class_scores = preds[:, 4:]
    class_ids = class_scores.argmax(axis=1)
    scores = class_scores[np.arange(len(preds)), class_ids]
    keep = scores >= conf_threshold
    if not keep.any():
        return []
    boxes, scores, class_ids = preds[keep, :4], scores[keep], class_ids[keep]

    # centre/size in letterboxed pixels -> top-left/size in original image pixels
    x = (boxes[:, 0] - boxes[:, 2] / 2 - pad_x) / scale
    y = (boxes[:, 1] - boxes[:, 3] / 2 - pad_y) / scale
    w, h = boxes[:, 2] / scale, boxes[:, 3] / scale
    x1, y1 = np.clip(x, 0, image_w), np.clip(y, 0, image_h)
    x2, y2 = np.clip(x + w, 0, image_w), np.clip(y + h, 0, image_h)
    rects = np.stack([x1, y1, x2 - x1, y2 - y1], axis=1)

    # class-aware NMS: offset boxes per class so different classes never suppress each other
    nms_rects = rects + (class_ids[:, None] * (max(image_w, image_h) + 1)) * np.array([1, 1, 0, 0])
    idx = cv2.dnn.NMSBoxes(nms_rects.tolist(), scores.tolist(), conf_threshold, iou_threshold)
    out = []
    for i in np.array(idx).reshape(-1):
        cid = int(class_ids[i])
        out.append(Detection(
            class_id=cid,
            label=labels[cid] if cid < len(labels) else str(cid),
            score=float(scores[i]),
            x=float(rects[i, 0]), y=float(rects[i, 1]),
            width=float(rects[i, 2]), height=float(rects[i, 3])))
    return out
