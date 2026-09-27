"""
yolo_ncnn.py — Lightweight YOLO11n NCNN inference wrapper (no torch, no ultralytics).

Uses the ncnn Python bindings directly to run the exported YOLO11n NCNN FP16 model.
Handles preprocessing, inference, postprocessing (decode + NMS) and returns
detections in a simple dataclass format.

Tested on Raspberry Pi 4B, Raspberry Pi OS 64-bit (Trixie), ncnn 1.0.20260526.
"""

import time
import numpy as np
import cv2
import ncnn
from dataclasses import dataclass, field
from typing import List, Optional


# ──────────────────────────────────────────────────────────────────────────────
# Detection dataclass
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class Detection:
    """Single detected object."""
    track_id: int          # -1 if no tracker assigned yet
    class_id: int          # COCO class index (0 = person)
    class_name: str
    confidence: float

    # Image-space bounding box (pixels, in original frame coordinates)
    bbox_x1: int
    bbox_y1: int
    bbox_x2: int
    bbox_y2: int

    # Derived image-space fields
    center_x: int = 0
    center_y: int = 0
    bbox_width: int = 0
    bbox_height: int = 0

    # Normalized center (0.0–1.0 relative to frame size)
    norm_cx: float = 0.0
    norm_cy: float = 0.0

    timestamp_ms: int = 0  # ms since epoch

    def __post_init__(self):
        self.bbox_width  = self.bbox_x2 - self.bbox_x1
        self.bbox_height = self.bbox_y2 - self.bbox_y1
        self.center_x    = self.bbox_x1 + self.bbox_width  // 2
        self.center_y    = self.bbox_y1 + self.bbox_height // 2

    def set_normalized(self, frame_w: int, frame_h: int):
        """Call after construction to fill norm_cx / norm_cy."""
        self.norm_cx = self.center_x / frame_w if frame_w > 0 else 0.5
        self.norm_cy = self.center_y / frame_h if frame_h > 0 else 0.5

    def __repr__(self):
        return (f"Detection(id={self.track_id} {self.class_name} "
                f"conf={self.confidence:.2f} "
                f"box=[{self.bbox_x1},{self.bbox_y1},{self.bbox_x2},{self.bbox_y2}] "
                f"norm=({self.norm_cx:.3f},{self.norm_cy:.3f}))")


# ──────────────────────────────────────────────────────────────────────────────
# COCO class names (80 classes)
# ──────────────────────────────────────────────────────────────────────────────

COCO_CLASSES = [
    "person","bicycle","car","motorcycle","airplane","bus","train","truck",
    "boat","traffic light","fire hydrant","stop sign","parking meter","bench",
    "bird","cat","dog","horse","sheep","cow","elephant","bear","zebra","giraffe",
    "backpack","umbrella","handbag","tie","suitcase","frisbee","skis","snowboard",
    "sports ball","kite","baseball bat","baseball glove","skateboard","surfboard",
    "tennis racket","bottle","wine glass","cup","fork","knife","spoon","bowl",
    "banana","apple","sandwich","orange","broccoli","carrot","hot dog","pizza",
    "donut","cake","chair","couch","potted plant","bed","dining table","toilet",
    "tv","laptop","mouse","remote","keyboard","cell phone","microwave","oven",
    "toaster","sink","refrigerator","book","clock","vase","scissors","teddy bear",
    "hair drier","toothbrush",
]

PERSON_CLASS_ID = 0


# ──────────────────────────────────────────────────────────────────────────────
# YoloNcnn — inference engine
# ──────────────────────────────────────────────────────────────────────────────

class YoloNcnn:
    """
    Wraps the NCNN yolo11n model for person detection.

    Usage:
        detector = YoloNcnn("/home/pi/yolo11n_ncnn_model")
        frame = cv2.VideoCapture(0).read()[1]
        detections = detector.detect(frame)
        for d in detections:
            print(d)
    """

    def __init__(
        self,
        model_dir: str,
        input_size: int = 320,
        conf_threshold: float = 0.40,
        nms_threshold: float  = 0.45,
        target_classes: Optional[List[int]] = None,
        num_threads: int = 4,
    ):
        """
        Args:
            model_dir:       Path to the exported NCNN model directory
                             (must contain model.ncnn.param and model.ncnn.bin).
            input_size:      YOLO input resolution (must match export — 320).
            conf_threshold:  Minimum confidence to keep a detection.
            nms_threshold:   IOU threshold for NMS.
            target_classes:  List of COCO class IDs to keep (None = all).
                             Default: [0] (person only).
            num_threads:     NCNN inference threads (4 = all Pi 4 cores).
        """
        import os
        self.input_size    = input_size
        self.conf_thresh   = conf_threshold
        self.nms_thresh    = nms_threshold
        self.target_cls    = target_classes if target_classes is not None else [PERSON_CLASS_ID]
        self.num_threads   = num_threads

        param_path = os.path.join(model_dir, "model.ncnn.param")
        bin_path   = os.path.join(model_dir, "model.ncnn.bin")

        if not os.path.exists(param_path):
            raise FileNotFoundError(f"NCNN param not found: {param_path}")
        if not os.path.exists(bin_path):
            raise FileNotFoundError(f"NCNN bin not found: {bin_path}")

        self._net = ncnn.Net()
        self._net.opt.use_vulkan_compute = False   # CPU only on Pi
        self._net.opt.num_threads = num_threads

        ret = self._net.load_param(param_path)
        if ret != 0:
            raise RuntimeError(f"Failed to load NCNN param: {param_path}")
        ret = self._net.load_model(bin_path)
        if ret != 0:
            raise RuntimeError(f"Failed to load NCNN bin: {bin_path}")

        self._frame_w = 0
        self._frame_h = 0
        print(f"[YoloNcnn] Loaded model from {model_dir}")
        print(f"[YoloNcnn] input={input_size}px  conf>={conf_threshold}  nms={nms_threshold}  threads={num_threads}")

    # ── Public API ────────────────────────────────────────────────────

    def detect(self, frame: np.ndarray) -> List[Detection]:
        """
        Run detection on a BGR frame (from cv2.VideoCapture).

        Returns a list of Detection objects (may be empty).
        Applies conf_threshold and NMS internally.
        Only returns classes in self.target_cls.
        """
        if frame is None or frame.size == 0:
            return []

        h, w = frame.shape[:2]
        self._frame_w = w
        self._frame_h = h

        # 1. Preprocess: letterbox + BGR→RGB + HWC→CHW + normalise
        inp, scale, (pad_x, pad_y) = self._letterbox(frame)

        mat_in = ncnn.Mat.from_pixels(
            inp.astype(np.uint8),
            ncnn.Mat.PixelType.PIXEL_BGR2RGB,
            self.input_size,
            self.input_size,
        )
        # Normalise to [0,1]
        mean_vals = [0.0, 0.0, 0.0]
        norm_vals = [1 / 255.0, 1 / 255.0, 1 / 255.0]
        mat_in.substract_mean_normalize(mean_vals, norm_vals)

        # 2. Inference
        ex = self._net.create_extractor()
        ex.input("in0", mat_in)
        ret, mat_out = ex.extract("out0")
        if ret != 0:
            return []

        # 3. Postprocess: decode YOLO11 output tensor
        return self._decode(mat_out, scale, pad_x, pad_y, w, h)

    # ── Internal helpers ──────────────────────────────────────────────

    def _letterbox(self, img: np.ndarray):
        """Resize with letterbox padding to (input_size × input_size)."""
        h, w = img.shape[:2]
        s = self.input_size
        scale = min(s / w, s / h)
        nw, nh = int(w * scale), int(h * scale)
        resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
        canvas = np.full((s, s, 3), 114, dtype=np.uint8)
        pad_x = (s - nw) // 2
        pad_y = (s - nh) // 2
        canvas[pad_y:pad_y + nh, pad_x:pad_x + nw] = resized
        return canvas, scale, (pad_x, pad_y)

    def _decode(self, mat_out, scale, pad_x, pad_y, orig_w, orig_h) -> List[Detection]:
        """
        Decode YOLO11 head output into Detection objects.

        YOLO11 NCNN output shape: [84, 2100] for 320×320 input
          - 84 = 4 (cx,cy,w,h) + 80 (class scores)
          - 2100 = 20×20 + 40×40 + 80×80 anchors (for input 320)
        Note: output is already sigmoid-activated inside NCNN graph.
        """
        out = np.array(mat_out)    # shape: (84, N) or (N, 84)
        if out.ndim == 1:
            return []

        # Ensure shape is (84, N) — transpose if needed
        if out.shape[0] != 84:
            out = out.T
        if out.shape[0] != 84:
            return []

        num_proposals = out.shape[1]
        ts = int(time.time() * 1000)

        boxes, scores, class_ids = [], [], []

        for i in range(num_proposals):
            col = out[:, i]
            cx, cy, bw, bh = col[0], col[1], col[2], col[3]
            cls_scores = col[4:]

            best_cls  = int(np.argmax(cls_scores))
            best_conf = float(cls_scores[best_cls])

            if best_conf < self.conf_thresh:
                continue
            if self.target_cls and best_cls not in self.target_cls:
                continue

            # Un-letterbox: remove padding, undo scale
            x1 = int((cx - bw / 2 - pad_x) / scale)
            y1 = int((cy - bh / 2 - pad_y) / scale)
            x2 = int((cx + bw / 2 - pad_x) / scale)
            y2 = int((cy + bh / 2 - pad_y) / scale)

            x1 = max(0, min(x1, orig_w - 1))
            y1 = max(0, min(y1, orig_h - 1))
            x2 = max(0, min(x2, orig_w - 1))
            y2 = max(0, min(y2, orig_h - 1))

            if x2 <= x1 or y2 <= y1:
                continue

            boxes.append([x1, y1, x2 - x1, y2 - y1])  # x,y,w,h for NMS
            scores.append(best_conf)
            class_ids.append(best_cls)

        if not boxes:
            return []

        # NMS using OpenCV (no scipy needed)
        indices = cv2.dnn.NMSBoxes(boxes, scores, self.conf_thresh, self.nms_thresh)
        if len(indices) == 0:
            return []

        detections = []
        for idx in indices.flatten():
            x, y, w, h = boxes[idx]
            d = Detection(
                track_id   = -1,
                class_id   = class_ids[idx],
                class_name = COCO_CLASSES[class_ids[idx]],
                confidence = scores[idx],
                bbox_x1    = x,
                bbox_y1    = y,
                bbox_x2    = x + w,
                bbox_y2    = y + h,
                timestamp_ms = ts,
            )
            d.set_normalized(orig_w, orig_h)
            detections.append(d)

        return detections


# ──────────────────────────────────────────────────────────────────────────────
# Quick smoke-test (run directly: python yolo_ncnn.py)
# ──────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import sys, os

    MODEL_DIR = os.path.expanduser("~/yolo11n_ncnn_model")
    detector  = YoloNcnn(MODEL_DIR, conf_threshold=0.4)

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[ERROR] Cannot open /dev/video0")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print("[TEST] Running 10 inference frames on webcam. Press Ctrl-C to stop.")
    for i in range(10):
        t0 = time.time()
        ret, frame = cap.read()
        if not ret:
            print("[ERROR] Frame grab failed")
            break

        detections = detector.detect(frame)
        elapsed_ms = int((time.time() - t0) * 1000)

        if detections:
            for d in detections:
                print(f"  Frame {i+1}: {d}  [{elapsed_ms}ms]")
        else:
            print(f"  Frame {i+1}: no person detected  [{elapsed_ms}ms]")

    cap.release()
    print("[TEST] Done.")
