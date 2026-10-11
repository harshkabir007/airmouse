"""Person detector backends. Each returns [(x1, y1, x2, y2, confidence), ...] in pixels."""
import cv2
import numpy as np


class UltralyticsDetector:
    """YOLO via ultralytics. `model` may be .pt, .onnx or an exported *_ncnn_model dir (fastest on Pi 5)."""

    def __init__(self, model, imgsz, conf):
        from ultralytics import YOLO
        self.model = YOLO(model, task='detect')
        self.imgsz = int(imgsz)
        self.conf = float(conf)

    def detect(self, bgr):
        res = self.model.predict(bgr, imgsz=self.imgsz, conf=self.conf, classes=[0], verbose=False)[0]
        out = []
        for b in res.boxes:
            x1, y1, x2, y2 = (float(v) for v in b.xyxy[0].tolist())
            out.append((x1, y1, x2, y2, float(b.conf[0])))
        return out


class HogDetector:
    """OpenCV HOG people detector: no extra dependencies, weak. Only for smoke tests."""

    def __init__(self, conf):
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        self.conf = float(conf)

    def detect(self, bgr):
        rects, weights = self.hog.detectMultiScale(bgr, winStride=(8, 8), padding=(8, 8), scale=1.05)
        weights = np.asarray(weights).reshape(-1)
        return [(float(x), float(y), float(x + w), float(y + h), float(min(1.0, s / 2.0)))
                for (x, y, w, h), s in zip(rects, weights) if s / 2.0 >= self.conf]


def make_detector(backend, model, imgsz, conf, logger):
    if backend == 'ultralytics':
        try:
            return UltralyticsDetector(model, imgsz, conf)
        except Exception as exc:  # noqa: BLE001 - fall back rather than kill the mission
            logger.error(f'ultralytics backend failed ({exc}); falling back to HOG')
    return HogDetector(conf)
