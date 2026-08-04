from __future__ import annotations

from pathlib import Path
from typing import Any

import cv2
import numpy as np


class ScopeCamera:
    def __init__(self, config: dict[str, Any], source: str | int | None = None) -> None:
        camera_config = config.get("camera", {})
        selected = camera_config.get("device", 0) if source is None else source
        if isinstance(selected, str) and selected.isdigit():
            selected = int(selected)
        self._is_file = isinstance(selected, str) and Path(selected).exists()
        self._capture = cv2.VideoCapture(selected)
        if not self._capture.isOpened():
            raise RuntimeError(f"cannot open camera/video source: {selected}")
        if not self._is_file:
            self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, camera_config.get("width", 1280))
            self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_config.get("height", 720))
            self._capture.set(cv2.CAP_PROP_FPS, camera_config.get("fps", 30))
            self._capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, camera_config.get("auto_exposure", 1))
            exposure = camera_config.get("exposure")
            if exposure is not None:
                self._capture.set(cv2.CAP_PROP_EXPOSURE, exposure)
        self._config = camera_config
        output = config.get("vision", {}).get("canonical_size", [640, 480])
        self._output_size = (int(output[0]), int(output[1]))

    def close(self) -> None:
        self._capture.release()

    def _read_capture(self) -> np.ndarray:
        ok, frame = self._capture.read()
        if not ok and self._is_file:
            self._capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self._capture.read()
        if not ok or frame is None:
            raise RuntimeError("camera returned no frame")
        return frame

    def read_raw(self) -> np.ndarray:
        """Return one camera frame before the runtime ROI transformation."""
        return self._read_capture()

    def read(self) -> np.ndarray:
        return self._rectify(self._read_capture())

    def _rectify(self, frame: np.ndarray) -> np.ndarray:
        points = self._config.get("perspective_points")
        if points and len(points) == 4:
            source = np.asarray(points, dtype=np.float32)
            width, height = self._output_size
            destination = np.asarray(
                [[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]],
                dtype=np.float32,
            )
            transform = cv2.getPerspectiveTransform(source, destination)
            return cv2.warpPerspective(frame, transform, self._output_size)

        roi = self._config.get("roi", [0.0, 0.0, 1.0, 1.0])
        height, width = frame.shape[:2]
        x, y, roi_width, roi_height = [float(value) for value in roi]
        if max(x, y, roi_width, roi_height) <= 1.0:
            x, roi_width = x * width, roi_width * width
            y, roi_height = y * height, roi_height * height
        x0 = max(0, min(width - 1, int(round(x))))
        y0 = max(0, min(height - 1, int(round(y))))
        x1 = max(x0 + 1, min(width, int(round(x + roi_width))))
        y1 = max(y0 + 1, min(height, int(round(y + roi_height))))
        return cv2.resize(frame[y0:y1, x0:x1], self._output_size,
                          interpolation=cv2.INTER_AREA)
