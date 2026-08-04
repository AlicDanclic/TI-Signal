from __future__ import annotations

import argparse
import csv
import json
import sys
from dataclasses import asdict
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml

from vision import WaveformPointExtractor


def read_image(path: Path) -> np.ndarray:
    data = np.fromfile(path, dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"cannot decode image: {path}")
    return image


def write_image(path: Path, image: np.ndarray) -> None:
    suffix = path.suffix or ".png"
    ok, encoded = cv2.imencode(suffix, image)
    if not ok:
        raise RuntimeError(f"cannot encode image: {path}")
    encoded.tofile(path)


def order_corners(points: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=np.float32).reshape(4, 2)
    ordered = np.empty((4, 2), dtype=np.float32)
    sums = points.sum(axis=1)
    differences = np.diff(points, axis=1).reshape(-1)
    ordered[0] = points[np.argmin(sums)]
    ordered[2] = points[np.argmax(sums)]
    ordered[1] = points[np.argmin(differences)]
    ordered[3] = points[np.argmax(differences)]
    return ordered


def detect_scope_corners(frame: np.ndarray) -> np.ndarray:
    """Find the largest green phosphor-screen quadrilateral."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        # A higher saturation floor rejects the pale green bezel reflection
        # and keeps the actual phosphor glass as the dominant contour.
        np.asarray([35, 45, 30], np.uint8),
        np.asarray([105, 255, 255], np.uint8),
    )
    scale = min(frame.shape[:2])
    kernel_size = max(11, int(round(scale * 0.025)) | 1)
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    minimum_area = frame.shape[0] * frame.shape[1] * 0.08
    candidates = [
        contour for contour in contours
        if cv2.contourArea(contour) >= minimum_area
    ]
    if not candidates:
        raise RuntimeError(
            "scope screen was not detected; pass --corners with four points")
    contour = max(candidates, key=cv2.contourArea)
    hull = cv2.convexHull(contour)
    perimeter = cv2.arcLength(hull, True)
    polygon = cv2.approxPolyDP(hull, 0.012 * perimeter, True)
    if len(polygon) == 4:
        corners = polygon.reshape(4, 2)
    else:
        corners = cv2.boxPoints(cv2.minAreaRect(hull))
    return order_corners(corners)


def rectify(frame: np.ndarray, corners: np.ndarray,
            size: tuple[int, int]) -> np.ndarray:
    width, height = size
    destination = np.asarray(
        [[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]],
        np.float32,
    )
    transform = cv2.getPerspectiveTransform(order_corners(corners), destination)
    return cv2.warpPerspective(frame, transform, (width, height))


def parse_corners(values: list[float] | None) -> np.ndarray | None:
    if values is None:
        return None
    return order_corners(np.asarray(values, np.float32).reshape(4, 2))


def load_config(path: Path | None) -> dict[str, Any]:
    if path is None or not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def capture_camera(index: int, config: dict[str, Any],
                   warmup_frames: int) -> np.ndarray:
    camera = config.get("camera", {})
    capture = cv2.VideoCapture(index)
    if not capture.isOpened():
        raise RuntimeError(f"cannot open camera index {index}")
    try:
        properties = (
            (cv2.CAP_PROP_FRAME_WIDTH, camera.get("width", 1280), "width"),
            (cv2.CAP_PROP_FRAME_HEIGHT, camera.get("height", 720), "height"),
            (cv2.CAP_PROP_FPS, camera.get("fps", 30), "fps"),
            (cv2.CAP_PROP_AUTO_EXPOSURE,
             camera.get("auto_exposure", 1), "auto exposure"),
            (cv2.CAP_PROP_EXPOSURE, camera.get("exposure"), "exposure"),
            (cv2.CAP_PROP_GAIN, camera.get("gain"), "gain"),
            (cv2.CAP_PROP_FOCUS, camera.get("focus"), "focus"),
        )
        for property_id, value, name in properties:
            if value is not None and not capture.set(property_id, float(value)):
                print(f"warning: camera backend rejected {name}={value}",
                      file=sys.stderr)
        for _ in range(max(1, warmup_frames)):
            ok, frame = capture.read()
            if not ok or frame is None:
                raise RuntimeError("camera returned no frame")
        return frame
    finally:
        capture.release()


def save_points(output_dir: Path, source: str, corners: np.ndarray,
                rectified: np.ndarray, result) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    overlay = WaveformPointExtractor.render_overlay(rectified, result)
    write_image(output_dir / "rectified.png", rectified)
    write_image(output_dir / "trace_mask.png", result.trace_mask)
    write_image(output_dir / "points_overlay.png", overlay)

    field_names = [
        "index", "x_px", "y_px", "x_normalized", "y_normalized",
        "y_volts", "time_normalized", "strength",
    ]
    with (output_dir / "points.csv").open(
            "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=field_names)
        writer.writeheader()
        for index, point in enumerate(result.points):
            row = asdict(point)
            row["index"] = index
            writer.writerow(row)

    payload = {
        "source": source,
        "screen_corners": corners.astype(float).tolist(),
        "rectified_size": [int(rectified.shape[1]), int(rectified.shape[0])],
        "reference": asdict(result.calibration),
        "point_count": len(result.points),
        "points": [asdict(point) for point in result.points],
    }
    with (output_dir / "points.json").open("w", encoding="utf-8") as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Extract sparse Task5 pulse-ramp points from an XY scope image")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--image", type=Path, help="input scope photograph")
    source.add_argument("--camera", type=int, help="OpenCV camera index")
    parser.add_argument("--config", type=Path,
                        default=Path(__file__).with_name("config.yaml"))
    parser.add_argument("--output-dir", type=Path,
                        default=Path("point_output"))
    parser.add_argument(
        "--corners", type=float, nargs=8,
        metavar=("TL_X", "TL_Y", "TR_X", "TR_Y",
                 "BR_X", "BR_Y", "BL_X", "BL_Y"),
        help="manual screen corners; otherwise the green screen is detected")
    parser.add_argument("--size", type=int, nargs=2, default=(800, 640),
                        metavar=("WIDTH", "HEIGHT"))
    parser.add_argument("--max-points", type=int,
                        help="maximum number of sparse points")
    parser.add_argument("--warmup-frames", type=int, default=15)
    parser.add_argument("--show", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = load_config(args.config)
    if args.image is not None:
        frame = read_image(args.image)
        source_name = str(args.image)
    else:
        frame = capture_camera(args.camera, config, args.warmup_frames)
        source_name = f"camera:{args.camera}"

    corners = parse_corners(args.corners)
    if corners is None:
        corners = detect_scope_corners(frame)
    screen = rectify(frame, corners, (int(args.size[0]), int(args.size[1])))
    extractor = WaveformPointExtractor(config)
    result = extractor.extract(screen, args.max_points)
    save_points(args.output_dir, source_name, corners, screen, result)

    detection = frame.copy()
    cv2.polylines(detection, [corners.round().astype(np.int32)], True,
                  (0, 0, 255), 4, cv2.LINE_AA)
    write_image(args.output_dir / "screen_detection.png", detection)

    calibration = result.calibration
    print(f"points={len(result.points)}")
    print(f"reference_confidence={calibration.confidence:.3f}")
    print("screen_corners=" + json.dumps(corners.astype(int).tolist()))
    print(f"outputs={args.output_dir.resolve()}")
    if args.show:
        cv2.imshow("screen detection", detection)
        cv2.imshow("rectified", screen)
        cv2.imshow("trace mask", result.trace_mask)
        cv2.imshow("waveform points", WaveformPointExtractor.render_overlay(
            screen, result))
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
