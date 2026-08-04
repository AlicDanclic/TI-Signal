from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

from opencv_main import DEFAULT_SCREEN_SIZE, process_frame
from vision import coarse_observation_from_points, summarize_coarse_observations


def read_image(path: Path) -> np.ndarray:
    image = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"cannot read {path}")
    return image


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Replay Task5 fixed-camera captures through coarse estimator")
    parser.add_argument("root", type=Path)
    parser.add_argument("--ramp-us", type=float, default=500.0)
    args = parser.parse_args()

    observations = []
    rejected = 0
    raw_frequencies = []
    for path in sorted(args.root.rglob("original.png")):
        try:
            result = process_frame(
                read_image(path), DEFAULT_SCREEN_SIZE, 32, None,
                args.ramp_us, False)
            observations.append(coarse_observation_from_points(result.points))
            if result.frequency_hz > 0.0:
                raw_frequencies.append(result.frequency_hz)
        except (RuntimeError, ValueError):
            rejected += 1
    summary = summarize_coarse_observations(
        observations, args.ramp_us, minimum_confidence=0.10)
    raw_span = ((min(raw_frequencies), max(raw_frequencies))
                if raw_frequencies else (0.0, 0.0))
    print(f"frames={len(observations) + rejected} extracted={len(observations)} "
          f"rejected={rejected}")
    print(f"raw_frequency_span_hz={raw_span[0]:.3f}..{raw_span[1]:.3f}")
    print(f"accepted={summary.accepted} frequency_hz={summary.frequency_hz:.3f} "
          f"points_median={summary.median_point_count} "
          f"valid_ratio={summary.valid_frame_ratio:.3f} "
          f"cv={summary.period_cv:.5f} confidence={summary.confidence:.3f} "
          f"reason={summary.reason}")
    return 0 if summary.accepted else 1


if __name__ == "__main__":
    raise SystemExit(main())
