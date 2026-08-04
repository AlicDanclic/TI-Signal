from __future__ import annotations

from dataclasses import dataclass
from math import ceil, floor, pi
from typing import Any

import cv2
import numpy as np


@dataclass(frozen=True)
class ProbeFit:
    cycles: float
    phase_radians: float
    confidence: float
    sample_count: int


@dataclass(frozen=True)
class DualProbeFit:
    grid_frequency_hz: int
    tuning_word: int
    phase_difference_cycles: float
    confidence: float
    fit_a: ProbeFit
    fit_b: ProbeFit


@dataclass(frozen=True)
class DualPhaseFit:
    phase_difference_cycles: float
    confidence: float
    fit_a: ProbeFit
    fit_b: ProbeFit


@dataclass(frozen=True)
class CoarseFrameObservation:
    point_count: int
    left_periods: tuple[float, ...]
    right_periods: tuple[float, ...]
    confidence: float


@dataclass(frozen=True)
class CoarseMeasurement:
    accepted: bool
    frequency_hz: float
    period_cv: float
    valid_frame_ratio: float
    median_point_count: int
    complete_period_count: int
    confidence: float
    reason: str


@dataclass(frozen=True)
class FineFrequencyFit:
    frequency_hz: float
    correction_hz: float
    residual_cycles: float
    confidence: float


def wrap_cycles(value: float) -> float:
    return (float(value) + 0.5) % 1.0 - 0.5


def circular_mean_cycles(values: list[float] | tuple[float, ...]) -> float:
    if not values:
        raise ValueError("at least one phase sample is required")
    angles = np.asarray(values, np.float64) * (2.0 * pi)
    vector = np.mean(np.exp(1j * angles))
    if abs(vector) < 1e-9:
        raise ValueError("phase samples have no circular consensus")
    return wrap_cycles(float(np.angle(vector) / (2.0 * pi)))


def reject_integer_multiple_periods(
        periods: list[float] | tuple[float, ...],
        tolerance: float = 0.20,
        maximum_multiple: int = 5) -> list[float]:
    """Keep the direct-period cluster and discard missed-point multiples."""
    values = np.asarray([value for value in periods if value > 0.0], np.float64)
    if values.size == 0:
        return []
    candidates = np.concatenate(
        [values / multiple for multiple in range(1, maximum_multiple + 1)])
    best_key: tuple[float, int, float] | None = None
    best_center = 0.0
    for candidate in candidates:
        if candidate <= 0.0:
            continue
        ratios = values / candidate
        nearest = np.rint(ratios)
        harmonic = ((nearest >= 1.0) & (nearest <= maximum_multiple) &
                    (np.abs(ratios - nearest) <= tolerance))
        direct = np.abs(ratios - 1.0) <= tolerance
        # A real direct-period cluster outranks a hypothetical sub-harmonic.
        score = 3.0 * float(np.count_nonzero(direct)) + float(np.count_nonzero(harmonic))
        key = (score, int(np.count_nonzero(direct)), float(candidate))
        if best_key is None or key > best_key:
            best_key = key
            best_center = float(candidate)
    direct_values = values[np.abs(values / best_center - 1.0) <= tolerance]
    if direct_values.size == 0:
        return []
    center = float(np.median(direct_values))
    return [float(value) for value in values
            if abs(value / center - 1.0) <= tolerance]


def coarse_observation_from_points(
        points: list[Any], ramp_height_px: float = 469.05) -> CoarseFrameObservation:
    if ramp_height_px <= 0.0:
        raise ValueError("ramp height must be positive")
    sides: dict[int, list[float]] = {0: [], 1: []}
    strengths: list[float] = []
    for point in points:
        side = 0 if float(point.x_normalized) < 0.0 else 1
        sides[side].append(float(point.y_px))
        strengths.append(float(point.strength))

    periods: dict[int, tuple[float, ...]] = {}
    for side, times in sides.items():
        ordered = sorted(times)
        periods[side] = tuple(
            (second - first) / ramp_height_px
            for first, second in zip(ordered, ordered[1:])
            if second > first)
    confidence = float(np.median(strengths)) if strengths else 0.0
    return CoarseFrameObservation(
        len(points), periods[0], periods[1], confidence)


def summarize_coarse_observations(
        observations: list[CoarseFrameObservation], width_us: float,
        minimum_points: int = 5, minimum_periods: int = 3,
        minimum_valid_ratio: float = 0.70, maximum_cv: float = 0.08,
        minimum_confidence: float = 0.35,
        maximum_points: int = 22) -> CoarseMeasurement:
    if not observations:
        return CoarseMeasurement(False, 0.0, 1.0, 0.0, 0, 0, 0.0,
                                 "NO_FRAMES")
    point_counts = np.asarray([item.point_count for item in observations])
    median_points = int(round(float(np.median(point_counts))))
    if median_points > maximum_points:
        return CoarseMeasurement(False, 0.0, 1.0, 0.0, median_points, 0, 0.0,
                                 "VISUAL_RANGE_HIGH")

    frame_periods: list[float] = []
    frame_confidences: list[float] = []
    complete_period_count = 0
    for observation in observations:
        left = reject_integer_multiple_periods(observation.left_periods)
        right = reject_integer_multiple_periods(observation.right_periods)
        combined_count = len(left) + len(right)
        if (observation.point_count < minimum_points or not left or not right or
                combined_count < minimum_periods or
                observation.confidence < minimum_confidence):
            continue
        side_periods = [float(np.median(left)), float(np.median(right))]
        frame_periods.append(float(np.median(side_periods)))
        frame_confidences.append(observation.confidence)
        complete_period_count += combined_count

    valid_ratio = len(frame_periods) / len(observations)
    if not frame_periods:
        return CoarseMeasurement(False, 0.0, 1.0, valid_ratio, median_points,
                                 complete_period_count, 0.0, "NO_VALID_PERIODS")
    periods = np.asarray(frame_periods, np.float64)
    period = float(np.median(periods))
    mad = float(np.median(np.abs(periods - period)))
    robust_sigma = 1.4826 * mad
    cv = robust_sigma / max(period, 1e-12)
    frequency = 1_000_000.0 / (period * width_us)
    visual_confidence = float(np.median(frame_confidences))
    ratio_score = min(1.0, valid_ratio / max(minimum_valid_ratio, 1e-6))
    cv_score = max(0.0, 1.0 - cv / max(maximum_cv, 1e-6))
    confidence = float(np.clip(
        0.45 * visual_confidence + 0.35 * ratio_score + 0.20 * cv_score,
        0.0, 1.0))

    reason = "OK"
    accepted = True
    if median_points < minimum_points:
        accepted, reason = False, "TOO_FEW_POINTS"
    elif complete_period_count < minimum_periods:
        accepted, reason = False, "TOO_FEW_PERIODS"
    elif valid_ratio < minimum_valid_ratio:
        accepted, reason = False, "LOW_VALID_FRAME_RATIO"
    elif cv > maximum_cv:
        accepted, reason = False, "PERIOD_UNSTABLE"
    elif confidence < minimum_confidence:
        accepted, reason = False, "LOW_CONFIDENCE"
    return CoarseMeasurement(
        accepted, frequency, cv, valid_ratio, median_points,
        complete_period_count, confidence, reason)


def resolve_dual_interval_frequency(
        coarse_frequency_hz: float, phase_3ms_cycles: float,
        phase_7ms_cycles: float, coarse_uncertainty_hz: float = 450.0,
        confidence_3ms: float = 1.0, confidence_7ms: float = 1.0,
        maximum_residual_cycles: float = 0.08) -> FineFrequencyFit:
    """Jointly unwrap the FPGA-timed 3 ms and 7 ms phase measurements."""
    if coarse_frequency_hz <= 0.0 or coarse_uncertainty_hz <= 0.0:
        raise ValueError("coarse frequency and uncertainty must be positive")
    phase_3 = wrap_cycles(phase_3ms_cycles)
    phase_7 = wrap_cycles(phase_7ms_cycles)
    low = coarse_frequency_hz - coarse_uncertainty_hz
    high = coarse_frequency_hz + coarse_uncertainty_hz
    dt_3 = 0.003
    dt_7 = 0.007
    first_n = floor(low * dt_7 - phase_7) - 1
    last_n = ceil(high * dt_7 - phase_7) + 1
    candidates: list[tuple[float, float, float]] = []
    weight_3 = max(0.05, confidence_3ms)
    weight_7 = max(0.05, confidence_7ms)
    for cycle_count in range(first_n, last_n + 1):
        frequency = (cycle_count + phase_7) / dt_7
        if not low <= frequency <= high:
            continue
        residual_3 = abs(wrap_cycles(frequency * dt_3 - phase_3))
        residual_7 = abs(wrap_cycles(frequency * dt_7 - phase_7))
        phase_cost = residual_3 * residual_3 / weight_3 + residual_7 * residual_7 / weight_7
        coarse_cost = 1e-4 * (
            (frequency - coarse_frequency_hz) / coarse_uncertainty_hz) ** 2
        candidates.append((phase_cost + coarse_cost, frequency,
                           max(residual_3, residual_7)))
    if not candidates:
        raise ValueError("no 3/7 ms frequency candidate in coarse range")
    candidates.sort()
    _, frequency, residual = candidates[0]
    if residual > maximum_residual_cycles:
        raise ValueError(f"3/7 ms phase residual is too large: {residual:.4f} cycles")
    if len(candidates) > 1 and abs(candidates[1][0] - candidates[0][0]) < 1e-6:
        raise ValueError("3/7 ms phase result is ambiguous")
    confidence = min(confidence_3ms, confidence_7ms)
    confidence *= max(0.0, 1.0 - residual / maximum_residual_cycles)
    return FineFrequencyFit(
        frequency, frequency - coarse_frequency_hz, residual,
        float(np.clip(confidence, 0.0, 1.0)))


@dataclass(frozen=True)
class TargetFit:
    estimated_phase: int
    desired_score: float
    quality: int
    span_x_div: float
    span_y_div: float
    center_error_div: float


@dataclass(frozen=True)
class ReferenceCalibration:
    top_y: float
    bottom_y: float
    left_x: float
    right_x: float
    top_band: tuple[int, int]
    bottom_band: tuple[int, int]
    confidence: float


@dataclass(frozen=True)
class WaveformPoint:
    x_px: float
    y_px: float
    x_normalized: float
    y_normalized: float
    y_volts: float
    time_normalized: float
    strength: float


@dataclass
class WaveformPointResult:
    calibration: ReferenceCalibration
    points: list[WaveformPoint]
    trace_mask: np.ndarray


class TraceExtractor:
    def __init__(self, config: dict[str, Any]) -> None:
        vision = config.get("vision", {})
        self._hsv_low = np.asarray(vision.get("hsv_low", [25, 60, 90]), np.uint8)
        self._hsv_high = np.asarray(vision.get("hsv_high", [100, 255, 255]), np.uint8)
        self._minimum_pixels = int(vision.get("minimum_trace_pixels", 150))
        self._brightness_threshold = int(vision.get("brightness_threshold", 165))

    def extract(self, frame: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        color_mask = cv2.inRange(hsv, self._hsv_low, self._hsv_high)
        if cv2.countNonZero(color_mask) >= self._minimum_pixels:
            mask = color_mask
        else:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            threshold = max(self._brightness_threshold,
                            int(np.percentile(gray, 97.5)))
            mask = cv2.inRange(gray, threshold, 255)

        height, width = mask.shape
        if cv2.countNonZero(mask) > height * width // 8:
            horizontal = cv2.morphologyEx(
                mask, cv2.MORPH_OPEN,
                cv2.getStructuringElement(cv2.MORPH_RECT, (max(15, width // 12), 1)))
            vertical = cv2.morphologyEx(
                mask, cv2.MORPH_OPEN,
                cv2.getStructuringElement(cv2.MORPH_RECT, (1, max(15, height // 10))))
            mask = cv2.subtract(mask, cv2.bitwise_or(horizontal, vertical))

        mask = cv2.morphologyEx(
            mask, cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2)))
        return mask


class WaveformPointExtractor:
    """Extract sparse points from the Task5 pulse-ramp XY pattern.

    The two bright idle lines are useful calibration features rather than
    waveform samples. They define Y=+2 V, Y=-2 V, and the full horizontal X
    sweep. The extractor masks those bands, enhances the remaining phosphor
    trace, finds separated row-activity peaks, and returns one weighted point
    for each visible pulse-ramp sample group.
    """

    def __init__(self, config: dict[str, Any] | None = None) -> None:
        vision = (config or {}).get("vision", {})
        settings = vision.get("point_extraction", {})
        self._reference_search_fraction = float(
            settings.get("reference_search_fraction", 0.35))
        self._reference_margin_px = int(
            settings.get("reference_margin_px", 8))
        self._horizontal_crop_fraction = float(
            settings.get("horizontal_crop_fraction", 0.05))
        self._activity_fraction = float(
            settings.get("activity_fraction", 0.04))
        self._peak_floor_percentile = float(
            settings.get("peak_floor_percentile", 58.0))
        self._profile_percentile = float(
            settings.get("profile_percentile", 85.0))
        self._maximum_points = int(settings.get("maximum_points", 64))
        self._minimum_points = int(settings.get("minimum_points", 5))
        self._minimum_green_excess = float(
            settings.get("minimum_green_excess", 30.0))
        self._minimum_brightness = float(
            settings.get("minimum_brightness", 155.0))
        self._minimum_reference_green_excess = float(
            settings.get("minimum_reference_green_excess", 35.0))
        self._minimum_reference_contrast = float(
            settings.get("minimum_reference_contrast", 12.0))
        self._minimum_reference_confidence = float(
            settings.get("minimum_reference_confidence", 0.55))

    @staticmethod
    def _green_excess(frame: np.ndarray) -> np.ndarray:
        blue, green, red = cv2.split(frame.astype(np.int16))
        return np.clip(green - np.maximum(blue, red), 0, 255).astype(np.uint8)

    @staticmethod
    def _top_fraction_mean(values: np.ndarray, fraction: float,
                           axis: int) -> np.ndarray:
        count = max(1, int(round(values.shape[axis] * fraction)))
        partitioned = np.partition(values, -count, axis=axis)
        indices = [slice(None)] * values.ndim
        indices[axis] = slice(-count, None)
        return np.mean(partitioned[tuple(indices)], axis=axis)

    @staticmethod
    def _smooth_1d(values: np.ndarray, size: int) -> np.ndarray:
        size = max(3, int(size) | 1)
        return cv2.GaussianBlur(
            values.astype(np.float32).reshape(-1, 1), (1, size), 0
        ).reshape(-1)

    @staticmethod
    def _reference_band(activity: np.ndarray, start: int,
                        stop: int) -> tuple[int, int, float, float]:
        region = activity[start:stop]
        if region.size < 3:
            raise ValueError("reference search region is too small")
        peak = start + int(np.argmax(region))
        baseline = float(np.percentile(region, 35.0))
        peak_value = float(activity[peak])
        threshold = baseline + 0.35 * max(0.0, peak_value - baseline)
        lower = peak
        upper = peak
        while lower > start and activity[lower - 1] >= threshold:
            lower -= 1
        while upper + 1 < stop and activity[upper + 1] >= threshold:
            upper += 1
        rows = np.arange(lower, upper + 1, dtype=np.float32)
        weights = np.maximum(activity[lower:upper + 1] - baseline, 0.0)
        center = (float(np.sum(rows * weights) / np.sum(weights))
                  if float(np.sum(weights)) > 0.0 else float(peak))
        prominence = ((peak_value - baseline) /
                      max(1.0, float(np.percentile(region, 95.0))))
        return lower, upper, center, max(0.0, prominence)

    def detect_reference_lines(self, frame: np.ndarray) -> ReferenceCalibration:
        score = self._green_excess(frame).astype(np.float32)
        height, width = score.shape
        x_margin = max(2, int(round(width * self._horizontal_crop_fraction)))
        row_source = score[:, x_margin:width - x_margin]
        row_activity = self._top_fraction_mean(row_source, 0.35, axis=1)
        row_activity = self._smooth_1d(row_activity, max(5, height // 58))

        search = min(0.48, max(0.2, self._reference_search_fraction))
        top_start = max(0, int(round(height * 0.015)))
        top_stop = max(top_start + 3, int(round(height * search)))
        bottom_start = min(height - 3, int(round(height * (1.0 - search))))
        bottom_stop = min(height, int(round(height * 0.985)))
        top_lower, top_upper, top_y, top_prominence = self._reference_band(
            row_activity, top_start, top_stop)
        bottom_lower, bottom_upper, bottom_y, bottom_prominence = (
            self._reference_band(row_activity, bottom_start, bottom_stop))
        if bottom_y - top_y < height * 0.45:
            raise ValueError("upper and lower reference lines are too close")

        reference_rows = np.concatenate((
            np.arange(top_lower, top_upper + 1),
            np.arange(bottom_lower, bottom_upper + 1),
        ))
        reference_pixels = score[reference_rows, :]
        baseline = float(np.percentile(reference_pixels, 35.0))
        bright = float(np.percentile(reference_pixels, 98.0))
        reference_level = float(np.percentile(reference_pixels, 75.0))
        if (reference_level < self._minimum_reference_green_excess or
                bright - baseline < self._minimum_reference_contrast):
            raise ValueError(
                "the +/-2 V reference lines are missing or too dim; "
                "shorten the Task5 ramp below 10 ms and lock exposure")
        threshold = baseline + 0.35 * max(1.0, bright - baseline)
        coverage = np.mean(reference_pixels >= threshold, axis=0)
        columns = np.flatnonzero(coverage >= 0.12)
        if columns.size >= max(20, width // 5):
            left_x, right_x = np.percentile(columns, [1.0, 99.0])
        else:
            raise ValueError(
                "the +/-2 V reference lines do not have enough horizontal "
                "coverage")
        if right_x - left_x < width * 0.35:
            raise ValueError("reference lines do not span enough screen width")

        separation_score = min(1.0, (bottom_y - top_y) / (height * 0.7))
        span_score = min(1.0, (right_x - left_x) / (width * 0.65))
        confidence = min(1.0, 0.35 * top_prominence +
                         0.35 * bottom_prominence +
                         0.15 * separation_score + 0.15 * span_score)
        if confidence < self._minimum_reference_confidence:
            raise ValueError(
                f"reference-line confidence is too low: {confidence:.3f}")
        return ReferenceCalibration(
            top_y=top_y,
            bottom_y=bottom_y,
            left_x=float(left_x),
            right_x=float(right_x),
            top_band=(top_lower, top_upper),
            bottom_band=(bottom_lower, bottom_upper),
            confidence=float(confidence),
        )

    @staticmethod
    def _runs(profile: np.ndarray, threshold: float) -> list[tuple[int, int]]:
        active = profile >= threshold
        padded = np.pad(active.astype(np.int8), (1, 1))
        changes = np.diff(padded)
        starts = np.flatnonzero(changes == 1)
        stops = np.flatnonzero(changes == -1)
        return list(zip(starts.tolist(), stops.tolist()))

    def extract(self, frame: np.ndarray,
                maximum_points: int | None = None) -> WaveformPointResult:
        calibration = self.detect_reference_lines(frame)
        score = self._green_excess(frame)
        brightness = np.max(frame, axis=2).astype(np.uint8)
        height, width = score.shape

        kernel_size = max(15, int(round(min(height, width) * 0.045)) | 1)
        background = cv2.morphologyEx(
            score,
            cv2.MORPH_OPEN,
            cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)),
        )
        enhanced = cv2.subtract(score, background).astype(np.float32)

        y_start = min(height - 2, calibration.top_band[1] +
                      self._reference_margin_px + 1)
        y_stop = max(y_start + 2, calibration.bottom_band[0] -
                     self._reference_margin_px)
        horizontal_span = calibration.right_x - calibration.left_x
        x_start = max(1, int(round(calibration.left_x -
                                  horizontal_span * 0.03)))
        x_stop = min(width - 1, int(round(calibration.right_x +
                                         horizontal_span * 0.03)))
        if y_stop - y_start < 20 or x_stop - x_start < 20:
            raise ValueError("reference-line mask leaves too little trace area")

        valid_values = enhanced[y_start:y_stop, x_start:x_stop]
        valid_green = score[y_start:y_stop, x_start:x_stop]
        valid_brightness = brightness[y_start:y_stop, x_start:x_stop]
        binary_threshold = max(2.0, float(np.percentile(valid_values, 94.0)))
        trace_mask = np.zeros((height, width), np.uint8)
        trace_mask[y_start:y_stop, x_start:x_stop] = np.where(
            (valid_values >= binary_threshold) &
            (valid_green >= self._minimum_green_excess) &
            (valid_brightness >= self._minimum_brightness),
            255, 0).astype(np.uint8)
        trace_mask = cv2.morphologyEx(
            trace_mask, cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_RECT, (3, 1)))

        row_source = enhanced[:, x_start:x_stop]
        row_activity = self._top_fraction_mean(
            row_source, self._activity_fraction, axis=1)
        row_activity = self._smooth_1d(row_activity, max(3, height // 128))
        active_rows = row_activity[y_start:y_stop]
        peak_floor = float(np.percentile(
            active_rows, self._peak_floor_percentile))
        candidates: list[tuple[float, int]] = []
        for row in range(y_start + 1, y_stop - 1):
            value = float(row_activity[row])
            if (value >= peak_floor and
                    value >= float(row_activity[row - 1]) and
                    value >= float(row_activity[row + 1])):
                candidates.append((value, row))

        point_limit = max(1, int(maximum_points or self._maximum_points))
        minimum_spacing = max(4, height // 80)
        selected_rows: list[tuple[float, int]] = []
        for value, row in sorted(candidates, reverse=True):
            if all(abs(row - selected) > minimum_spacing
                   for _, selected in selected_rows):
                selected_rows.append((value, row))
        selected_rows.sort(key=lambda item: item[1])

        points: list[WaveformPoint] = []
        band_radius = max(1, height // 256)
        minimum_run_width = max(2, width // 400)
        activity_high = max(peak_floor + 1e-6,
                            float(np.percentile(active_rows, 95.0)))
        for row_value, row in selected_rows:
            profile = np.max(
                enhanced[max(y_start, row - band_radius):
                         min(y_stop, row + band_radius + 1),
                         x_start:x_stop],
                axis=0,
            )
            green_profile = np.max(
                score[max(y_start, row - band_radius):
                      min(y_stop, row + band_radius + 1),
                      x_start:x_stop],
                axis=0,
            )
            brightness_profile = np.max(
                brightness[max(y_start, row - band_radius):
                           min(y_stop, row + band_radius + 1),
                           x_start:x_stop],
                axis=0,
            )
            profile = np.where(
                (green_profile >= self._minimum_green_excess) &
                (brightness_profile >= self._minimum_brightness),
                profile, 0.0)
            profile_threshold = max(
                2.0, float(np.percentile(profile, self._profile_percentile)))
            runs = self._runs(profile, profile_threshold)
            scored_runs: list[tuple[float, float, int]] = []
            for run_start, run_stop in runs:
                run_width = run_stop - run_start
                if run_width < minimum_run_width:
                    continue
                values = profile[run_start:run_stop]
                weights = np.maximum(values - profile_threshold + 1.0, 1.0)
                columns = np.arange(run_start, run_stop, dtype=np.float32)
                power = float(np.sum(values))
                center = float(np.sum(columns * weights) / np.sum(weights))
                scored_runs.append((power, center, run_width))
            if not scored_runs:
                continue
            scored_runs.sort(reverse=True)
            best_power, center, _ = scored_runs[0]
            competing_power = sum(item[0] for item in scored_runs[:3])
            dominance = best_power / max(best_power, competing_power)
            peak_strength = (row_value - peak_floor) / (activity_high - peak_floor)
            strength = float(np.clip(
                0.55 * peak_strength + 0.45 * dominance, 0.0, 1.0))
            x_px = center + x_start
            y_px = float(row)
            x_normalized = 2.0 * (x_px - calibration.left_x) / horizontal_span - 1.0
            y_normalized = 1.0 - 2.0 * (
                y_px - calibration.top_y) / (
                    calibration.bottom_y - calibration.top_y)
            points.append(WaveformPoint(
                x_px=x_px,
                y_px=y_px,
                x_normalized=float(np.clip(x_normalized, -1.25, 1.25)),
                y_normalized=float(np.clip(y_normalized, -1.1, 1.1)),
                y_volts=float(np.clip(2.0 * y_normalized, -2.2, 2.2)),
                time_normalized=float(np.clip(
                    (y_normalized + 1.0) * 0.5, 0.0, 1.0)),
                strength=strength,
            ))

        # The ramp starts at -2 V and rises to +2 V, so descending image Y is
        # the real time order. CSV/JSON indices therefore run from ramp start
        # to ramp end instead of top-to-bottom screen order.
        points.sort(key=lambda point: point.y_px, reverse=True)
        if len(points) > point_limit:
            # Apply the limit only after color/brightness validation. Limiting
            # candidate rows earlier can select a rejected grid edge and leave
            # fewer points than the caller requested. Even spacing here keeps
            # the complete -2 V to +2 V time range represented.
            sample_indices = np.linspace(
                0, len(points) - 1, point_limit).round().astype(int)
            points = [points[index] for index in np.unique(sample_indices)]
        required_points = min(self._minimum_points, point_limit)
        if len(points) < required_points:
            raise ValueError(
                f"only {len(points)} waveform points found; check focus, "
                "exposure, perspective points, and green thresholds")
        return WaveformPointResult(calibration, points, trace_mask)

    @staticmethod
    def render_overlay(frame: np.ndarray,
                       result: WaveformPointResult) -> np.ndarray:
        overlay = frame.copy()
        calibration = result.calibration
        cv2.line(overlay, (round(calibration.left_x), round(calibration.top_y)),
                 (round(calibration.right_x), round(calibration.top_y)),
                 (0, 80, 255), 2)
        cv2.line(overlay, (round(calibration.left_x), round(calibration.bottom_y)),
                 (round(calibration.right_x), round(calibration.bottom_y)),
                 (255, 80, 0), 2)
        for index, point in enumerate(result.points):
            center = (round(point.x_px), round(point.y_px))
            cv2.circle(overlay, center, 5, (0, 0, 255), -1, cv2.LINE_AA)
            cv2.putText(overlay, str(index), (center[0] + 6, center[1] - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (20, 20, 255), 1,
                        cv2.LINE_AA)
        cv2.putText(overlay, "+2 V reference",
                    (round(calibration.left_x),
                     max(14, round(calibration.top_y) - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 80, 255), 1,
                    cv2.LINE_AA)
        cv2.putText(overlay, "-2 V reference",
                    (round(calibration.left_x),
                     min(frame.shape[0] - 5, round(calibration.bottom_y) + 18)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 80, 0), 1,
                    cv2.LINE_AA)
        return overlay


class FrequencyEstimator:
    WIDTHS_US = {0: 100.0, 1: 500.0, 2: 2000.0, 3: 5000.0}

    @staticmethod
    def _centerline(mask: np.ndarray, y_top: float, y_bottom: float) -> tuple[np.ndarray, np.ndarray]:
        height, width = mask.shape
        top = max(0, min(height - 1, int(round(y_top * height))))
        bottom = max(top + 1, min(height, int(round(y_bottom * height))))
        q_values: list[float] = []
        x_values: list[float] = []
        span = max(1, bottom - top - 1)
        for row in range(top, bottom):
            columns = np.flatnonzero(mask[row])
            columns = columns[(columns > width * 0.02) & (columns < width * 0.98)]
            if columns.size:
                q_values.append((bottom - 1 - row) / span)
                x_values.append(float(np.median(columns)) / max(1, width - 1))
        if len(q_values) < 30:
            raise ValueError("not enough trace rows for sinusoid fitting")
        order = np.argsort(q_values)
        return np.asarray(q_values)[order], np.asarray(x_values)[order]

    @staticmethod
    def _fit(q: np.ndarray, x: np.ndarray, minimum_cycles: float,
             maximum_cycles: float, expected_cycles: float | None = None) -> ProbeFit:
        if expected_cycles is not None:
            minimum_cycles = max(minimum_cycles, expected_cycles - 0.8)
            maximum_cycles = min(maximum_cycles, expected_cycles + 0.8)
        if maximum_cycles <= minimum_cycles:
            raise ValueError("invalid cycle search range")

        centered = x - np.mean(x)
        trend = np.polyfit(q, centered, 1)
        centered = centered - np.polyval(trend, q)
        coarse = np.arange(minimum_cycles, maximum_cycles + 0.0101, 0.01)
        best_cycle = float(coarse[0])
        best_power = -1.0
        for start in range(0, coarse.size, 256):
            candidates = coarse[start:start + 256]
            angle = 2.0 * pi * candidates[:, None] * q[None, :]
            projection = np.exp(-1j * angle) @ centered
            powers = np.abs(projection) ** 2
            index = int(np.argmax(powers))
            if float(powers[index]) > best_power:
                best_power = float(powers[index])
                best_cycle = float(candidates[index])

        fine = np.arange(max(minimum_cycles, best_cycle - 0.025),
                         min(maximum_cycles, best_cycle + 0.025) + 0.000251,
                         0.00025)
        best_residual = float("inf")
        best_coefficients: np.ndarray | None = None
        for cycles in fine:
            angle = 2.0 * pi * cycles * q
            matrix = np.column_stack((np.ones_like(q), q, np.sin(angle), np.cos(angle)))
            coefficients, _, _, _ = np.linalg.lstsq(matrix, x, rcond=None)
            residual = float(np.mean((x - matrix @ coefficients) ** 2))
            if residual < best_residual:
                best_residual = residual
                best_cycle = float(cycles)
                best_coefficients = coefficients

        assert best_coefficients is not None
        signal_std = max(1e-6, float(np.std(x)))
        confidence = max(0.0, min(1.0, 1.0 - best_residual ** 0.5 / signal_std))
        phase = float(np.arctan2(best_coefficients[3], best_coefficients[2]))
        return ProbeFit(best_cycle, phase, confidence, int(q.size))

    def estimate_single(self, mask: np.ndarray, width_code: int) -> ProbeFit:
        q, x = self._centerline(mask, 0.05, 0.95)
        maximum = {0: 12.0, 1: 55.0, 2: 205.0, 3: 505.0}.get(width_code, 55.0)
        return self._fit(q, x, 0.05, maximum)

    def estimate_dual_phase(self, mask: np.ndarray, width_code: int,
                            coarse_frequency_hz: float) -> DualPhaseFit:
        width_us = self.WIDTHS_US[width_code]
        expected_cycles = coarse_frequency_hz * width_us / 1_000_000.0
        q_a, x_a = self._centerline(mask, 0.55, 0.95)
        q_b, x_b = self._centerline(mask, 0.05, 0.45)
        fit_a = self._fit(q_a, x_a, 0.05, 205.0, expected_cycles)
        fit_b = self._fit(q_b, x_b, 0.05, 205.0, expected_cycles)
        phase_cycles = wrap_cycles(
            (fit_b.phase_radians - fit_a.phase_radians) / (2.0 * pi))
        return DualPhaseFit(
            phase_cycles, min(fit_a.confidence, fit_b.confidence), fit_a, fit_b)

    def estimate_dual(self, mask: np.ndarray, width_code: int,
                      coarse_frequency_hz: float, offset_us: int) -> DualProbeFit:
        phase_fit = self.estimate_dual_phase(
            mask, width_code, coarse_frequency_hz)
        fit_a = phase_fit.fit_a
        fit_b = phase_fit.fit_b
        phase_cycles = phase_fit.phase_difference_cycles
        coarse_grid = int(round(coarse_frequency_hz / 100.0) * 100)
        candidates = range(max(1000, coarse_grid - 600),
                           min(100000, coarse_grid + 600) + 1, 100)

        def candidate_error(frequency: int) -> float:
            expected_fraction = (frequency * offset_us / 1_000_000.0 + 0.5) % 1.0 - 0.5
            circular = abs((expected_fraction - phase_cycles + 0.5) % 1.0 - 0.5)
            coarse_penalty = abs(frequency - coarse_frequency_hz) / 5000.0
            return circular + coarse_penalty

        grid_frequency = min(candidates, key=candidate_error)
        expected_offset_cycles = grid_frequency * offset_us / 1_000_000.0
        integer_cycles = round(expected_offset_cycles - phase_cycles)
        measured_offset_cycles = integer_cycles + phase_cycles
        offset_clock_cycles = offset_us * 50
        tuning_word = int(round(measured_offset_cycles * (2**32) / offset_clock_cycles))
        nominal_word = int(round(grid_frequency * (2**32) / 50_000_000.0))
        if tuning_word <= 0 or abs(tuning_word - nominal_word) > nominal_word * 0.002:
            tuning_word = nominal_word

        confidence = min(fit_a.confidence, fit_b.confidence)
        confidence *= max(0.0, 1.0 - candidate_error(grid_frequency) * 1.5)
        return DualProbeFit(grid_frequency, tuning_word, phase_cycles,
                            confidence, fit_a, fit_b)


class TargetAnalyzer:
    def __init__(self, config: dict[str, Any]) -> None:
        vision = config.get("vision", {})
        self._phase_step = max(1, int(vision.get("phase_search_step", 4)))

    @staticmethod
    def _model_points(shape: int, phase: int, center_x: float, center_y: float,
                      amplitude_x: float, amplitude_y: float) -> np.ndarray:
        parameter = np.linspace(0.0, 2.0 * pi, 1400, endpoint=False)
        ratio = 2 if shape == 3 else 1
        phase_radians = phase * 2.0 * pi / 256.0
        x = center_x + amplitude_x * np.sin(parameter)
        y = center_y - amplitude_y * np.sin(ratio * parameter + phase_radians)
        return np.column_stack((x, y)).round().astype(np.int32)

    @staticmethod
    def _chamfer(distance: np.ndarray, points: np.ndarray) -> float:
        height, width = distance.shape
        valid = ((points[:, 0] >= 0) & (points[:, 0] < width) &
                 (points[:, 1] >= 0) & (points[:, 1] < height))
        points = points[valid]
        if not points.size:
            return 1.0
        return float(np.mean(distance[points[:, 1], points[:, 0]]) / min(width, height))

    def analyze(self, mask: np.ndarray, shape: int) -> TargetFit:
        rows, columns = np.nonzero(mask)
        if rows.size < 100:
            raise ValueError("not enough target trace pixels")
        x_low, x_high = np.percentile(columns, [1.0, 99.0])
        y_low, y_high = np.percentile(rows, [1.0, 99.0])
        center_x = (x_low + x_high) * 0.5
        center_y = (y_low + y_high) * 0.5
        amplitude_x = max(4.0, (x_high - x_low) * 0.5)
        amplitude_y = max(4.0, (y_high - y_low) * 0.5)

        inverse = cv2.bitwise_not(mask)
        distance = cv2.distanceTransform(inverse, cv2.DIST_L2, 3)
        phase_scores: list[tuple[float, int]] = []
        for phase in range(0, 256, self._phase_step):
            points = self._model_points(shape, phase, center_x, center_y,
                                        amplitude_x, amplitude_y)
            phase_scores.append((self._chamfer(distance, points), phase))
        best_score, best_phase = min(phase_scores)

        desired_phases = (64, 192) if shape == 2 else (0, 128)
        desired_score = min(
            self._chamfer(distance, self._model_points(
                shape, phase, center_x, center_y, amplitude_x, amplitude_y))
            for phase in desired_phases
        )
        height, width = mask.shape
        span_x_div = (x_high - x_low) / (width / 10.0)
        span_y_div = (y_high - y_low) / (height / 8.0)
        center_error = (((center_x - width * 0.5) / (width / 10.0)) ** 2 +
                        ((center_y - height * 0.5) / (height / 8.0)) ** 2) ** 0.5
        shape_quality = max(0.0, 1.0 - desired_score / 0.055)
        amplitude_quality = max(0.0, 1.0 - abs(span_y_div - 8.0) / 4.0)
        quality = int(round(100.0 * (0.8 * shape_quality + 0.2 * amplitude_quality)))
        return TargetFit(best_phase, desired_score, max(0, min(100, quality)),
                         float(span_x_div), float(span_y_div), float(center_error))


def aggregate_masks(masks: list[np.ndarray]) -> np.ndarray:
    if not masks:
        raise ValueError("at least one mask is required")
    stack = np.stack([(mask > 0).astype(np.uint8) for mask in masks], axis=0)
    required = max(1, (len(masks) + 1) // 2)
    return ((np.sum(stack, axis=0) >= required) * 255).astype(np.uint8)
