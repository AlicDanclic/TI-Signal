from math import pi

import cv2
import numpy as np

from vision import (
    CoarseFrameObservation,
    FrequencyEstimator,
    TargetAnalyzer,
    reject_integer_multiple_periods,
    resolve_dual_interval_frequency,
    summarize_coarse_observations,
    wrap_cycles,
)


def draw_probe(mask: np.ndarray, top: float, bottom: float, cycles: float,
               phase: float) -> None:
    height, width = mask.shape
    y_top = int(round(top * height))
    y_bottom = int(round(bottom * height))
    span = y_bottom - y_top - 1
    previous = None
    for row in range(y_top, y_bottom):
        q = (y_bottom - 1 - row) / span
        column = int(round(width * 0.5 + width * 0.38 *
                           np.sin(2.0 * pi * cycles * q + phase)))
        if previous is not None:
            cv2.line(mask, previous, (column, row), 255, 2)
        previous = (column, row)


def test_single_probe_cycle_fit() -> None:
    mask = np.zeros((480, 640), np.uint8)
    draw_probe(mask, 0.05, 0.95, 4.25, 0.37)
    fit = FrequencyEstimator().estimate_single(mask, 1)
    assert abs(fit.cycles - 4.25) < 0.02
    assert fit.confidence > 0.8


def test_dual_probe_selects_100_hz_grid_and_tuning_word() -> None:
    frequency = 43700
    width_us = 100
    offset_us = 7000
    cycles = frequency * width_us / 1_000_000.0
    phase_a = 0.41
    phase_b = phase_a + 2.0 * pi * frequency * offset_us / 1_000_000.0
    mask = np.zeros((480, 640), np.uint8)
    draw_probe(mask, 0.55, 0.95, cycles, phase_a)
    draw_probe(mask, 0.05, 0.45, cycles, phase_b)

    result = FrequencyEstimator().estimate_dual(
        mask, 0, coarse_frequency_hz=43680.0, offset_us=offset_us)
    expected_word = round(frequency * (2**32) / 50_000_000.0)
    assert result.grid_frequency_hz == frequency
    assert abs(result.tuning_word - expected_word) < 100
    assert result.confidence > 0.6


def test_target_analyzer_accepts_circle_and_eight() -> None:
    analyzer = TargetAnalyzer({"vision": {"phase_search_step": 4}})
    for shape, phase in ((2, 64), (3, 0)):
        mask = np.zeros((480, 640), np.uint8)
        points = analyzer._model_points(shape, phase, 320, 240, 250, 210)
        for first, second in zip(points[:-1], points[1:]):
            cv2.line(mask, tuple(first), tuple(second), 255, 3)
        fit = analyzer.analyze(mask, shape)
        assert fit.quality >= 85
        assert abs(fit.span_y_div - 7.0) < 0.2


def test_integer_multiple_missed_points_do_not_shift_period() -> None:
    filtered = reject_integer_multiple_periods(
        [0.100, 0.101, 0.099, 0.200, 0.301])
    assert len(filtered) == 3
    assert abs(float(np.median(filtered)) - 0.100) < 0.002


def test_one_second_coarse_summary_uses_frame_medians() -> None:
    observations = [
        CoarseFrameObservation(
            6,
            (0.200, 0.400),  # 0.400 is a missed-point 2x interval.
            (0.199, 0.201),
            0.9,
        )
        for _ in range(10)
    ]
    result = summarize_coarse_observations(observations, width_us=500.0)
    assert result.accepted
    assert abs(result.frequency_hz - 10_000.0) < 30.0
    assert result.median_point_count == 6
    assert result.valid_frame_ratio == 1.0


def test_three_and_seven_ms_joint_phase_disambiguation() -> None:
    actual_frequency = 43_700.032
    phase_3 = wrap_cycles(actual_frequency * 0.003)
    phase_7 = wrap_cycles(actual_frequency * 0.007)
    result = resolve_dual_interval_frequency(
        43_680.0, phase_3, phase_7, coarse_uncertainty_hz=450.0)
    assert abs(result.frequency_hz - actual_frequency) < 1e-6
    assert abs(result.correction_hz - 20.032) < 1e-6
