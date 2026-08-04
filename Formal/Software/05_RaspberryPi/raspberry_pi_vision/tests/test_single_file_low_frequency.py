import copy

import cv2
import numpy as np
import time

import task5_cv_single as single


class FakeLiveCapture:
    def __init__(self, _source: object) -> None:
        self.counter = 0
        self.released = False

    def isOpened(self) -> bool:
        return True

    def set(self, _property: int, _value: object) -> bool:
        return True

    def read(self) -> tuple[bool, np.ndarray | None]:
        time.sleep(0.005)
        if self.released:
            return False, None
        self.counter += 1
        return True, np.full((4, 4, 3), self.counter, np.uint8)

    def release(self) -> None:
        self.released = True


def measurement(accepted: bool, frequency_hz: float, reason: str,
                points: int = 10) -> single.CoarseMeasurement:
    return single.CoarseMeasurement(
        accepted=accepted,
        frequency_hz=frequency_hz,
        period_cv=0.01 if accepted else 1.0,
        valid_frame_ratio=1.0 if accepted else 0.0,
        median_point_count=points,
        complete_period_count=8 if accepted else 0,
        confidence=0.9 if accepted else 0.0,
        reason=reason,
    )


def controller_at_final_w2() -> single.AutoLissajousController:
    controller = single.AutoLissajousController({}, object(), object())
    controller._coarse_width_codes = (0, 1, 2)
    controller._coarse_index = 2
    controller._coarse_width_code = 2
    controller._best_coarse_index = 0
    controller._best_coarse_width_code = 0
    controller._best_coarse_summary_width_us = 100.0
    controller._best_coarse_frequency_hz = 45_000.0
    controller._best_coarse_quality = 90
    controller._best_coarse_points = 8
    return controller


def wave_point(x_normalized: float, y_px: float) -> single.WavePoint:
    return single.WavePoint(
        x_px=100.0 if x_normalized < 0.0 else 540.0,
        y_px=y_px,
        x_normalized=x_normalized,
        y_normalized=0.0,
        y_volts=0.0,
        time_normalized=y_px / single.FREQUENCY_RAMP_HEIGHT_PX,
        strength=0.9,
    )


def test_live_camera_reader_delivers_latest_post_settle_frame(monkeypatch) -> None:
    monkeypatch.setattr(single.cv2, "VideoCapture", FakeLiveCapture)
    camera = single.ScopeCamera({
        "camera": {"device": 0, "read_timeout_s": 0.3},
        "vision": {"canonical_size": [4, 4]},
    })
    try:
        time.sleep(0.04)
        first = int(camera.read_raw()[0, 0, 0])
        time.sleep(0.04)
        second = int(camera.read_raw()[0, 0, 0])
        assert second - first >= 3

        settle_ended = time.monotonic()
        camera.require_frame_after(settle_ended)
        third = int(camera.read_raw()[0, 0, 0])
        assert third > second
        assert camera._latest_frame_time >= settle_ended
    finally:
        camera.close()


def test_raw_turning_band_count_detects_dense_w2_trace() -> None:
    sparse = np.zeros((512, 640), np.uint8)
    dense = np.zeros_like(sparse)
    for index, row in enumerate(range(60, 460, 80)):
        cv2.rectangle(sparse, (80, row), (110, row + 5), 255, -1)
        cv2.rectangle(sparse, (530, row), (560, row + 5), 255, -1)
    for row in range(50, 470, 35):
        cv2.rectangle(dense, (80, row), (110, row + 5), 255, -1)
        cv2.rectangle(dense, (530, row), (560, row + 5), 255, -1)

    assert single.count_raw_turning_bands(sparse) == 10
    assert single.count_raw_turning_bands(dense) == 24
    assert single.count_raw_turning_bands(sparse) <= single.W2_MAX_RAW_TURNS
    assert single.count_raw_turning_bands(dense) > single.W2_MAX_RAW_TURNS


def test_fragment_chain_is_merged_as_one_thick_turning_band() -> None:
    merged = single.merge_nearby_y_candidates([
        (100.0, 50.0, 10.0),
        (101.0, 57.0, 8.0),
        (99.0, 64.0, 9.0),
        (100.0, 110.0, 10.0),
    ], maximum_gap=8.0)

    assert len(merged) == 2
    assert 56.0 < merged[0][1] < 58.0


def test_dense_opposite_side_points_are_not_cross_side_deduplicated() -> None:
    candidates = []
    for row in range(40, 241, 40):
        candidates.append((100.0, float(row), 20.0))
        candidates.append((540.0, float(row + 3), 19.0))

    global_nms = single.deduplicate_turning_candidates(
        candidates, 100.0, 540.0, 11.0, same_side_only=False)
    same_side_nms = single.deduplicate_turning_candidates(
        candidates, 100.0, 540.0, 11.0, same_side_only=True)
    global_sequence = single.select_alternating_edge_points(
        global_nms, 100.0, 540.0)
    dense_sequence = single.select_alternating_edge_points(
        same_side_nms, 100.0, 540.0)

    assert len(global_sequence) <= 2
    assert len(dense_sequence) == 12
    assert single.dense_candidate_period_is_consistent(
        dense_sequence, 100.0, 540.0)


def synthetic_period_profile(
        period: float, phase: float, dropped_index: int) -> np.ndarray:
    coordinate = np.arange(512, dtype=np.float64)
    profile = np.full(512, 0.03, np.float64)
    index = 0
    position = 110.0 + phase
    while position < 420.0:
        amplitude = 0.42 + 0.05 * ((index * 7 + 3) % 11)
        if index != dropped_index:
            profile += amplitude * np.exp(
                -0.5 * ((coordinate - position) / 2.0) ** 2)
        position += period
        index += 1
    # A strong isolated reflection must not win over a shared periodic trace.
    profile += 1.4 * np.exp(
        -0.5 * ((coordinate - (245.0 + 0.3 * phase)) / 3.0) ** 2)
    profile += 0.015 * np.sin(0.37 * coordinate + phase)
    return profile.astype(np.float32)


def test_shared_profile_period_recovers_dense_high_frequency_spacing() -> None:
    for expected_period in (90.0, 80.0, 59.0, 40.0, 26.0, 21.0, 18.0):
        left = synthetic_period_profile(expected_period, 0.0, 2)
        right = synthetic_period_profile(
            expected_period, 0.47 * expected_period, 4)

        period, confidence = single.estimate_shared_profile_period(
            left, right, 100, 407)
        left_peaks, left_coverage = single.fit_profile_comb_peaks(
            left, 100, 407, period)
        right_peaks, right_coverage = single.fit_profile_comb_peaks(
            right, 100, 407, period)

        assert abs(period / expected_period - 1.0) < 0.04
        assert confidence >= 0.22
        # At 80-90 px only three complete same-side turns fit in the usable
        # vertical ROI; that is still enough for the downstream two-side check.
        assert min(len(left_peaks), len(right_peaks)) >= 3
        assert min(left_coverage, right_coverage) >= 0.55


def test_sparse_236px_profile_is_not_forced_into_dense_recovery() -> None:
    coordinate = np.arange(512, dtype=np.float64)

    def sparse_profile(phase: float) -> np.ndarray:
        profile = np.zeros(512, np.float64)
        for position in (130.0 + phase, 366.0 + phase):
            profile += np.exp(
                -0.5 * ((coordinate - position) / 2.0) ** 2)
        return profile.astype(np.float32)

    period, confidence = single.estimate_shared_profile_period(
        sparse_profile(0.0), sparse_profile(70.0), 100, 407)

    assert period == 0.0
    assert confidence == 0.0


def test_fixed_camera_energy_fallback_locates_short_edge_bands() -> None:
    image = np.zeros((512, 640, 3), np.uint8)
    for row in range(150, 430, 45):
        cv2.rectangle(image, (145, row), (175, row + 4), (0, 220, 0), -1)
    for row in range(170, 420, 45):
        cv2.rectangle(image, (555, row), (585, row + 4), (0, 220, 0), -1)

    left_x, right_x, _, _ = single.estimate_waveform_edges(
        image, 132, 439)

    assert 140.0 <= left_x <= 180.0
    assert 550.0 <= right_x <= 590.0


def test_fixed_camera_scale_corrects_two_recorded_w0_high_frequency_runs() -> None:
    for old_period, actual_frequency in ((0.2223, 57_000.0),
                                         (0.2813, 45_000.0)):
        pixel_period = old_period * 469.05
        points = []
        for side, offset in ((-1.0, 20.0), (1.0, 35.0)):
            points.extend([
                wave_point(side, offset),
                wave_point(side, offset + pixel_period),
                wave_point(side, offset + 2.0 * pixel_period),
            ])
        _, _, count, frequency = single.compute_robust_phase_interval(
            points, ramp_duration_us=100.0)
        assert count == 4
        assert abs(frequency - actual_frequency) / actual_frequency < 0.002


def test_three_khz_w2_summary_uses_real_2ms_period() -> None:
    observations = [
        single.CoarseFrameObservation(
            point_count=10,
            left_periods=(0.1660, 0.1668, 0.1671, 0.1665),
            right_periods=(0.1664, 0.1669, 0.1666, 0.1670),
            confidence=0.9,
            raw_turn_count=10,
        )
        for _ in range(10)
    ]
    result = single.summarize_coarse_observations(
        observations,
        width_us=2000.0,
        maximum_points=single.W2_MAX_RAW_TURNS,
        period_mode="fundamental",
    )

    assert result.accepted
    assert abs(result.frequency_hz - 3000.0) < 20.0


def test_two_same_side_periods_per_frame_are_fused_across_frames() -> None:
    observations = [
        single.CoarseFrameObservation(
            point_count=5,
            left_periods=(0.1020,),
            # The supplied 20 kHz image also contains a missed-point 3x gap.
            right_periods=(0.1030, 0.3100),
            confidence=0.85,
            raw_turn_count=10,
        )
        for _ in range(3)
    ]

    result = single.summarize_coarse_observations(
        observations,
        width_us=500.0,
        minimum_periods=3,
    )

    assert result.accepted
    assert result.complete_period_count == 6
    assert 19_000.0 < result.frequency_hz < 20_500.0


def test_joint_side_filter_rejects_singleton_three_x_missed_period() -> None:
    left, right = single.reject_integer_multiple_periods_by_side(
        (60.0 / 594.0,),
        (62.0 / 594.0, 186.0 / 594.0),
    )

    assert len(left) == 1
    assert len(right) == 1
    assert abs(left[0] * 594.0 - 60.0) < 0.01
    assert abs(right[0] * 594.0 - 62.0) < 0.01


def test_one_sparse_frame_does_not_satisfy_three_period_total() -> None:
    result = single.summarize_coarse_observations([
        single.CoarseFrameObservation(
            point_count=5,
            left_periods=(0.1020,),
            right_periods=(0.1030,),
            confidence=0.85,
            raw_turn_count=5,
        )
    ], width_us=500.0, minimum_periods=3)

    assert not result.accepted
    assert result.reason == "TOO_FEW_PERIODS"


def test_w2_alias_does_not_override_earlier_reliable_result() -> None:
    controller = controller_at_final_w2()
    controller._coarse_summary = lambda: measurement(True, 3_000.0, "OK")

    controller._finish_coarse(1.0)

    assert controller._coarse_width_code == 0
    assert controller._coarse_frequency_hz == 45_000.0


def test_w2_is_used_when_both_shorter_widths_have_no_reliable_result() -> None:
    controller = controller_at_final_w2()
    controller._best_coarse_index = -1
    controller._best_coarse_frequency_hz = 0.0
    controller._coarse_summary = lambda: measurement(True, 3_000.0, "OK")

    controller._finish_coarse(1.0)

    assert controller._coarse_width_code == 2
    assert controller._coarse_frequency_hz == 3_000.0


def test_dense_w2_may_fallback_to_earlier_high_frequency_result() -> None:
    controller = controller_at_final_w2()
    controller._coarse_summary = lambda: measurement(
        False, 0.0, "VISUAL_RANGE_HIGH", points=24)

    controller._finish_coarse(1.0)

    assert controller._coarse_width_code == 0
    assert controller._coarse_frequency_hz == 45_000.0


def test_w2_period_mismatch_keeps_an_earlier_reliable_frequency() -> None:
    controller = controller_at_final_w2()
    controller._coarse_summary = lambda: measurement(
        False, 0.0, "POINT_FREQ_MISMATCH", points=10)

    controller._finish_coarse(1.0)

    assert controller._coarse_width_code == 0
    assert controller._coarse_frequency_hz == 45_000.0


def test_all_calculation_widths_equal_the_fpga_command_widths() -> None:
    controller = single.AutoLissajousController(single.DEFAULT_CONFIG,
                                                object(), object())
    assert [controller._coarse_calculation_width_us(code)
            for code in (0, 1, 2)] == [100.0, 500.0, 2000.0]


def test_recorded_20khz_period_maps_to_w1_not_w2() -> None:
    pixel_period = 0.1255 * 469.05
    points = []
    for side, offset in ((-1.0, 20.0), (1.0, 35.0)):
        points.extend([
            wave_point(side, offset),
            wave_point(side, offset + pixel_period),
            wave_point(side, offset + 2.0 * pixel_period),
        ])
    _, _, _, w1_frequency = single.compute_robust_phase_interval(points, 500.0)
    _, _, _, w2_frequency = single.compute_robust_phase_interval(points, 2000.0)

    assert abs(w1_frequency - 20_000.0) / 20_000.0 < 0.02
    assert 4_500.0 < w2_frequency < 5_500.0


def test_left_and_right_periods_must_agree() -> None:
    observations = [
        single.CoarseFrameObservation(
            point_count=8,
            left_periods=(0.10, 0.101, 0.099),
            right_periods=(0.16, 0.161, 0.159),
            confidence=0.9,
            raw_turn_count=8,
        )
        for _ in range(10)
    ]
    result = single.summarize_coarse_observations(
        observations, width_us=500.0,
        maximum_side_period_difference=0.20,
    )

    assert not result.accepted
    assert result.reason == "SIDE_PERIOD_MISMATCH"


def test_inconsistent_w1_candidate_does_not_overwrite_valid_w0_high_result() -> None:
    controller = single.AutoLissajousController({}, object(), object())
    controller._coarse_width_codes = (0, 1)
    controller._coarse_index = 1
    controller._coarse_width_code = 1
    controller._best_coarse_index = 0
    controller._best_coarse_width_code = 0
    controller._best_coarse_summary_width_us = 100.0
    controller._best_coarse_frequency_hz = 45_000.0
    controller._best_coarse_quality = 85
    controller._best_coarse_points = 6
    controller._coarse_summary = lambda: measurement(
        True, 12_600.0, "OK", points=8)

    controller._finish_coarse(1.0)

    assert controller._coarse_width_code == 0
    assert controller._coarse_frequency_hz == 45_000.0


class RecordingLink:
    def __init__(self) -> None:
        self.frames: list[single.Frame] = []

    def send_frame(self, command: int, payload: bytes, *,
                   flags: int = 0) -> single.Frame:
        frame = single.Frame(len(self.frames) & 0xFF, command, payload, flags)
        self.frames.append(frame)
        return frame


def run_three_width_scan(
        summaries: list[single.CoarseMeasurement],
        config: dict | None = None,
) -> tuple[single.AutoLissajousController, RecordingLink]:
    link = RecordingLink()
    controller = single.AutoLissajousController(config or {}, link, object())
    pending = iter(summaries)

    def next_summary() -> single.CoarseMeasurement:
        controller._coarse_summary_width_us = (
            controller._coarse_calculation_width_us(
                controller._coarse_width_code))
        return next(pending)

    controller._coarse_summary = next_summary
    assert controller.start(single.TARGET_DIAGONAL, 0.0)
    controller._finish_coarse(1.0)
    controller._finish_coarse(2.0)
    controller._finish_coarse(3.0)
    return controller, link


def test_high_frequency_consensus_skips_only_the_2ms_diagnostic() -> None:
    config = copy.deepcopy(single.DEFAULT_CONFIG)
    link = RecordingLink()
    controller = single.AutoLissajousController(config, link, object())
    pending = iter((
        measurement(True, 44_000.0, "OK", points=5),
        measurement(True, 43_700.0, "OK", points=10),
    ))

    def next_summary() -> single.CoarseMeasurement:
        controller._coarse_summary_width_us = (
            controller._coarse_calculation_width_us(
                controller._coarse_width_code))
        return next(pending)

    controller._coarse_summary = next_summary
    assert controller.start(single.TARGET_DIAGONAL, 0.0)
    controller._finish_coarse(1.0)
    controller._finish_coarse(2.0)

    assert [frame.payload[0] for frame in link.frames
            if frame.command == single.CMD_PROBE_SINGLE] == [0, 1]
    assert link.frames[-1].command == single.CMD_TARGET
    assert link.frames[-1].payload[0] == single.TARGET_DIAGONAL
    assert controller.mode == "CIRCLE_SWEEP_SETTLE"
    assert controller._coarse_width_code == 1
    assert controller._coarse_frequency_hz == 43_700.0


def test_low_frequency_match_still_advances_to_2ms() -> None:
    config = copy.deepcopy(single.DEFAULT_CONFIG)
    link = RecordingLink()
    controller = single.AutoLissajousController(config, link, object())
    pending = iter((
        measurement(False, 0.0, "TOO_FEW_POINTS", points=3),
        measurement(True, 3_000.0, "OK", points=10),
    ))
    controller._coarse_summary = lambda: next(pending)

    assert controller.start(single.TARGET_DIAGONAL, 0.0)
    controller._finish_coarse(1.0)
    controller._finish_coarse(2.0)

    assert [frame.payload[0] for frame in link.frames
            if frame.command == single.CMD_PROBE_SINGLE] == [0, 1, 2]
    assert controller._coarse_width_code == 2


def test_full_scan_uses_w1_w2_consensus_to_reject_false_w0_high() -> None:
    controller, link = run_three_width_scan([
        measurement(True, 66_000.0, "OK", points=6),
        measurement(True, 3_050.0, "OK", points=6),
        measurement(True, 3_000.0, "OK", points=10),
    ])

    assert [frame.payload[0] for frame in link.frames
            if frame.command == single.CMD_PROBE_SINGLE] == [0, 1, 2]
    assert link.frames[-1].command == single.CMD_TARGET
    assert controller.mode == "CIRCLE_SWEEP_SETTLE"
    assert controller._coarse_width_code == 2
    assert controller._coarse_frequency_hz == 3_000.0


def test_full_scan_sparse_w1_allows_low_w2_to_reject_false_w0() -> None:
    controller, _ = run_three_width_scan([
        measurement(True, 66_000.0, "OK", points=5),
        measurement(False, 0.0, "TOO_FEW_POINTS", points=3),
        measurement(True, 3_000.0, "OK", points=10),
    ])

    assert controller._coarse_width_code == 2
    assert controller._coarse_frequency_hz == 3_000.0


def test_full_scan_dense_w1_keeps_high_w0_over_w2_alias() -> None:
    controller, _ = run_three_width_scan([
        measurement(True, 45_000.0, "OK", points=6),
        measurement(False, 0.0, "VISUAL_RANGE_HIGH", points=24),
        measurement(True, 4_200.0, "OK", points=9),
    ])

    assert controller._coarse_width_code == 0
    assert controller._coarse_frequency_hz == 45_000.0


def test_second_start_clears_all_three_width_candidates() -> None:
    controller, _ = run_three_width_scan([
        measurement(True, 45_000.0, "OK", points=6),
        measurement(False, 0.0, "VISUAL_RANGE_HIGH", points=24),
        measurement(False, 0.0, "VISUAL_RANGE_HIGH", points=24),
    ])
    assert controller._coarse_candidates

    assert controller.start(single.TARGET_CIRCLE, 4.0)

    assert controller._coarse_candidates == []
    assert controller._coarse_stage_measurements == {}
    assert controller._best_coarse_frequency_hz == 0.0
