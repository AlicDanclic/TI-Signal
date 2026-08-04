import copy

import cv2
import numpy as np
import pytest

import task5_cv_single as single


class RecordingLink:
    def __init__(self) -> None:
        self.frames: list[single.Frame] = []

    def send_frame(self, command: int, payload: bytes, *,
                   flags: int = 0) -> single.Frame:
        frame = single.Frame(len(self.frames) & 0xFF, command, payload, flags)
        self.frames.append(frame)
        return frame


class FreshFrameCamera:
    def __init__(self) -> None:
        self.required_after: list[float] = []

    def require_frame_after(self, timestamp: float) -> None:
        self.required_after.append(timestamp)


def controller_config() -> dict:
    return copy.deepcopy(single.DEFAULT_CONFIG)


def circle_fit(score: float = 0.72) -> single.CircleLockFit:
    return single.CircleLockFit(
        quality=int(round(score * 100.0)),
        score=score,
        span_x_div=7.8,
        span_y_div=5.2,
        center_error_div=0.1,
        radial_cv=0.08,
        inner_fill_ratio=0.05,
        angular_coverage=0.82,
        fill_ratio=0.16,
        pixel_count=1200,
    )


def locked_circle_fit(score: float = 0.90) -> single.CircleLockFit:
    return single.CircleLockFit(
        quality=int(round(score * 100.0)),
        score=score,
        span_x_div=7.8,
        span_y_div=7.8,
        center_error_div=0.1,
        radial_cv=0.06,
        inner_fill_ratio=0.05,
        angular_coverage=0.90,
        fill_ratio=0.16,
        pixel_count=1200,
        ellipse_axis_ratio=1.02,
        ellipse_angle_degrees=0.0,
    )


def trace_fit(
    score: float = 0.90,
    *,
    span_x_div: float = 7.8,
    span_y_div: float = 5.2,
) -> single.FrequencyTraceFit:
    return single.FrequencyTraceFit(
        quality=int(round(score * 100.0)),
        score=score,
        thinness_quality=0.90,
        temporal_overlap=0.90,
        extent_quality=1.0,
        span_x_div=span_x_div,
        span_y_div=span_y_div,
        thickness_px=3.0,
        pixel_count=900,
        valid_frames=5,
        aggregate_pixel_count=900,
        total_frames=5,
    )


def sweep_result(
    frequency_hz: float,
    *,
    shape_score: float = 0.72,
    trace_score: float = 0.90,
    span_x_div: float = 7.8,
    span_y_div: float = 5.2,
) -> single.CircleSweepResult:
    return single.CircleSweepResult(
        frequency_hz=frequency_hz,
        tuning_word=single.dds_tuning_word_for_frequency(frequency_hz),
        amplitude=103,
        phase=64,
        fit=circle_fit(shape_score),
        trace_fit=trace_fit(
            trace_score,
            span_x_div=span_x_div,
            span_y_div=span_y_div,
        ),
    )


def test_two_axis_span_outranks_one_axis_reflection() -> None:
    config = controller_config()
    sweep = config["target"]["circle_sweep"]
    sweep["trace_minimum_pixels"] = 20
    sweep["trace_minimum_frames"] = 3

    ellipse = np.zeros((512, 640), np.uint8)
    reflection = np.zeros_like(ellipse)
    cv2.ellipse(
        ellipse, (320, 256), (230, 155), 0, 0, 360,
        255, 3, cv2.LINE_AA,
    )
    cv2.line(
        reflection, (45, 256), (595, 256),
        255, 3, cv2.LINE_AA,
    )

    ellipse_trace = single.analyze_frequency_trace_masks(
        [ellipse, ellipse, ellipse], config)
    reflection_trace = single.analyze_frequency_trace_masks(
        [reflection, reflection, reflection], config)

    ellipse_result = single.CircleSweepResult(
        50_000.0, 1, 103, 64, circle_fit(), ellipse_trace)
    reflection_result = single.CircleSweepResult(
        50_100.0, 2, 103, 64, circle_fit(), reflection_trace)

    assert ellipse_trace.extent_quality > reflection_trace.extent_quality
    assert ellipse_trace.score > reflection_trace.score
    assert single.circle_sweep_result_is_ellipse_seed(ellipse_result, config)
    assert not single.circle_sweep_result_is_ellipse_seed(
        reflection_result, config)


def test_rotated_ellipse_is_servo_seed_but_not_a_locked_circle() -> None:
    config = controller_config()
    mask = np.zeros((512, 640), np.uint8)
    points = single.TargetAnalyzer._model_points(
        single.TARGET_CIRCLE, 32, 320.0, 256.0, 230.0, 230.0)
    cv2.polylines(mask, [points], True, 255, 3, cv2.LINE_AA)

    shape = single.analyze_circle_lock_mask(mask, config)
    trace = single.analyze_frequency_trace_masks([mask, mask, mask], config)
    result = single.CircleSweepResult(
        50_000.0, 1, 103, 64, shape, trace)

    assert shape.ellipse_axis_ratio > config["target"]["circle_lock"][
        "maximum_axis_ratio"]
    assert single.circle_sweep_result_is_ellipse_seed(
        result, config, strong=True)
    assert not single.circle_fit_is_locked(shape, config)


def test_equal_axis_quadrature_trace_is_a_locked_circle() -> None:
    config = controller_config()
    mask = np.zeros((512, 640), np.uint8)
    points = single.TargetAnalyzer._model_points(
        single.TARGET_CIRCLE, 64, 320.0, 256.0, 230.0, 230.0)
    cv2.polylines(mask, [points], True, 255, 3, cv2.LINE_AA)

    fit = single.analyze_circle_lock_mask(mask, config)

    assert fit.ellipse_axis_ratio == pytest.approx(1.0, abs=0.03)
    assert single.circle_fit_is_locked(fit, config)


@pytest.mark.parametrize("coarse_hz", (85_000.0, 87_000.0))
def test_high_biased_tiers_reach_90khz_before_negative_side(
        coarse_hz: float) -> None:
    radii = [500.0, 1500.0, 3500.0, 5000.0]
    tiers = single.circle_sweep_biased_frequency_tiers(
        coarse_hz,
        radii,
        step_hz=100.0,
        minimum_hz=1_000.0,
        maximum_hz=100_000.0,
    )
    positive_tiers = tiers[:len(radii)]
    negative_tiers = tiers[len(radii):]

    assert any(90_000.0 in tier for tier in positive_tiers)
    assert all(value >= coarse_hz for tier in positive_tiers for value in tier)
    assert all(value < coarse_hz for tier in negative_tiers for value in tier)
    flattened = [value for tier in tiers for value in tier]
    assert len(flattened) == len(set(flattened))
    assert all(value % 100.0 == 0.0 for value in flattened)


def test_28khz_coarse_result_uses_2500hz_high_range_insurance() -> None:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._coarse_frequency_hz = 28_000.0

    controller._start_circle_sweep(1.0)

    assert any(
        30_500.0 in tier for tier in controller._circle_sweep_tiers)
    assert controller._circle_screen_step_hz == 100.0


def test_high_frequency_100hz_screen_seed_enters_repeated_validation() -> None:
    config = controller_config()
    # This test isolates validation; R17's separate field regression verifies
    # the mandatory +2.5 kHz high-frequency screening range.
    config["target"]["circle_sweep"][
        "high_frequency_required_positive_search_hz"] = 0.0
    config["target"]["circle_sweep"]["reject_boundary_best"] = False
    controller = single.AutoLissajousController(
        config, RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._coarse_frequency_hz = 50_000.0
    controller._start_circle_sweep(1.0)
    controller._circle_sweep_results = [
        sweep_result(50_000.0),
        sweep_result(50_100.0, trace_score=0.60),
    ]

    controller._finish_circle_screen_tier(2.0)

    assert controller._circle_sweep_stage == "VERIFY"
    assert controller._circle_validation_anchor_hz == 50_000.0
    assert controller.mode == "CIRCLE_SWEEP_SETTLE"
    assert controller._circle_sweep_frequencies[0] == 50_000.0
    assert {49_900.0, 50_000.0, 50_100.0}.issubset(
        set(controller._circle_sweep_frequencies))


def test_micro_strong_ellipse_enters_hold() -> None:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._coarse_frequency_hz = 50_000.0
    controller._start_circle_sweep(1.0)
    controller._circle_sweep_stage = "MICRO"
    controller._circle_micro_seed_hz = 50_000.0
    controller._circle_micro_results = [
        sweep_result(50_000.0, trace_score=0.72),
        sweep_result(50_000.5, trace_score=0.91),
    ]

    controller._finish_circle_micro_scan(2.0)

    assert controller._circle_sweep_stage == "HOLD"
    assert controller._circle_sweep_frequencies == [50_000.5]
    assert controller.mode == "CIRCLE_SWEEP_SETTLE"


def test_failed_hold_tries_next_micro_candidate() -> None:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._coarse_frequency_hz = 50_000.0
    controller._circle_micro_seed_hz = 50_000.0
    best = sweep_result(50_000.5, trace_score=0.95)
    fallback = sweep_result(49_999.5, trace_score=0.82)
    controller._circle_micro_results = [best, fallback]
    controller._circle_sweep_frequencies = [best.frequency_hz]
    controller._circle_hold_result = sweep_result(
        best.frequency_hz, trace_score=0.10)

    controller._finish_circle_hold(2.0)

    assert best.frequency_hz in controller._circle_rejected_frequencies
    assert controller._circle_sweep_stage == "HOLD"
    assert controller._circle_sweep_frequencies == [fallback.frequency_hz]


def test_micro_scan_does_not_reselect_rejected_frequency() -> None:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._coarse_frequency_hz = 50_000.0
    controller._circle_micro_seed_hz = 50_000.0
    rejected = sweep_result(50_000.5, trace_score=0.95)
    fallback = sweep_result(49_999.5, trace_score=0.82)
    controller._circle_micro_results = [rejected, fallback]
    controller._circle_rejected_frequencies.add(rejected.frequency_hz)

    controller._finish_circle_micro_scan(2.0)

    assert controller._circle_sweep_frequencies == [fallback.frequency_hz]


def test_circle_lock_enters_continuous_maintenance() -> None:
    config = controller_config()
    controller = single.AutoLissajousController(
        config, RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._final_frequency_hz = 50_000.5

    controller._enter_locked(10.0, 82)

    interval = config["target"]["circle_lock"]["maintenance_interval_s"]
    assert controller._circle_locked_announced is True
    assert controller.mode == "CIRCLE_MAINTAIN_SETTLE"
    assert controller._deadline == pytest.approx(
        10.0 + max(0.10, interval))
    assert controller.active is True


def test_improving_phase_a_trial_does_not_integrate_its_own_command(
        monkeypatch) -> None:
    config = controller_config()
    config["target"]["circle_lock"][
        "frequency_integral_required_steps"] = 1
    controller = single.AutoLissajousController(
        config, RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._phase = 64
    controller._amplitude = 103
    controller._coarse_frequency_hz = 49_900.0
    controller._final_frequency_hz = 50_000.0
    controller._tuning_word = single.dds_tuning_word_for_frequency(50_000.0)
    controller._circle_locked_announced = True
    controller._circle_last_servo_at = 1.0
    mask = np.ones((32, 32), np.uint8)
    sent: list[tuple[int, int]] = []
    monkeypatch.setattr(
        controller.target_analyzer,
        "analyze",
        lambda _mask, _target: single.TargetFit(
            0, 0.2, 50, 7.8, 5.2, 0.0),
    )
    monkeypatch.setattr(
        controller,
        "_send_circle_confirm_target",
        lambda _now: sent.append((controller._phase, controller._tuning_word)),
    )

    baseline = circle_fit(0.45)
    improved_a = circle_fit(0.80)
    original_frequency = controller._final_frequency_hz
    original_word = controller._tuning_word

    assert controller._try_adjust_circle_target(mask, baseline, 1.0)
    phase_step = ((controller._phase - 64 + 128) & 0xFF) - 128
    assert 0 < abs(phase_step) <= config["target"]["circle_lock"][
        "phase_maximum_step"]

    assert controller._try_adjust_circle_target(mask, improved_a, 2.0)

    assert controller._circle_phase_trial_stage == 0
    assert controller._final_frequency_hz == original_frequency
    assert controller._tuning_word == original_word
    assert controller._circle_frequency_adjustments == 0
    assert len(sent) == 2


def test_locked_circle_finishes_pending_phase_trial_before_fast_path(
        monkeypatch) -> None:
    config = controller_config()
    config["target"]["circle_lock"][
        "frequency_integral_required_steps"] = 1
    config["target"]["circle_lock"]["frames_per_block"] = 1
    config["target"]["circle_lock"]["maximum_frame_attempts"] = 1
    controller = single.AutoLissajousController(
        config, RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._circle_frequency_verified = True
    controller._circle_locked_announced = True
    controller._coarse_frequency_hz = 50_000.0
    controller._final_frequency_hz = 50_000.0
    controller._tuning_word = single.dds_tuning_word_for_frequency(50_000.0)
    controller._phase = 68
    controller._circle_phase_trial_baseline = 64
    controller._circle_phase_trial_baseline_score = 0.10
    controller._circle_phase_trial_delta = 4
    controller._circle_phase_trial_stage = 1
    controller._circle_last_servo_at = 9.0
    mask = np.ones((32, 32), np.uint8)
    sent: list[int] = []
    monkeypatch.setattr(single.time, "monotonic", lambda: 10.0)
    monkeypatch.setattr(controller, "_read_target_mask", lambda: mask)
    monkeypatch.setattr(
        controller, "_circle_capture_mask_is_usable", lambda _mask: True)
    monkeypatch.setattr(single, "aggregate_masks", lambda _masks: mask)
    monkeypatch.setattr(
        single, "analyze_circle_lock_mask",
        lambda _mask, _config: locked_circle_fit())
    monkeypatch.setattr(
        single, "analyze_frequency_trace_masks",
        lambda _masks, _config: trace_fit(
            0.90, span_x_div=7.8, span_y_div=7.8))
    monkeypatch.setattr(
        controller.target_analyzer,
        "analyze",
        lambda _mask, _target: single.TargetFit(
            0, 0.1, 90, 7.8, 7.8, 0.0),
    )
    monkeypatch.setattr(
        controller, "_send_circle_confirm_target",
        lambda _now: sent.append(controller._tuning_word),
    )

    original_frequency = controller._final_frequency_hz
    controller._capture_circle_confirm_frame(10.0)

    assert controller._circle_phase_trial_stage == 0
    assert controller._circle_frequency_adjustments == 0
    assert controller._final_frequency_hz == original_frequency
    assert sent


def test_good_maintenance_block_preserves_frequency_integrator_window(
        monkeypatch) -> None:
    config = controller_config()
    config["target"]["circle_lock"]["frames_per_block"] = 1
    config["target"]["circle_lock"]["maximum_frame_attempts"] = 1
    controller = single.AutoLissajousController(
        config, RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._circle_frequency_verified = True
    controller._circle_locked_announced = True
    controller._circle_last_servo_at = 4.0
    mask = np.ones((32, 32), np.uint8)
    monkeypatch.setattr(single.time, "monotonic", lambda: 10.0)
    monkeypatch.setattr(controller, "_read_target_mask", lambda: mask)
    monkeypatch.setattr(
        controller, "_circle_capture_mask_is_usable", lambda _mask: True)
    monkeypatch.setattr(single, "aggregate_masks", lambda _masks: mask)
    monkeypatch.setattr(
        single, "analyze_circle_lock_mask",
        lambda _mask, _config: locked_circle_fit())
    monkeypatch.setattr(
        single, "analyze_frequency_trace_masks",
        lambda _masks, _config: trace_fit(
            0.90, span_x_div=7.8, span_y_div=7.8))
    monkeypatch.setattr(
        controller.target_analyzer,
        "analyze",
        lambda _mask, _target: single.TargetFit(
            64, 0.0, 95, 7.8, 7.8, 0.0),
    )

    controller._capture_circle_confirm_frame(10.0)

    assert controller._circle_last_servo_at == 4.0
    assert controller.mode == "CIRCLE_MAINTAIN_SETTLE"


def test_frozen_lock_skips_circle_servo_after_lock(monkeypatch) -> None:
    config = controller_config()
    config["target"]["freeze_after_lock"] = True
    config["target"]["circle_lock"]["frames_per_block"] = 1
    config["target"]["circle_lock"]["maximum_frame_attempts"] = 1
    controller = single.AutoLissajousController(
        config, RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._circle_frequency_verified = True
    controller._circle_locked_announced = True
    controller._circle_last_servo_at = 4.0
    mask = np.ones((32, 32), np.uint8)

    monkeypatch.setattr(single.time, "monotonic", lambda: 10.0)
    monkeypatch.setattr(controller, "_read_target_mask", lambda: mask)
    monkeypatch.setattr(
        controller, "_circle_capture_mask_is_usable", lambda _mask: True)
    monkeypatch.setattr(single, "aggregate_masks", lambda _masks: mask)
    monkeypatch.setattr(
        single, "analyze_circle_lock_mask",
        lambda _mask, _config: locked_circle_fit())
    monkeypatch.setattr(
        single, "analyze_frequency_trace_masks",
        lambda _masks, _config: trace_fit(
            0.90, span_x_div=7.8, span_y_div=7.8))
    monkeypatch.setattr(
        controller.target_analyzer,
        "analyze",
        lambda _mask, _target: single.TargetFit(
            80, 0.0, 95, 7.8, 7.8, 0.0),
    )
    monkeypatch.setattr(
        controller,
        "_try_adjust_circle_target",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("servo should be skipped after frozen lock")),
    )

    controller._capture_circle_confirm_frame(10.0)

    assert controller._circle_last_servo_at == 4.0
    assert controller.mode == "CIRCLE_MAINTAIN_SETTLE"


def test_frequency_integral_obeys_sign_and_step_limit() -> None:
    config = controller_config()
    config["target"]["circle_lock"][
        "frequency_integral_required_steps"] = 1
    controller = single.AutoLissajousController(
        config, RecordingLink(), FreshFrameCamera())
    controller._circle_locked_announced = True
    controller._coarse_frequency_hz = 49_900.0
    controller._final_frequency_hz = 50_000.0
    controller._tuning_word = single.dds_tuning_word_for_frequency(50_000.0)
    controller._circle_last_servo_at = 1.0
    maximum_step = config["target"]["circle_lock"][
        "frequency_maximum_step_hz"]

    assert controller._integrate_circle_frequency(127, 2.0)
    positive_frequency = controller._final_frequency_hz
    assert 0.0 < positive_frequency - 50_000.0 <= maximum_step

    assert controller._integrate_circle_frequency(-127, 3.0)
    assert controller._final_frequency_hz < positive_frequency
    assert controller._circle_frequency_adjustments == 2


def test_frequency_integral_requires_repeated_same_direction() -> None:
    config = controller_config()
    controller = single.AutoLissajousController(
        config, RecordingLink(), FreshFrameCamera())
    controller._circle_locked_announced = True
    controller._coarse_frequency_hz = 50_000.0
    controller._final_frequency_hz = 50_000.0
    controller._circle_frequency_anchor_hz = 50_000.0
    controller._tuning_word = single.dds_tuning_word_for_frequency(50_000.0)
    controller._circle_last_servo_at = 1.0

    assert not controller._integrate_circle_frequency(4, 2.0)
    assert controller._circle_frequency_adjustments == 0
    assert controller._integrate_circle_frequency(4, 3.0)
    assert controller._circle_frequency_adjustments == 1
    assert controller._final_frequency_hz > 50_000.0


def test_frequency_integral_total_correction_is_bounded() -> None:
    config = controller_config()
    circle = config["target"]["circle_lock"]
    circle["frequency_integral_required_steps"] = 1
    circle["frequency_maximum_total_correction_hz"] = 5.0
    controller = single.AutoLissajousController(
        config, RecordingLink(), FreshFrameCamera())
    controller._circle_locked_announced = True
    controller._coarse_frequency_hz = 50_000.0
    controller._circle_frequency_anchor_hz = 50_000.0
    controller._final_frequency_hz = 50_005.0
    controller._tuning_word = single.dds_tuning_word_for_frequency(50_005.0)
    controller._circle_last_servo_at = 1.0

    assert not controller._integrate_circle_frequency(127, 2.0)
    assert controller._final_frequency_hz == 50_005.0


@pytest.mark.parametrize(
    ("frequency_hz", "phase_step"),
    ((100_000.0, 127), (1_000.0, -127)),
)
def test_frequency_integral_stays_inside_protocol_limits(
        frequency_hz: float, phase_step: int) -> None:
    config = controller_config()
    config["target"]["circle_lock"][
        "frequency_integral_required_steps"] = 1
    controller = single.AutoLissajousController(
        config, RecordingLink(), FreshFrameCamera())
    controller._circle_locked_announced = True
    controller._coarse_frequency_hz = frequency_hz
    controller._final_frequency_hz = frequency_hz
    controller._tuning_word = single.dds_tuning_word_for_frequency(
        frequency_hz)
    controller._circle_last_servo_at = 1.0

    assert not controller._integrate_circle_frequency(phase_step, 2.0)
    assert controller._final_frequency_hz == frequency_hz
    assert controller._tuning_word == single.dds_tuning_word_for_frequency(
        frequency_hz)
