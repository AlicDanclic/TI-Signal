import ast
import copy
from pathlib import Path

import numpy as np
import pytest

import task5_cv_single as single


class RecordingLink:
    def __init__(self) -> None:
        self.frames: list[single.Frame] = []

    def send_frame(
        self,
        command: int,
        payload: bytes,
        *,
        flags: int = 0,
    ) -> single.Frame:
        frame = single.Frame(len(self.frames) & 0xFF, command, payload, flags)
        self.frames.append(frame)
        return frame


class FreshFrameCamera:
    def require_frame_after(self, _timestamp: float) -> None:
        return None


def controller_config() -> dict:
    return copy.deepcopy(single.DEFAULT_CONFIG)


def circle_fit(score: float = 0.85) -> single.CircleLockFit:
    return single.CircleLockFit(
        quality=int(round(score * 100.0)),
        score=score,
        span_x_div=7.8,
        span_y_div=7.7,
        center_error_div=0.1,
        radial_cv=0.08,
        inner_fill_ratio=0.10,
        angular_coverage=0.80,
        fill_ratio=0.16,
        pixel_count=900,
        ellipse_axis_ratio=1.05,
    )


def trace_fit(score: float = 0.85) -> single.FrequencyTraceFit:
    return single.FrequencyTraceFit(
        quality=int(round(score * 100.0)),
        score=score,
        thinness_quality=0.80,
        temporal_overlap=0.82,
        extent_quality=0.90,
        span_x_div=7.8,
        span_y_div=7.7,
        thickness_px=5.0,
        pixel_count=800,
        valid_frames=5,
        aggregate_pixel_count=900,
        total_frames=5,
    )


def sweep_result(frequency_hz: float, score: float = 0.85) -> single.CircleSweepResult:
    return single.CircleSweepResult(
        frequency_hz=frequency_hz,
        tuning_word=single.dds_tuning_word_for_frequency(frequency_hz),
        amplitude=103,
        phase=64,
        fit=circle_fit(score),
        trace_fit=trace_fit(score),
        phase_fit=single.TargetFit(64, 0.01, 90, 7.8, 7.7, 0.1, 0.01),
        foreground_occupancy=0.08,
    )


def test_mirrored_phase_estimator_rejects_68_188_branch_jumps() -> None:
    samples = [
        (0.0, 64),
        (0.2, 192),
        (0.4, 68),
        (0.6, 188),
        (0.8, 64),
    ]

    estimate = single.estimate_mirrored_phase_drift(samples)

    assert estimate is not None
    assert estimate.magnitude_hz <= 0.020
    assert estimate.inlier_fraction >= 0.80


def test_mirrored_phase_estimator_recovers_known_drift_magnitude() -> None:
    rate_hz = 0.050
    samples = []
    for index, timestamp in enumerate(np.linspace(0.0, 1.0, 6)):
        physical = int(round(48.0 + rate_hz * 256.0 * timestamp)) & 0xFF
        observed = physical if index % 2 == 0 else (-physical) & 0xFF
        samples.append((float(timestamp), observed))

    estimate = single.estimate_mirrored_phase_drift(samples)

    assert estimate is not None
    assert estimate.magnitude_hz == pytest.approx(rate_hz, abs=0.008)
    assert estimate.residual_codes <= 2.0


def test_screen_boundary_winner_expands_before_sub_hz_scan(monkeypatch) -> None:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._coarse_frequency_hz = 10_000.0
    controller._circle_screen_step_hz = 100.0
    controller._circle_sweep_tiers = single.circle_sweep_frequency_tiers(
        10_000.0, (300.0, 800.0), 100.0)
    controller._circle_sweep_tier_index = 0
    controller._circle_sweep_results = [
        sweep_result(9_700.0, 0.94),
        sweep_result(10_000.0, 0.70),
    ]
    expanded: list[str] = []
    monkeypatch.setattr(
        controller,
        "_expand_circle_sweep",
        lambda _now, reason, *_args: expanded.append(reason),
    )

    controller._finish_circle_screen_tier(1.0)

    assert expanded == ["best stationary trace is on current sweep boundary"]
    assert controller._circle_sweep_stage == "SCREEN"


def test_interior_screen_winner_enters_repeated_validation() -> None:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._coarse_frequency_hz = 10_000.0
    controller._circle_screen_step_hz = 100.0
    controller._circle_sweep_tiers = single.circle_sweep_frequency_tiers(
        10_000.0, (300.0, 800.0), 100.0)
    controller._circle_sweep_tier_index = 0
    controller._circle_sweep_results = [
        sweep_result(10_000.0, 0.94),
        sweep_result(10_100.0, 0.75),
    ]

    controller._finish_circle_screen_tier(1.0)

    assert controller._circle_sweep_stage == "VERIFY"
    assert controller._circle_validation_anchor_hz == 10_000.0
    assert controller._circle_sweep_frequencies[0] == 10_000.0
    assert controller._circle_sweep_frequencies[-1] == 10_000.0


def test_validation_passes_to_micro_before_hold() -> None:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._coarse_frequency_hz = 10_020.0
    controller._circle_validation_anchor_hz = 10_000.0
    controller._circle_validation_results = [
        sweep_result(10_000.0, 0.92),
        sweep_result(10_100.0, 0.70),
        sweep_result(10_000.0, 0.90),
    ]

    controller._finish_circle_validation(2.0)

    assert controller._circle_sweep_stage == "MICRO"
    assert controller._circle_micro_seed_hz == 10_000.0
    assert controller._circle_sweep_frequencies[0] == 10_000.0


def test_circle_phase_trial_never_integrates_its_own_command(monkeypatch) -> None:
    config = controller_config()
    config["target"]["circle_lock"]["frequency_integral_required_steps"] = 1
    controller = single.AutoLissajousController(
        config, RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._phase = 72
    controller._amplitude = 103
    controller._coarse_frequency_hz = 10_000.0
    controller._final_frequency_hz = 10_000.0
    controller._tuning_word = single.dds_tuning_word_for_frequency(10_000.0)
    controller._circle_locked_announced = True
    controller._circle_phase_trial_baseline = 64
    controller._circle_phase_trial_baseline_score = 0.40
    controller._circle_phase_trial_delta = 8
    controller._circle_phase_trial_stage = 1
    monkeypatch.setattr(
        controller, "_send_circle_confirm_target", lambda _now: None)
    original_word = controller._tuning_word

    outcome = controller._try_adjust_circle_target(
        np.ones((32, 32), np.uint8),
        circle_fit(0.90),
        2.0,
        single.TargetFit(64, 0.01, 90, 7.8, 7.7, 0.1, 0.01),
    )

    assert outcome == single.CIRCLE_ADJUST_SENT
    assert controller._tuning_word == original_word
    assert controller._circle_frequency_adjustments == 0


def _feed_drift_window(
    controller: single.AutoLissajousController,
    rate_hz: float,
    start_s: float,
) -> int:
    outcome = single.CIRCLE_DRIFT_WAIT
    for index in range(5):
        elapsed = 0.2 * index
        phase = int(round(44.0 + rate_hz * 256.0 * elapsed)) & 0xFF
        observed = phase if index % 2 == 0 else (-phase) & 0xFF
        fit = single.TargetFit(observed, 0.02, 80, 7.8, 7.7, 0.1, 0.02)
        outcome = controller._observe_circle_frequency_drift(
            fit, start_s + elapsed)
    return outcome


def test_frequency_probe_resolves_direction_and_removes_50mhz_drift(
    monkeypatch,
) -> None:
    config = controller_config()
    circle = config["target"]["circle_lock"]
    circle["frequency_drift_minimum_samples"] = 5
    circle["frequency_drift_minimum_span_s"] = 0.65
    controller = single.AutoLissajousController(
        config, RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._coarse_frequency_hz = 10_000.0
    controller._final_frequency_hz = 10_000.0
    controller._circle_frequency_anchor_hz = 10_000.0
    controller._tuning_word = single.dds_tuning_word_for_frequency(10_000.0)
    controller._reset_circle_drift_controller(True)
    sent: list[tuple[str, float]] = []
    monkeypatch.setattr(
        controller,
        "_send_circle_confirm_target",
        lambda _now: sent.append((
            controller._circle_drift_state,
            controller._final_frequency_hz,
        )),
    )

    assert _feed_drift_window(controller, 0.050, 0.0) == single.CIRCLE_DRIFT_SENT
    assert controller._circle_drift_state == "PLUS"
    assert _feed_drift_window(controller, 0.300, 2.0) == single.CIRCLE_DRIFT_SENT
    assert controller._circle_drift_state == "MINUS"
    assert _feed_drift_window(controller, 0.200, 4.0) == single.CIRCLE_DRIFT_SENT

    assert controller._circle_drift_state == "BASELINE"
    assert controller._circle_frequency_adjustments == 1
    assert controller._final_frequency_hz == pytest.approx(9_999.95, abs=0.02)
    assert [stage for stage, _ in sent] == ["PLUS", "MINUS", "BASELINE"]


def test_confirm_blocks_frequency_drift_before_static_phase_servo(
    monkeypatch,
) -> None:
    config = controller_config()
    circle = config["target"]["circle_lock"]
    circle.update({
        "frames_per_block": 1,
        "maximum_frame_attempts": 1,
        "frequency_drift_minimum_samples": 3,
        "frequency_drift_maximum_samples": 5,
        "frequency_drift_minimum_span_s": 0.40,
    })
    controller = single.AutoLissajousController(
        config, RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._mode = "CIRCLE_CONFIRM_CAPTURE"
    controller._circle_frequency_verified = True
    controller._coarse_frequency_hz = 10_000.0
    controller._final_frequency_hz = 10_000.0
    controller._circle_frequency_anchor_hz = 10_000.0
    controller._tuning_word = single.dds_tuning_word_for_frequency(10_000.0)
    controller._reset_circle_drift_controller(True)
    mask = np.ones((64, 64), np.uint8) * 255
    phases = iter((44, (-47) & 0xFF, 49))
    clock = [0.0]
    static_adjustments: list[float] = []
    target_sends: list[str] = []
    monkeypatch.setattr(single.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(controller, "_read_target_mask", lambda: mask)
    monkeypatch.setattr(
        controller, "_circle_capture_mask_is_usable", lambda _mask: True)
    monkeypatch.setattr(single, "aggregate_masks", lambda _masks: mask)
    monkeypatch.setattr(
        single, "analyze_circle_lock_mask", lambda _mask, _config: circle_fit(0.60))
    monkeypatch.setattr(
        single,
        "analyze_frequency_trace_masks",
        lambda _masks, _config: trace_fit(0.85),
    )
    monkeypatch.setattr(
        controller.target_analyzer,
        "analyze",
        lambda _mask, _target: single.TargetFit(
            next(phases), 0.08, 70, 7.8, 7.7, 0.1, 0.05),
    )
    monkeypatch.setattr(
        controller,
        "_try_adjust_circle_target",
        lambda *_args: static_adjustments.append(clock[0]),
    )
    monkeypatch.setattr(
        controller,
        "_send_circle_confirm_target",
        lambda _now: target_sends.append(controller._circle_drift_state),
    )

    for timestamp in (0.0, 0.2, 0.4):
        clock[0] = timestamp
        controller._capture_circle_confirm_frame(timestamp)

    assert controller._circle_drift_state == "PLUS"
    assert target_sends == ["PLUS"]
    assert static_adjustments == []


def _definitions(path: Path) -> dict[str, str]:
    source = path.read_text(encoding="utf-8")
    tree = ast.parse(source)
    lines = source.splitlines()
    definitions: dict[str, str] = {}
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            definitions[node.name] = "\n".join(
                lines[node.lineno - 1:node.end_lineno])
    return definitions


def test_r15_keeps_frequency_measurement_code_identical_to_r14() -> None:
    current = Path(single.__file__).resolve()
    backup = current.parent / "versions" / (
        "20260802_cv_r14_before_circle_drift_fix") / "task5_cv_single.py"
    current_definitions = _definitions(current)
    backup_definitions = _definitions(backup)
    protected = {
        "FrequencyEstimator",
        "TemporalPeriodFilter",
        "process_frame",
        "select_coarse_candidate",
        "_coarse_summary",
        "_finish_coarse",
        "coarse_observation_from_points",
        "summarize_coarse_observations",
        "reject_integer_multiple_periods",
        "reject_integer_multiple_periods_by_side",
        "choose_observed_fundamental_period",
        "choose_observed_long_period",
        "compute_same_side_period_samples",
        "select_standard_period_samples",
        "count_raw_turning_bands",
        "localize_turning_point",
        "deduplicate_turning_candidates",
        "dense_candidate_period_is_consistent",
        "prepare_period_detection_signal",
        "estimate_shared_profile_period",
    }

    assert protected <= current_definitions.keys()
    assert protected <= backup_definitions.keys()
    for name in protected:
        assert current_definitions[name] == backup_definitions[name]


def test_circle_phase_deadband_covers_visual_phase_quantization() -> None:
    phase_step = single.DEFAULT_CONFIG["vision"]["phase_search_step"]
    deadband = single.DEFAULT_CONFIG["target"]["circle_lock"]["phase_deadband"]

    assert deadband >= phase_step
