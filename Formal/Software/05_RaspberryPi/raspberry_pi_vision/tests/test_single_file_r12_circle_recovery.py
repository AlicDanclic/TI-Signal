import copy

import cv2
import numpy as np

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


def trace_fit(score: float, span_x: float = 7.5,
              span_y: float = 7.2) -> single.FrequencyTraceFit:
    return single.FrequencyTraceFit(
        quality=int(round(score * 100.0)),
        score=score,
        thinness_quality=0.80,
        temporal_overlap=0.90,
        extent_quality=1.0,
        span_x_div=span_x,
        span_y_div=span_y,
        thickness_px=5.0,
        pixel_count=900,
        valid_frames=5,
        aggregate_pixel_count=1200,
        total_frames=5,
    )


def circle_fit(*, quality: int, coverage: float,
               span_x: float = 7.5, span_y: float = 7.2,
               radial_cv: float = 0.12) -> single.CircleLockFit:
    return single.CircleLockFit(
        quality=quality,
        score=quality / 100.0,
        span_x_div=span_x,
        span_y_div=span_y,
        center_error_div=0.2,
        radial_cv=radial_cv,
        inner_fill_ratio=0.10,
        angular_coverage=coverage,
        fill_ratio=0.18,
        pixel_count=900,
        ellipse_axis_ratio=1.3,
    )


def result(frequency: float, *, trace_score: float,
           shape_quality: int, coverage: float) -> single.CircleSweepResult:
    return single.CircleSweepResult(
        frequency,
        single.dds_tuning_word_for_frequency(frequency),
        103,
        64,
        circle_fit(quality=shape_quality, coverage=coverage),
        trace_fit(trace_score),
    )


def test_complete_ellipse_outranks_thinner_short_arc_for_circle() -> None:
    config = controller_config()
    complete = result(
        19_500.0, trace_score=0.82, shape_quality=85, coverage=0.86)
    short_arc = result(
        19_300.0, trace_score=0.96, shape_quality=69, coverage=0.31)

    complete_score = single.circle_sweep_result_control_score(
        complete, config, prefer_circle_geometry=True)
    short_arc_score = single.circle_sweep_result_control_score(
        short_arc, config, prefer_circle_geometry=True)

    assert complete_score > short_arc_score
    assert single.circle_sweep_result_control_score(
        short_arc, config) > single.circle_sweep_result_control_score(
            complete, config)


def test_hold_rejects_stable_short_arc_without_control_seed() -> None:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._coarse_frequency_hz = 19_600.0
    controller._circle_micro_seed_hz = 19_300.0
    short_arc = result(
        19_300.0, trace_score=0.90, shape_quality=45, coverage=0.25)
    fallback = result(
        19_300.5, trace_score=0.80, shape_quality=82, coverage=0.82)
    controller._circle_micro_results = [short_arc, fallback]
    controller._circle_sweep_frequencies = [short_arc.frequency_hz]
    controller._circle_hold_result = short_arc

    controller._finish_circle_hold(2.0)

    assert short_arc.frequency_hz in controller._circle_rejected_frequencies
    assert controller._circle_frequency_verified is False
    assert controller._circle_sweep_stage == "HOLD"
    assert controller._circle_sweep_frequencies == [fallback.frequency_hz]


def test_clean_line_seed_can_start_circle_phase_trial_without_circle_fit(
    monkeypatch,
) -> None:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._phase = 64
    controller._amplitude = 103
    controller._tuning_word = single.dds_tuning_word_for_frequency(20_000.0)
    mask = np.zeros((512, 640), np.uint8)
    cv2.line(mask, (80, 430), (560, 80), 255, 5, cv2.LINE_AA)
    phase_fit = single.TargetFit(
        0, 0.20, 25, 7.5, 6.0, 0.1, model_score=0.01)
    sent: list[int] = []
    monkeypatch.setattr(
        controller, "_send_circle_confirm_target",
        lambda _now: sent.append(controller._phase))

    changed = controller._try_adjust_circle_target(
        mask, None, 1.0, phase_fit)

    assert changed
    assert controller._circle_phase_trial_stage == 1
    assert controller._phase != 64
    assert sent == [controller._phase]


def test_dense_reflection_never_changes_control_parameters(monkeypatch) -> None:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._phase = 64
    controller._amplitude = 103
    controller._tuning_word = single.dds_tuning_word_for_frequency(20_000.0)
    dense = np.zeros((512, 640), np.uint8)
    cv2.rectangle(dense, (90, 80), (550, 430), 255, -1)
    phase_fit = single.TargetFit(
        0, 0.20, 25, 7.2, 5.5, 0.1, model_score=0.01)
    sent: list[int] = []
    monkeypatch.setattr(
        controller, "_send_circle_confirm_target",
        lambda _now: sent.append(controller._phase))
    before = (controller._phase, controller._amplitude,
              controller._tuning_word)

    changed = controller._try_adjust_circle_target(
        dense, None, 1.0, phase_fit)

    assert not changed
    assert (controller._phase, controller._amplitude,
            controller._tuning_word) == before
    assert sent == []


def test_three_invalid_confirmation_blocks_return_to_next_grid_seed(
    monkeypatch,
) -> None:
    config = controller_config()
    config["target"]["circle_lock"].update({
        "frames_per_block": 1,
        "maximum_frame_attempts": 1,
        "confirmation_maximum_invalid_blocks": 3,
    })
    config["target"]["circle_sweep"]["trace_minimum_frames"] = 1
    controller = single.AutoLissajousController(
        config, RecordingLink(), FreshFrameCamera())
    controller._mode = "CIRCLE_CONFIRM_CAPTURE"
    controller._target = single.TARGET_CIRCLE
    controller._coarse_frequency_hz = 50_000.0
    controller._circle_frequency_verified = True
    controller._circle_screen_step_hz = 500.0
    controller._circle_micro_seed_hz = 50_000.0
    controller._circle_best = result(
        50_000.5, trace_score=0.90, shape_quality=80, coverage=0.82)
    controller._circle_micro_results = [controller._circle_best]
    rejected_grid = result(
        50_000.0, trace_score=0.92, shape_quality=80, coverage=0.82)
    fallback_grid = result(
        50_100.0, trace_score=0.85, shape_quality=78, coverage=0.80)
    controller._circle_grid_seed_hz = 50_000.0
    controller._circle_grid_results = [rejected_grid, fallback_grid]
    invalid_mask = np.zeros((512, 640), np.uint8)
    cv2.rectangle(invalid_mask, (70, 70), (570, 440), 255, -1)
    invalid_phase = single.TargetFit(
        0, 0.20, 25, 7.8, 6.0, 0.1, model_score=0.20)
    monkeypatch.setattr(controller, "_read_target_mask", lambda: invalid_mask)
    monkeypatch.setattr(
        controller.target_analyzer, "analyze",
        lambda _mask, _target: invalid_phase)
    monkeypatch.setattr(
        single, "analyze_circle_lock_mask",
        lambda _mask, _config: circle_fit(
            quality=25, coverage=0.20, radial_cv=0.80))
    monkeypatch.setattr(
        single, "analyze_frequency_trace_masks",
        lambda _masks, _config: trace_fit(0.90))

    controller._capture_circle_confirm_frame(1.0)
    controller._capture_circle_confirm_frame(2.0)
    controller._capture_circle_confirm_frame(3.0)

    assert 50_000.0 in controller._circle_rejected_frequencies
    assert 50_000.5 in controller._circle_rejected_frequencies
    assert controller._circle_frequency_verified is False
    assert controller._circle_sweep_stage == "VERIFY"
    assert controller._circle_validation_anchor_hz == 50_100.0
    assert controller._circle_sweep_frequencies[0] == 50_100.0
    assert controller._circle_sweep_frequencies[-1] == 50_100.0
