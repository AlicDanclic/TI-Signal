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


def fast_config() -> dict:
    config = copy.deepcopy(single.DEFAULT_CONFIG)
    config["target"]["circle_lock"]["fast_single_frame_enabled"] = True
    return config


def ellipse_mask(axes: tuple[int, int] = (220, 150)) -> np.ndarray:
    mask = np.zeros((512, 640), np.uint8)
    cv2.ellipse(
        mask, (320, 256), axes, 18.0, 0.0, 360.0,
        255, 5, cv2.LINE_AA,
    )
    return mask


def test_fast_seed_uses_one_screen_frame() -> None:
    controller = single.AutoLissajousController(
        fast_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._circle_sweep_stage = "SCREEN"

    required, minimum, attempts, _, _ = controller._circle_stage_parameters()

    assert (required, minimum, attempts) == (1, 1, 1)


def test_one_ellipse_frame_stops_broad_sweep(monkeypatch) -> None:
    controller = single.AutoLissajousController(
        fast_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._coarse_frequency_hz = 50_000.0
    controller._circle_sweep_stage = "SCREEN"
    controller._circle_sweep_frequencies = [50_000.0, 50_100.0]
    controller._circle_sweep_index = 0
    controller._circle_current_masks = [ellipse_mask()]
    accepted: list[single.CircleSweepResult] = []
    monkeypatch.setattr(
        controller,
        "_accept_circle_frequency",
        lambda result, _now: accepted.append(result),
    )

    controller._finish_circle_sweep_candidate(1.0)

    assert len(accepted) == 1
    assert accepted[0].frequency_hz == 50_000.0
    assert controller._circle_sweep_index == 0
    assert controller._circle_frequency_verified


def test_one_nearly_round_frame_enters_lock(monkeypatch) -> None:
    controller = single.AutoLissajousController(
        fast_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._mode = "CIRCLE_CONFIRM_CAPTURE"
    controller._circle_frequency_verified = True
    controller._coarse_frequency_hz = 10_000.0
    controller._final_frequency_hz = 10_000.0
    controller._tuning_word = single.dds_tuning_word_for_frequency(10_000.0)
    mask = ellipse_mask((210, 205))
    monkeypatch.setattr(controller, "_read_target_mask", lambda: mask)

    controller._capture_circle_confirm_frame(2.0)

    assert controller._circle_locked_announced
    assert controller.mode == "CIRCLE_MAINTAIN_SETTLE"


def test_filled_or_figure_eight_center_is_not_a_fast_ellipse_seed() -> None:
    fit = single.CircleLockFit(
        quality=90,
        score=0.90,
        span_x_div=7.0,
        span_y_div=6.8,
        center_error_div=0.1,
        radial_cv=0.12,
        inner_fill_ratio=0.30,
        angular_coverage=1.0,
        fill_ratio=0.90,
        pixel_count=40_000,
        ellipse_axis_ratio=1.05,
    )

    assert not single.circle_fit_is_fast_ellipse_seed(
        fit, 0.10, fast_config())


def test_fast_control_seed_kind_accepts_phase_seed_when_ellipse_fit_fails() -> None:
    result = single.CircleSweepResult(
        frequency_hz=19_000.0,
        tuning_word=single.dds_tuning_word_for_frequency(19_000.0),
        amplitude=103,
        phase=64,
        fit=single.CircleLockFit(
            quality=72,
            score=0.72,
            span_x_div=7.73,
            span_y_div=7.56,
            center_error_div=0.20,
            radial_cv=0.031,
            inner_fill_ratio=0.000,
            angular_coverage=0.17,
            fill_ratio=0.002,
            pixel_count=2101,
            ellipse_axis_ratio=5.619,
        ),
        trace_fit=single.FrequencyTraceFit(
            quality=92,
            score=0.92,
            thinness_quality=0.79,
            temporal_overlap=1.00,
            extent_quality=1.00,
            span_x_div=7.73,
            span_y_div=7.56,
            thickness_px=7.6,
            pixel_count=2101,
            valid_frames=1,
            aggregate_pixel_count=2101,
            total_frames=1,
        ),
        phase_fit=single.TargetFit(
            estimated_phase=64,
            desired_score=0.010,
            quality=86,
            span_x_div=7.73,
            span_y_div=7.56,
            center_error_div=0.20,
            model_score=0.020,
        ),
        foreground_occupancy=0.009,
    )

    assert not single.circle_fit_is_fast_ellipse_seed(
        result.fit, result.foreground_occupancy, fast_config())
    assert single.circle_result_fast_seed_kind(result, fast_config()) == "control"
