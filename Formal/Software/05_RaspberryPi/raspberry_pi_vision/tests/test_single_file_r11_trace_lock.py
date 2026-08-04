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


def green_ellipse_frame(*, split_at_edges: bool = False) -> np.ndarray:
    frame = np.zeros((512, 640, 3), np.uint8)
    frame[:, :, 1] = 90
    cv2.ellipse(
        frame,
        (320, 256),
        (250, 250),
        0.0,
        0.0,
        360.0,
        (0, 245, 0),
        10,
        cv2.LINE_AA,
    )
    if split_at_edges:
        frame[:15, :, 1] = 90
        frame[-15:, :, 1] = 90
    return frame


def stable_trace_fit() -> single.FrequencyTraceFit:
    return single.FrequencyTraceFit(
        quality=82,
        score=0.82,
        thinness_quality=0.70,
        temporal_overlap=0.90,
        extent_quality=1.0,
        span_x_div=7.8,
        span_y_div=7.5,
        thickness_px=10.0,
        pixel_count=1200,
        valid_frames=1,
        aggregate_pixel_count=1200,
        total_frames=1,
    )


def test_target_extractor_uses_green_channel_only() -> None:
    green = green_ellipse_frame()[:, :, 1]
    low_rb = np.dstack((np.zeros_like(green), green, np.zeros_like(green)))
    high_rb = np.dstack((
        np.full_like(green, 255),
        green,
        np.full_like(green, 255),
    ))

    low_mask = single.extract_target_trace_mask(
        low_rb, single.DEFAULT_CONFIG)
    high_mask = single.extract_target_trace_mask(
        high_rb, single.DEFAULT_CONFIG)

    np.testing.assert_array_equal(low_mask, high_mask)
    assert cv2.countNonZero(low_mask) > 10_000


def test_target_extractor_preserves_disconnected_edge_arcs() -> None:
    mask = single.extract_target_trace_mask(
        green_ellipse_frame(split_at_edges=True), single.DEFAULT_CONFIG)
    component_count, _, stats, _ = cv2.connectedComponentsWithStats(
        (mask > 0).astype(np.uint8), connectivity=8)

    assert component_count - 1 == 2
    areas = sorted(stats[1:, cv2.CC_STAT_AREA], reverse=True)
    assert areas[1] >= 0.90 * areas[0]
    assert cv2.countNonZero(mask[:, :320]) > 5_000
    assert cv2.countNonZero(mask[:, 320:]) > 5_000

    fit = single.analyze_circle_lock_mask(mask, single.DEFAULT_CONFIG)
    assert fit.angular_coverage >= 0.80
    assert single.circle_fit_is_usable_for_correction(
        fit, single.DEFAULT_CONFIG)
    assert single.circle_fit_is_locked(fit, single.DEFAULT_CONFIG)


def test_circle_fit_failure_holds_dds_and_does_not_invoke_servo(
    monkeypatch,
) -> None:
    config = controller_config()
    config["target"]["circle_sweep"]["trace_minimum_frames"] = 1
    config["target"]["circle_lock"].update({
        "frames_per_block": 1,
        "maximum_frame_attempts": 1,
    })
    link = RecordingLink()
    controller = single.AutoLissajousController(
        config, link, FreshFrameCamera())
    controller._mode = "CIRCLE_CONFIRM_CAPTURE"
    controller._target = single.TARGET_CIRCLE
    controller._circle_frequency_verified = True
    controller._phase = 64
    controller._amplitude = 103
    controller._tuning_word = single.dds_tuning_word_for_frequency(20_000.0)
    original_parameters = (
        controller._phase,
        controller._amplitude,
        controller._tuning_word,
    )
    mask = single.extract_target_trace_mask(
        green_ellipse_frame(), single.DEFAULT_CONFIG)
    corrections: list[float] = []

    monkeypatch.setattr(controller, "_read_target_mask", lambda: mask)
    monkeypatch.setattr(
        controller.target_analyzer,
        "analyze",
        lambda _mask, _target: single.TargetFit(
            0, 0.01, 90, 7.8, 7.5, 0.0),
    )
    monkeypatch.setattr(
        single,
        "analyze_circle_lock_mask",
        lambda _mask, _config: (_ for _ in ()).throw(
            ValueError("circle trace is clipped")),
    )
    monkeypatch.setattr(
        single,
        "analyze_frequency_trace_masks",
        lambda _masks, _config: stable_trace_fit(),
    )
    monkeypatch.setattr(
        controller,
        "_try_adjust_circle_target",
        lambda _mask, _fit, now, _phase_fit=None: corrections.append(now),
    )

    controller._capture_circle_confirm_frame(3.0)

    assert corrections == []
    assert (
        controller._phase,
        controller._amplitude,
        controller._tuning_word,
    ) == original_parameters
    assert controller._circle_confirm_passes == 0


def test_invalid_geometry_aborts_pending_circle_trial_to_baseline(
    monkeypatch,
) -> None:
    config = controller_config()
    config["target"]["circle_sweep"]["trace_minimum_frames"] = 1
    config["target"]["circle_lock"].update({
        "frames_per_block": 1,
        "maximum_frame_attempts": 1,
    })
    controller = single.AutoLissajousController(
        config, RecordingLink(), FreshFrameCamera())
    controller._mode = "CIRCLE_CONFIRM_CAPTURE"
    controller._target = single.TARGET_CIRCLE
    controller._circle_frequency_verified = True
    controller._phase = 80
    controller._amplitude = 103
    controller._tuning_word = single.dds_tuning_word_for_frequency(20_000.0)
    controller._circle_phase_trial_baseline = 64
    controller._circle_phase_trial_delta = 16
    controller._circle_phase_trial_stage = 1
    mask = single.extract_target_trace_mask(
        green_ellipse_frame(), single.DEFAULT_CONFIG)
    sent_phases: list[int] = []

    monkeypatch.setattr(controller, "_read_target_mask", lambda: mask)
    monkeypatch.setattr(
        controller.target_analyzer,
        "analyze",
        lambda _mask, _target: single.TargetFit(
            0, 0.01, 90, 7.8, 7.5, 0.0),
    )
    monkeypatch.setattr(
        single,
        "analyze_circle_lock_mask",
        lambda _mask, _config: (_ for _ in ()).throw(
            ValueError("circle trace is clipped")),
    )
    monkeypatch.setattr(
        single,
        "analyze_frequency_trace_masks",
        lambda _masks, _config: stable_trace_fit(),
    )
    monkeypatch.setattr(
        controller,
        "_send_circle_confirm_target",
        lambda _now: sent_phases.append(controller._phase),
    )

    controller._capture_circle_confirm_frame(4.0)

    assert controller._phase == 64
    assert controller._circle_phase_trial_stage == 0
    assert controller._circle_phase_trial_baseline is None
    assert sent_phases == [64]
