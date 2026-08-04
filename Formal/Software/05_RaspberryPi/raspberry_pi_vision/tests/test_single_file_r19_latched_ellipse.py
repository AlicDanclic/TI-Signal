import copy

import cv2
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


def fast_config() -> dict:
    config = copy.deepcopy(single.DEFAULT_CONFIG)
    config["target"]["circle_lock"]["fast_single_frame_enabled"] = True
    return config


def ellipse_mask() -> np.ndarray:
    mask = np.zeros((512, 640), np.uint8)
    cv2.ellipse(
        mask, (320, 256), (220, 150), 18.0, 0.0, 360.0,
        255, 5, cv2.LINE_AA,
    )
    return mask


def ellipse_result(frequency_hz: float = 10_000.0) -> single.CircleSweepResult:
    mask = ellipse_mask()
    fit = single.analyze_circle_lock_mask(mask, fast_config())
    trace = single.analyze_frequency_trace_masks(
        [mask], fast_config(), minimum_frames_override=1)
    phase = single.TargetAnalyzer(fast_config()).analyze(
        mask, single.TARGET_CIRCLE)
    return single.CircleSweepResult(
        frequency_hz,
        single.dds_tuning_word_for_frequency(frequency_hz),
        103,
        64,
        fit,
        trace,
        phase,
        single.target_mask_foreground_occupancy(mask),
    )


def test_confirmation_rejection_restores_latched_seed_without_screen() -> None:
    link = RecordingLink()
    controller = single.AutoLissajousController(
        fast_config(), link, FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._coarse_frequency_hz = 10_900.0
    controller._circle_sweep_index = 4
    controller._circle_sweep_frequencies = [9_800.0, 9_900.0, 10_000.0]
    selected = ellipse_result()
    controller._circle_best = selected
    controller._latch_fast_circle_seed(selected)

    controller._final_frequency_hz = 10_000.5
    controller._tuning_word = single.dds_tuning_word_for_frequency(10_000.5)
    controller._amplitude = 119
    controller._phase = 37
    controller._circle_corrections = 7
    controller._circle_rejected_frequencies = {9_900.0}

    controller._reject_circle_confirmation_frequency(
        2.0, "three unusable blocks")

    assert controller._circle_fast_seed_latched
    assert controller._circle_fast_seed_recoveries == 1
    assert controller._circle_sweep_index == 4
    assert controller._circle_rejected_frequencies == {9_900.0}
    assert controller._final_frequency_hz == pytest.approx(10_000.0)
    assert controller._tuning_word == selected.tuning_word
    assert controller._amplitude == 103
    assert controller._phase == 64
    assert controller._circle_frequency_verified
    assert controller.mode == "CIRCLE_CONFIRM_SETTLE"
    assert link.frames[-1].command == single.CMD_TARGET


def test_three_unusable_blocks_recover_locally_not_next_candidate(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    controller = single.AutoLissajousController(
        fast_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._coarse_frequency_hz = 10_900.0
    controller._circle_sweep_index = 2
    selected = ellipse_result()
    controller._circle_best = selected
    controller._latch_fast_circle_seed(selected)
    controller._final_frequency_hz = selected.frequency_hz
    controller._tuning_word = selected.tuning_word
    controller._amplitude = selected.amplitude
    controller._phase = selected.phase
    controller._circle_frequency_verified = True
    controller._reset_circle_drift_controller(True)
    monkeypatch.setattr(
        controller, "_read_target_mask",
        lambda: np.zeros((512, 640), np.uint8))
    monkeypatch.setattr(
        controller,
        "_send_next_circle_sweep_candidate",
        lambda _now: pytest.fail("latched seed resumed broad SCREEN"),
    )

    for index in range(6):
        controller._mode = "CIRCLE_CONFIRM_CAPTURE"
        controller._capture_circle_confirm_frame(3.0 + index)

    assert controller._circle_fast_seed_recoveries == 1
    assert controller._circle_sweep_index == 2
    assert controller._circle_frequency_verified
    assert controller.mode == "CIRCLE_CONFIRM_SETTLE"


def test_latched_seed_observes_frequency_drift_before_phase_servo(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    controller = single.AutoLissajousController(
        fast_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._mode = "CIRCLE_CONFIRM_CAPTURE"
    controller._coarse_frequency_hz = 10_900.0
    selected = ellipse_result()
    controller._circle_best = selected
    controller._latch_fast_circle_seed(selected)
    controller._final_frequency_hz = selected.frequency_hz
    controller._tuning_word = selected.tuning_word
    controller._amplitude = selected.amplitude
    controller._phase = selected.phase
    controller._circle_frequency_verified = True
    controller._reset_circle_drift_controller(True)
    mask = ellipse_mask()
    monkeypatch.setattr(controller, "_read_target_mask", lambda: mask)
    monkeypatch.setattr(
        controller,
        "_observe_circle_frequency_drift",
        lambda _fit, _now: single.CIRCLE_DRIFT_WAIT,
    )
    monkeypatch.setattr(
        controller,
        "_try_adjust_circle_target",
        lambda *_args: pytest.fail("phase servo ran before drift observation"),
    )

    controller._capture_circle_confirm_frame(4.0)

    assert controller._phase == selected.phase
    assert controller._amplitude == selected.amplitude
    assert controller.mode == "CIRCLE_CONFIRM_SETTLE"


def test_latched_seed_is_not_aborted_by_control_timeout() -> None:
    controller = single.AutoLissajousController(
        fast_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._mode = "CIRCLE_CONFIRM_SETTLE"
    controller._deadline = 2_000.0
    controller._run_started = 0.0
    controller._circle_fast_seed_latched = True
    controller._circle_fast_seed_kind = "ellipse"

    controller.poll(1_000.0)

    assert controller.mode == "CIRCLE_CONFIRM_SETTLE"


def test_control_seed_rejection_resumes_next_screen_candidate(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    controller = single.AutoLissajousController(
        fast_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._circle_sweep_index = 2
    controller._circle_sweep_frequencies = [9_800.0, 9_900.0, 10_000.0]
    controller._final_frequency_hz = 10_000.0
    controller._circle_micro_seed_hz = 10_000.0
    controller._circle_best = ellipse_result()
    controller._latch_fast_circle_seed(controller._circle_best, "control")

    resumed: list[float] = []
    monkeypatch.setattr(
        controller,
        "_send_next_circle_sweep_candidate",
        lambda now: resumed.append(now),
    )

    controller._reject_circle_confirmation_frequency(5.0, "weak seed failed")

    assert resumed == [5.0]
    assert not controller._circle_fast_seed_latched
    assert controller._circle_fast_seed_kind == "none"
    assert controller._circle_sweep_index == 3
