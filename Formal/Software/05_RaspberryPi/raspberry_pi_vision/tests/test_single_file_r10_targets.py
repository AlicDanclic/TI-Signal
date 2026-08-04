import copy
from pathlib import Path

import cv2
import numpy as np
import pytest

import task5_cv_single as single


FIXTURE_DIR = Path(__file__).parent / "fixtures" / "target_shapes"
TARGET_CASES = (
    ("line.png", single.TARGET_DIAGONAL, 0),
    ("circle.png", single.TARGET_CIRCLE, 64),
    ("eight.png", single.TARGET_EIGHT, 0),
)


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
    def __init__(self) -> None:
        self.required_after: list[float] = []

    def require_frame_after(self, timestamp: float) -> None:
        self.required_after.append(timestamp)


def controller_config() -> dict:
    return copy.deepcopy(single.DEFAULT_CONFIG)


def load_target_mask(filename: str, config: dict | None = None) -> np.ndarray:
    active_config = single.DEFAULT_CONFIG if config is None else config
    frame = cv2.imread(str(FIXTURE_DIR / filename), cv2.IMREAD_COLOR)
    assert frame is not None
    extraction = active_config["target"]["trace_extraction"]
    screen_size = tuple(int(value) for value in extraction["screen_size"])
    rectified = single.rectify_screen(
        frame,
        single.get_target_screen_corners(frame, active_config),
        screen_size,
    )
    return single.extract_target_trace_mask(rectified, active_config)


def empty_circle_fit() -> single.CircleLockFit:
    return single.CircleLockFit(
        quality=0,
        score=0.0,
        span_x_div=0.0,
        span_y_div=0.0,
        center_error_div=99.0,
        radial_cv=99.0,
        inner_fill_ratio=1.0,
        angular_coverage=0.0,
        fill_ratio=1.0,
        pixel_count=0,
    )


def strong_trace_fit() -> single.FrequencyTraceFit:
    return single.FrequencyTraceFit(
        quality=88,
        score=0.88,
        thinness_quality=0.72,
        temporal_overlap=0.91,
        extent_quality=1.0,
        span_x_div=7.4,
        span_y_div=7.1,
        thickness_px=8.0,
        pixel_count=1200,
        valid_frames=3,
        aggregate_pixel_count=1600,
        total_frames=3,
    )


def prime_lock_controller(target: int) -> single.AutoLissajousController:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = target
    controller._coarse_frequency_hz = 45_000.0
    controller._final_frequency_hz = 45_000.0
    controller._frequency_correction_hz = 0.0
    controller._tuning_word = single.dds_tuning_word_for_frequency(45_000.0)
    controller._amplitude = controller._target_initial_amplitude(target)
    controller._phase = controller._target_initial_phase(target)
    return controller


@pytest.mark.parametrize(("filename", "expected_target", "expected_phase"),
                         TARGET_CASES)
def test_real_target_photographs_use_dedicated_extraction_and_classify(
    filename: str,
    expected_target: int,
    expected_phase: int,
) -> None:
    mask = load_target_mask(filename)

    assert mask.shape == (512, 640)
    assert set(np.unique(mask)).issubset({0, 255})
    assert cv2.countNonZero(mask) > 10_000

    analyzer = single.TargetAnalyzer(single.DEFAULT_CONFIG)
    fits = {target: analyzer.analyze(mask, target) for target in (1, 2, 3)}
    expected = fits[expected_target]
    wrong_quality = max(
        fit.quality for target, fit in fits.items() if target != expected_target)

    assert expected.quality >= 80
    assert expected.desired_score <= 0.010
    assert expected.quality >= wrong_quality + 40
    assert abs(single.target_phase_delta(expected.estimated_phase,
                                         expected_target)) <= 4
    assert expected.estimated_phase == expected_phase
    assert single.target_fit_is_locked(
        expected, expected_target, single.DEFAULT_CONFIG)


def test_target_analyzer_treats_zero_one_and_zero_255_masks_equally() -> None:
    mask_255 = load_target_mask("eight.png")
    mask_1 = (mask_255 > 0).astype(np.uint8)
    analyzer = single.TargetAnalyzer(single.DEFAULT_CONFIG)

    fit_255 = analyzer.analyze(mask_255, single.TARGET_EIGHT)
    fit_1 = analyzer.analyze(mask_1, single.TARGET_EIGHT)

    assert fit_1.estimated_phase == fit_255.estimated_phase
    assert fit_1.quality == fit_255.quality
    assert fit_1.desired_score == pytest.approx(fit_255.desired_score)
    assert fit_1.span_x_div == pytest.approx(fit_255.span_x_div)
    assert fit_1.span_y_div == pytest.approx(fit_255.span_y_div)


def test_target_corner_override_does_not_change_coarse_calibration() -> None:
    frame = np.zeros((480, 640, 3), np.uint8)
    baseline_coarse = single.get_fixed_screen_corners(frame)
    baseline_target = single.get_target_screen_corners(
        frame, single.DEFAULT_CONFIG)
    config = controller_config()
    config["target"]["trace_extraction"]["screen_corners"] = [
        [1.0, 2.0], [638.0, 3.0], [637.0, 478.0], [2.0, 477.0],
    ]

    overridden_target = single.get_target_screen_corners(frame, config)
    coarse_after_override = single.get_fixed_screen_corners(frame)

    assert not np.allclose(baseline_coarse, baseline_target)
    assert not np.allclose(overridden_target, baseline_target)
    np.testing.assert_allclose(coarse_after_override, baseline_coarse)


def test_stationary_real_line_is_a_strong_frequency_seed_without_circle_fit() -> None:
    mask = load_target_mask("line.png")
    trace_fit = single.analyze_frequency_trace_masks(
        [mask, mask.copy(), mask.copy()], single.DEFAULT_CONFIG)
    result = single.CircleSweepResult(
        frequency_hz=45_000.0,
        tuning_word=single.dds_tuning_word_for_frequency(45_000.0),
        amplitude=103,
        phase=0,
        fit=empty_circle_fit(),
        trace_fit=trace_fit,
    )

    assert trace_fit.temporal_overlap == pytest.approx(1.0)
    assert trace_fit.span_x_div > 6.0
    assert trace_fit.span_y_div > 6.0
    assert single.circle_sweep_result_is_ellipse_seed(
        result, single.DEFAULT_CONFIG, strong=True)


@pytest.mark.parametrize(("target", "expected_phase"), (
    (single.TARGET_DIAGONAL, 0),
    (single.TARGET_CIRCLE, 64),
    (single.TARGET_EIGHT, 0),
))
def test_all_targets_start_sweep_with_calibrated_target_payload(
    target: int,
    expected_phase: int,
) -> None:
    link = RecordingLink()
    controller = single.AutoLissajousController(
        controller_config(), link, FreshFrameCamera())
    controller._target = target
    controller._coarse_frequency_hz = 45_000.0

    controller._start_circle_sweep(1.0)

    frame = link.frames[-1]
    assert frame.command == single.CMD_TARGET
    assert frame.payload[0] == target
    assert frame.payload[1] == 103
    assert frame.payload[2] == expected_phase
    assert controller._amplitude == 103
    assert controller._phase == expected_phase


@pytest.mark.parametrize("target", (
    single.TARGET_DIAGONAL,
    single.TARGET_CIRCLE,
    single.TARGET_EIGHT,
))
def test_enter_locked_keeps_all_targets_in_continuous_maintenance(
    target: int,
) -> None:
    controller = prime_lock_controller(target)

    controller._enter_locked(10.0, 92)

    assert controller.mode == "CIRCLE_MAINTAIN_SETTLE"
    assert controller.active
    assert controller._circle_locked_announced
    assert controller._deadline > 10.0
    controller.poll(controller._deadline - 0.001)
    assert controller.mode == "CIRCLE_MAINTAIN_SETTLE"


def test_enter_locked_reports_status_locked_once() -> None:
    controller = prime_lock_controller(single.TARGET_CIRCLE)
    link = controller.link

    controller._enter_locked(10.0, 92)
    controller._enter_locked(11.0, 88)

    assert len(link.frames) == 1
    frame = link.frames[0]
    assert frame.command == single.STATUS_LOCKED
    assert frame.payload[0] == single.TARGET_CIRCLE
    assert frame.payload[1] == 92
    assert frame.payload[2] == controller._coarse_width_code
    assert int.from_bytes(frame.payload[4:8], "little") == 45_000_000


def test_new_start_and_cancel_exit_previous_continuous_maintenance() -> None:
    controller = prime_lock_controller(single.TARGET_CIRCLE)
    controller._enter_locked(10.0, 90)

    assert controller.start(single.TARGET_EIGHT, now=11.0)
    assert controller._target == single.TARGET_EIGHT
    assert not controller._circle_locked_announced
    assert controller.mode == "COARSE_SETTLE"

    controller._final_frequency_hz = 45_000.0
    controller._enter_locked(12.0, 88)
    controller.cancel()

    assert controller.mode == "IDLE"
    assert controller._target == 0
    assert not controller.active


def test_high_frequency_boundary_seed_expands_before_validation() -> None:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_DIAGONAL
    controller._coarse_frequency_hz = 90_000.0
    controller._start_circle_sweep(1.0)

    first_two_tiers = [
        frequency
        for tier in controller._circle_sweep_tiers[:2]
        for frequency in tier
    ]
    assert controller._circle_screen_step_hz == 100.0
    assert first_two_tiers == [
        90_000.0,
        90_100.0, 90_200.0, 90_300.0, 90_400.0, 90_500.0,
        90_600.0, 90_700.0, 90_800.0, 90_900.0, 91_000.0,
    ]

    seed_frequency = 90_500.0
    controller._circle_sweep_results = [single.CircleSweepResult(
        seed_frequency,
        single.dds_tuning_word_for_frequency(seed_frequency),
        103,
        0,
        empty_circle_fit(),
        strong_trace_fit(),
    )]
    controller._circle_tier_result_start = 0
    controller._finish_circle_screen_tier(2.0)

    assert controller._circle_sweep_stage == "SCREEN"
    assert controller._circle_sweep_tier_index == 1
    assert controller._circle_sweep_frequencies == [
        90_600.0, 90_700.0, 90_800.0, 90_900.0, 91_000.0,
    ]
    assert all(frequency % 100.0 == 0.0
               for frequency in controller._circle_sweep_frequencies)


def test_quick_target_screen_seed_accepts_diagonal_candidate(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    config = controller_config()
    config["target"]["tracking_lock"]["quick_lock_enabled"] = True
    controller = single.AutoLissajousController(
        config, RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_DIAGONAL
    controller._coarse_frequency_hz = 45_000.0
    controller._start_circle_sweep(1.0)
    controller.link.frames.clear()
    controller._circle_sweep_stage = "SCREEN"
    controller._circle_sweep_frequencies = [45_000.0]
    controller._circle_sweep_index = 0
    controller._circle_current_masks = [np.ones((32, 32), np.uint8)]

    monkeypatch.setattr(single.time, "monotonic", lambda: 2.0)
    monkeypatch.setattr(
        single, "aggregate_masks",
        lambda _masks: np.ones((32, 32), np.uint8),
    )
    monkeypatch.setattr(
        single,
        "analyze_frequency_trace_masks",
        lambda *_args, **_kwargs: strong_trace_fit(),
    )
    monkeypatch.setattr(
        single,
        "analyze_circle_lock_mask",
        lambda *_args, **_kwargs: empty_circle_fit(),
    )
    monkeypatch.setattr(
        controller.target_analyzer,
        "analyze",
        lambda _mask, _target: single.TargetFit(
            0, 0.018, 78, 7.2, 7.0, 0.0),
    )

    controller._finish_circle_sweep_candidate(2.0)

    assert controller._circle_frequency_verified is True
    assert controller._final_frequency_hz == pytest.approx(45_000.0)
    assert controller.mode == "CIRCLE_CONFIRM_SETTLE"
    assert controller.link.frames
    assert controller.link.frames[-1].command == single.CMD_TARGET


def test_eight_frequency_integral_uses_two_to_one_phase_ratio() -> None:
    corrections: dict[int, float] = {}
    for target in (single.TARGET_CIRCLE, single.TARGET_EIGHT):
        config = controller_config()
        config["target"]["circle_lock"].update({
            "frequency_integral_required_steps": 1,
            "frequency_integral_gain": 0.80,
            "frequency_maximum_step_hz": 1.0,
        })
        controller = single.AutoLissajousController(
            config, RecordingLink(), FreshFrameCamera())
        controller._target = target
        controller._circle_frequency_verified = True
        controller._coarse_frequency_hz = 50_000.0
        controller._final_frequency_hz = 50_000.0
        controller._circle_frequency_anchor_hz = 50_000.0
        controller._tuning_word = single.dds_tuning_word_for_frequency(50_000.0)
        controller._circle_last_servo_at = 1.0

        assert controller._integrate_circle_frequency(64, 2.0)
        corrections[target] = controller._final_frequency_hz - 50_000.0

    assert corrections[single.TARGET_CIRCLE] == pytest.approx(0.20)
    assert corrections[single.TARGET_EIGHT] == pytest.approx(0.10)
    assert corrections[single.TARGET_CIRCLE] == pytest.approx(
        2.0 * corrections[single.TARGET_EIGHT])
