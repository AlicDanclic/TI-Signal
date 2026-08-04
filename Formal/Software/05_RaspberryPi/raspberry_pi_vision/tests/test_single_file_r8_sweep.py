import copy
from dataclasses import replace
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

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


def make_stationary_trace(angle_degrees: float = 31.0) -> np.ndarray:
    mask = np.zeros((512, 640), np.uint8)
    radians = np.deg2rad(angle_degrees)
    dx = int(round(260 * np.cos(radians)))
    dy = int(round(220 * np.sin(radians)))
    cv2.line(
        mask,
        (320 - dx, 256 - dy),
        (320 + dx, 256 + dy),
        255,
        3,
        cv2.LINE_AA,
    )
    return mask


def strong_trace_fit(score: float = 0.90) -> single.FrequencyTraceFit:
    return single.FrequencyTraceFit(
        quality=int(round(score * 100)),
        score=score,
        thinness_quality=0.90,
        temporal_overlap=0.90,
        extent_quality=1.0,
        span_x_div=7.8,
        span_y_div=5.2,
        thickness_px=3.0,
        pixel_count=900,
        valid_frames=5,
    )


def weak_circle_fit() -> single.CircleLockFit:
    return single.CircleLockFit(
        quality=20,
        score=0.20,
        span_x_div=7.8,
        span_y_div=5.2,
        center_error_div=0.1,
        radial_cv=0.20,
        inner_fill_ratio=0.20,
        angular_coverage=0.70,
        fill_ratio=0.20,
        pixel_count=900,
    )


def strong_ellipse_fit() -> single.CircleLockFit:
    return single.CircleLockFit(
        quality=72,
        score=0.72,
        span_x_div=7.8,
        span_y_div=5.2,
        center_error_div=0.1,
        radial_cv=0.08,
        inner_fill_ratio=0.05,
        angular_coverage=0.82,
        fill_ratio=0.16,
        pixel_count=1200,
    )


def test_r8_frequency_tiers_add_only_the_requested_100hz_bands() -> None:
    tiers = single.circle_sweep_frequency_tiers(
        20_040.0,
        step_hz=100.0,
        minimum_hz=1_000.0,
        maximum_hz=100_000.0,
    )

    assert tiers == [
        [
            20_000.0,
            20_100.0, 19_900.0,
            20_200.0, 19_800.0,
            20_300.0, 19_700.0,
        ],
        [
            20_400.0, 19_600.0,
            20_500.0, 19_500.0,
            20_600.0, 19_400.0,
            20_700.0, 19_300.0,
            20_800.0, 19_200.0,
        ],
        [
            20_900.0, 19_100.0,
            21_000.0, 19_000.0,
            21_100.0, 18_900.0,
            21_200.0, 18_800.0,
            21_300.0, 18_700.0,
            21_400.0, 18_600.0,
            21_500.0, 18_500.0,
        ],
    ]
    flattened = [frequency for tier in tiers for frequency in tier]
    assert len(flattened) == len(set(flattened)) == 31
    assert all(frequency % 100.0 == 0.0 for frequency in flattened)


def test_r8_frequency_tiers_clip_at_contest_limits_without_duplicates() -> None:
    tiers = single.circle_sweep_frequency_tiers(
        1_000.0,
        step_hz=100.0,
        minimum_hz=1_000.0,
        maximum_hz=100_000.0,
    )

    assert tiers[0] == [1_000.0, 1_100.0, 1_200.0, 1_300.0]
    assert tiers[1] == [1_400.0, 1_500.0, 1_600.0, 1_700.0, 1_800.0]
    assert tiers[2] == [
        1_900.0, 2_000.0, 2_100.0, 2_200.0,
        2_300.0, 2_400.0, 2_500.0,
    ]
    flattened = [frequency for tier in tiers for frequency in tier]
    assert len(flattened) == len(set(flattened))


@pytest.mark.parametrize("coarse_hz,true_hz", (
    (89_000.0, 90_000.0),
    (89_200.0, 90_200.0),
))
def test_high_frequency_profile_covers_one_khz_error_on_100hz_grid(
        coarse_hz: float, true_hz: float) -> None:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._coarse_frequency_hz = coarse_hz

    controller._start_circle_sweep(1.0)

    assert controller._circle_sweep_tier_radii[-1] >= 2_500.0
    assert controller._circle_screen_step_hz == 100.0
    assert any(true_hz in tier for tier in controller._circle_sweep_tiers)
    assert all(frequency % 100.0 == 0.0
               for tier in controller._circle_sweep_tiers
               for frequency in tier)


def test_yaml_and_embedded_r9_sweep_defaults_are_identical() -> None:
    config_path = Path(__file__).resolve().parents[1] / "config.yaml"
    yaml_config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    yaml_sweep = yaml_config["target"]["circle_sweep"]
    embedded_sweep = single.DEFAULT_CONFIG["target"]["circle_sweep"]
    behavior_keys = (
        "step_hz",
        "tier_radii_hz",
        "high_frequency_threshold_hz",
        "high_frequency_tier_radii_hz",
        "high_frequency_positive_first",
        "screen_settle_s",
        "micro_settle_s",
        "validation_settle_s",
        "hold_settle_s",
        "screen_frames_per_candidate",
        "screen_minimum_frames",
        "screen_frame_intervals_s",
        "micro_offsets_hz",
        "micro_frames",
        "micro_minimum_frames",
        "micro_minimum_aggregate_pixels",
        "micro_frame_intervals_s",
        "validation_frames",
        "validation_minimum_frames",
        "validation_minimum_aggregate_pixels",
        "validation_frame_intervals_s",
        "validation_minimum_anchor_visits",
        "hold_frames",
        "hold_minimum_frames",
        "hold_minimum_aggregate_pixels",
        "hold_frame_intervals_s",
        "trace_minimum_pixels",
        "trace_minimum_frames",
        "trace_overlap_dilation_px",
    )

    assert {key: yaml_sweep[key] for key in behavior_keys} == {
        key: embedded_sweep[key] for key in behavior_keys
    }


def test_two_high_scoring_frames_cannot_early_accept_a_frequency() -> None:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._coarse_frequency_hz = 20_047.0
    controller._start_circle_sweep(1.0)
    trace = make_stationary_trace()
    controller._circle_current_masks = [trace, trace]

    controller._finish_circle_sweep_candidate(1.3)

    assert controller.mode != "CIRCLE_CONFIRM_SETTLE"
    assert controller._circle_sweep_stage in ("SCREEN", "VERIFY")
    assert controller._circle_frequency_verified is False


def test_verify_stage_collects_five_frames_before_scoring(
        monkeypatch) -> None:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._coarse_frequency_hz = 20_000.0
    controller._start_circle_sweep(1.0)
    controller._circle_sweep_stage = "VERIFY"
    trace = make_stationary_trace()
    monkeypatch.setattr(controller, "_read_target_mask", lambda: trace.copy())
    completed: list[tuple[int, int]] = []
    monkeypatch.setattr(
        controller,
        "_finish_circle_sweep_candidate",
        lambda _now: completed.append((
            controller._circle_capture_attempts,
            len(controller._circle_current_masks),
        )),
    )

    for attempt in range(4):
        controller._capture_circle_sweep_frame(2.0 + attempt * 0.07)
        assert completed == []

    controller._capture_circle_sweep_frame(2.30)

    assert completed == [(5, 5)]


def test_frequency_trace_validation_requires_at_least_three_of_five() -> None:
    config = controller_config()
    sweep = config["target"]["circle_sweep"]
    sweep["trace_minimum_pixels"] = 35
    sweep["trace_minimum_frames"] = 3
    trace = make_stationary_trace()
    missing = np.zeros_like(trace)

    fit = single.analyze_frequency_trace_masks(
        [trace, missing, trace, missing, trace], config)

    assert fit.valid_frames == 3
    assert fit.quality >= sweep["minimum_trace_quality"]
    with pytest.raises(ValueError, match=r"only 2/3 usable frequency frames"):
        single.analyze_frequency_trace_masks(
            [trace, missing, missing, trace, missing], config)


def test_screen_ranking_enters_validation_before_micro(
        monkeypatch) -> None:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._coarse_frequency_hz = 20_040.0
    controller._start_circle_sweep(1.0)
    circle_fit = weak_circle_fit()
    controller._circle_sweep_results = [
        single.CircleSweepResult(
            20_000.0,
            single.dds_tuning_word_for_frequency(20_000.0),
            103,
            64,
            circle_fit,
            strong_trace_fit(0.92),
        ),
        single.CircleSweepResult(
            20_100.0,
            single.dds_tuning_word_for_frequency(20_100.0),
            103,
            64,
            circle_fit,
            strong_trace_fit(0.76),
        ),
        single.CircleSweepResult(
            19_900.0,
            single.dds_tuning_word_for_frequency(19_900.0),
            103,
            64,
            circle_fit,
            strong_trace_fit(0.68),
        ),
    ]
    confirms: list[float] = []
    monkeypatch.setattr(
        controller,
        "_send_circle_confirm_target",
        lambda now: confirms.append(now),
    )

    controller._finish_circle_sweep(3.0)

    assert confirms == []
    assert controller._circle_sweep_stage == "VERIFY"
    assert controller._circle_frequency_verified is False
    assert controller.mode != "CIRCLE_CONFIRM_SETTLE"
    assert controller._circle_validation_anchor_hz == 20_000.0
    assert controller._circle_sweep_frequencies[0] == 20_000.0
    assert controller._circle_sweep_frequencies[-1] == 20_000.0


def test_successful_hold_is_the_only_gate_to_circle_confirmation(
        monkeypatch) -> None:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._coarse_frequency_hz = 20_040.0
    controller._start_circle_sweep(1.0)
    result = single.CircleSweepResult(
        20_000.0,
        single.dds_tuning_word_for_frequency(20_000.0),
        103,
        64,
        strong_ellipse_fit(),
        strong_trace_fit(0.90),
    )
    controller._circle_sweep_stage = "HOLD"
    controller._circle_hold_result = result
    confirms: list[float] = []
    monkeypatch.setattr(
        controller,
        "_send_circle_confirm_target",
        lambda now: confirms.append(now),
    )

    assert controller._circle_frequency_verified is False

    controller._finish_circle_hold(4.0)

    assert controller._circle_frequency_verified is True
    assert confirms == [4.0]


def test_next_candidate_settle_uses_post_opencv_monotonic_time(
        monkeypatch) -> None:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._coarse_frequency_hz = 20_000.0
    controller._start_circle_sweep(1.0)
    trace = make_stationary_trace()
    controller._circle_current_masks = [trace, trace, trace]
    monkeypatch.setattr(single.time, "monotonic", lambda: 10.0)

    controller._finish_circle_sweep_candidate(1.3)

    assert controller.mode == "CIRCLE_SWEEP_SETTLE"
    assert controller._deadline == pytest.approx(10.18)


def test_failed_frequency_is_not_selected_again_after_tier_expansion() -> None:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._coarse_frequency_hz = 20_000.0
    controller._start_circle_sweep(1.0)
    circle_fit = weak_circle_fit()
    controller._circle_sweep_results = [
        single.CircleSweepResult(
            20_000.0, 1, 103, 64, circle_fit, strong_trace_fit(0.92)),
        single.CircleSweepResult(
            20_100.0, 2, 103, 64, circle_fit, strong_trace_fit(0.82)),
        single.CircleSweepResult(
            19_900.0, 3, 103, 64, circle_fit, strong_trace_fit(0.72)),
    ]
    controller._circle_rejected_frequencies.add(20_000.0)

    controller._finish_circle_screen_tier(3.0)

    assert controller._circle_sweep_stage == "VERIFY"
    assert controller._circle_validation_anchor_hz == 20_100.0
    assert controller._circle_sweep_frequencies[0] == 20_100.0
    assert 20_000.0 not in controller._circle_sweep_frequencies


def test_target_extractor_keeps_40_to_149_pixel_green_trace() -> None:
    extractor = single.TraceExtractor({
        "vision": {
            "minimum_trace_pixels": 150,
            "brightness_threshold": 255,
        }
    })
    frame = np.zeros((80, 100, 3), np.uint8)
    frame[30:40, 45:55] = (0, 255, 0)

    default_mask = extractor.extract(frame)
    target_mask = extractor.extract(frame, minimum_color_pixels=40)

    assert cv2.countNonZero(default_mask) == 0
    assert cv2.countNonZero(target_mask) >= 40


def test_rejected_neighbour_cannot_win_validation_ranking() -> None:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._coarse_frequency_hz = 20_000.0
    controller._start_circle_sweep(1.0)
    fit = weak_circle_fit()
    controller._circle_sweep_stage = "VERIFY"
    controller._circle_validation_anchor_hz = 20_100.0
    controller._circle_rejected_frequencies.add(20_000.0)
    controller._circle_validation_results = [
        single.CircleSweepResult(
            20_000.0, 1, 103, 64, fit, strong_trace_fit(0.96)),
        single.CircleSweepResult(
            20_100.0, 2, 103, 64, fit, strong_trace_fit(0.82)),
        single.CircleSweepResult(
            20_100.0, 2, 103, 64, fit, strong_trace_fit(0.81)),
    ]

    controller._finish_circle_validation(4.0)

    assert controller._circle_sweep_stage == "MICRO"
    assert controller._circle_micro_seed_hz == 20_100.0
    assert controller._circle_sweep_frequencies[0] == 20_100.0
    assert controller._circle_validation_anchor_hz == 20_100.0


def test_verify_retries_empty_masks_until_maximum_attempts(
        monkeypatch) -> None:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._coarse_frequency_hz = 20_000.0
    controller._start_circle_sweep(1.0)
    controller._circle_sweep_stage = "VERIFY"
    empty = np.zeros((512, 640), np.uint8)
    monkeypatch.setattr(controller, "_read_target_mask", lambda: empty.copy())
    completed: list[tuple[int, int]] = []
    monkeypatch.setattr(
        controller,
        "_finish_circle_sweep_candidate",
        lambda _now: completed.append((
            controller._circle_capture_attempts,
            len(controller._circle_current_masks),
        )),
    )

    for attempt in range(7):
        controller._capture_circle_sweep_frame(2.0 + attempt * 0.1)
        assert completed == []

    controller._capture_circle_sweep_frame(2.7)

    assert completed == [(8, 0)]


def test_unstable_confirmation_trace_cannot_change_phase(
        monkeypatch) -> None:
    config = controller_config()
    config["target"]["circle_lock"].update({
        "frames_per_block": 1,
        "maximum_frame_attempts": 1,
    })
    config["target"]["circle_sweep"]["trace_minimum_frames"] = 1
    controller = single.AutoLissajousController(
        config, RecordingLink(), FreshFrameCamera())
    controller._mode = "CIRCLE_CONFIRM_CAPTURE"
    controller._circle_frequency_verified = True
    trace = make_stationary_trace()
    unstable_trace = replace(strong_trace_fit(), temporal_overlap=0.05)
    corrections: list[float] = []
    monkeypatch.setattr(controller, "_read_target_mask", lambda: trace.copy())
    monkeypatch.setattr(
        single, "analyze_circle_lock_mask", lambda *_args, **_kwargs: weak_circle_fit())
    monkeypatch.setattr(
        single, "analyze_frequency_trace_masks",
        lambda *_args, **_kwargs: unstable_trace)
    monkeypatch.setattr(
        controller,
        "_try_adjust_circle_target",
        lambda _mask, _fit, now: corrections.append(now) or True,
    )

    controller._capture_circle_confirm_frame(3.0)

    assert corrections == []
    assert controller._circle_confirm_passes == 0
