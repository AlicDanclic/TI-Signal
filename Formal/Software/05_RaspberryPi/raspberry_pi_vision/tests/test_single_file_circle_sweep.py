import copy
from dataclasses import replace

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


class RawOnlyCamera:
    def __init__(self) -> None:
        self.raw = np.zeros((720, 1280, 3), np.uint8)
        self.read_raw_calls = 0

    def read_raw(self) -> np.ndarray:
        self.read_raw_calls += 1
        return self.raw

    def read(self) -> np.ndarray:
        raise AssertionError("target capture must not use the legacy camera ROI")


def controller_config() -> dict:
    return copy.deepcopy(single.DEFAULT_CONFIG)


def make_ring_mask(*, axes: tuple[int, int] = (250, 230),
                   filled: bool = False) -> np.ndarray:
    mask = np.zeros((480, 640), np.uint8)
    cv2.ellipse(
        mask,
        (320, 240),
        axes,
        0,
        0,
        360,
        255,
        -1 if filled else 4,
        cv2.LINE_AA,
    )
    return mask


def make_reflection_corrupted_mask() -> np.ndarray:
    mask = make_ring_mask()
    cv2.rectangle(mask, (360, 80), (620, 330), 255, -1)
    return mask


def usable_fit(score: float, quality: int) -> single.CircleLockFit:
    return single.CircleLockFit(
        quality=quality,
        score=score,
        span_x_div=7.8,
        span_y_div=7.7,
        center_error_div=0.1,
        radial_cv=0.04,
        inner_fill_ratio=0.02,
        angular_coverage=0.95,
        fill_ratio=0.12,
        pixel_count=2000,
    )


def trace_fit(score: float, quality: int) -> single.FrequencyTraceFit:
    return single.FrequencyTraceFit(
        quality=quality,
        score=score,
        thinness_quality=score,
        temporal_overlap=score,
        extent_quality=1.0,
        span_x_div=7.8,
        span_y_div=5.2,
        thickness_px=4.0,
        pixel_count=1200,
        valid_frames=3,
    )


def make_trace_mask(*, angle_degrees: float = 0.0,
                    filled: bool = False) -> np.ndarray:
    mask = np.zeros((512, 640), np.uint8)
    if filled:
        cv2.circle(mask, (320, 256), 160, 255, -1, cv2.LINE_AA)
        return mask
    radians = np.deg2rad(angle_degrees)
    dx = int(round(260 * np.cos(radians)))
    dy = int(round(260 * np.sin(radians)))
    cv2.line(
        mask,
        (320 - dx, 256 - dy),
        (320 + dx, 256 + dy),
        255,
        3,
        cv2.LINE_AA,
    )
    return mask


def make_circle_phase_mask(phase: int) -> np.ndarray:
    mask = np.zeros((512, 640), np.uint8)
    points = single.TargetAnalyzer._model_points(
        single.TARGET_CIRCLE,
        phase & 0xFF,
        320.0,
        256.0,
        230.0,
        205.0,
    )
    cv2.polylines(mask, [points], True, 255, 3, cv2.LINE_AA)
    return mask


@pytest.mark.parametrize(
    ("measured_hz", "expected_hz"),
    (
        (1049.999, 1000.0),
        (1050.0, 1100.0),
        (99_949.999, 99_900.0),
        (99_950.0, 100_000.0),
        (0.0, 0.0),
        (float("nan"), 0.0),
    ),
)
def test_control_frequency_uses_half_up_100hz_grid(
        measured_hz: float, expected_hz: float) -> None:
    assert single.quantize_control_frequency_hz(measured_hz) == expected_hz


def test_circle_candidates_are_center_out_and_stay_in_contest_range() -> None:
    assert single.circle_sweep_frequency_candidates(
        10_040.0, radius_hz=300.0, step_hz=100.0) == [
            10_000.0,
            10_100.0,
            9_900.0,
            10_200.0,
            9_800.0,
            10_300.0,
            9_700.0,
        ]
    assert single.circle_sweep_frequency_candidates(
        1_000.0, radius_hz=200.0, step_hz=100.0) == [
            1_000.0,
            1_100.0,
            1_200.0,
        ]
    assert single.circle_sweep_frequency_candidates(
        100_000.0, radius_hz=200.0, step_hz=100.0) == [
            100_000.0,
            99_900.0,
            99_800.0,
        ]


def test_default_circle_sweep_covers_plus_minus_1p5khz() -> None:
    sweep = controller_config()["target"]["circle_sweep"]

    candidates = single.circle_sweep_frequency_candidates(
        46_500.0,
        radius_hz=sweep["radius_hz"],
        step_hz=sweep["step_hz"],
        minimum_hz=sweep["minimum_hz"],
        maximum_hz=sweep["maximum_hz"],
    )

    assert len(candidates) == 31
    assert candidates[:7] == [
        46_500.0,
        46_600.0,
        46_400.0,
        46_700.0,
        46_300.0,
        46_800.0,
        46_200.0,
    ]
    assert candidates[-2:] == [48_000.0, 45_000.0]


def test_target_capture_uses_raw_frame_and_target_screen_rectification(
        monkeypatch) -> None:
    camera = RawOnlyCamera()
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), camera)
    rectified = np.zeros((512, 640, 3), np.uint8)
    expected_corners = np.array(
        [[10.0, 20.0], [1200.0, 20.0], [1200.0, 690.0], [10.0, 690.0]],
        np.float32,
    )
    calls: list[tuple[np.ndarray, np.ndarray, tuple[int, int]]] = []

    monkeypatch.setattr(
        single,
        "get_target_screen_corners",
        lambda frame, _config: expected_corners if frame is camera.raw else None,
    )

    def rectify(frame: np.ndarray, corners: np.ndarray,
                size: tuple[int, int]) -> np.ndarray:
        calls.append((frame, corners, size))
        return rectified

    monkeypatch.setattr(single, "rectify_screen", rectify)
    monkeypatch.setattr(
        single,
        "extract_target_trace_mask",
        lambda frame, _config: np.zeros(frame.shape[:2], np.uint8),
    )

    mask = controller._read_target_mask()

    assert camera.read_raw_calls == 1
    assert len(calls) == 1
    assert calls[0][0] is camera.raw
    assert calls[0][1] is expected_corners
    assert calls[0][2] == (640, 512)
    assert mask.shape == (512, 640)


def test_circle_sweep_target_payload_uses_calibrated_circle_parameters() -> None:
    link = RecordingLink()
    controller = single.AutoLissajousController(
        controller_config(), link, FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._coarse_frequency_hz = 10_040.0

    controller._start_circle_sweep(1.0)

    assert controller.mode == "CIRCLE_SWEEP_SETTLE"
    assert controller._deadline == pytest.approx(1.18)
    assert len(link.frames) == 1
    frame = link.frames[0]
    expected_frequency_hz = 10_000.0
    expected_tw = single.dds_tuning_word_for_frequency(expected_frequency_hz)
    assert frame.command == single.CMD_TARGET
    assert frame.flags == 0
    assert frame.payload[:4] == bytes((single.TARGET_CIRCLE, 103, 64, 0))
    assert int.from_bytes(frame.payload[4:8], "little") == expected_tw
    assert expected_tw == round(expected_frequency_hz * 2**32 / 50_000_000)


def test_circle_coarse_completion_enters_sweep_without_quantizing_raw_fit(
        monkeypatch) -> None:
    link = RecordingLink()
    controller = single.AutoLissajousController(
        controller_config(), link, FreshFrameCamera())
    controller._coarse_width_codes = (0,)
    raw_frequency_hz = 20_047.25
    summary = single.CoarseMeasurement(
        accepted=True,
        frequency_hz=raw_frequency_hz,
        period_cv=0.01,
        valid_frame_ratio=1.0,
        median_point_count=8,
        complete_period_count=6,
        confidence=0.90,
        reason="OK",
    )
    controller._coarse_summary = lambda: summary
    sweep_starts: list[float] = []

    def start_sweep(now: float) -> None:
        sweep_starts.append(now)
        controller._mode = "CIRCLE_SWEEP_SETTLE"

    monkeypatch.setattr(controller, "_start_circle_sweep", start_sweep)
    assert controller.start(single.TARGET_CIRCLE, 1.0)

    controller._finish_coarse(2.0)

    assert sweep_starts == [2.0]
    assert controller.mode == "CIRCLE_SWEEP_SETTLE"
    assert controller._coarse_frequency_hz == raw_frequency_hz


def test_thin_circle_scores_above_filled_disk_and_wide_ellipse() -> None:
    config = controller_config()
    circle = single.analyze_circle_lock_mask(make_ring_mask(), config)
    disk = single.analyze_circle_lock_mask(
        make_ring_mask(filled=True), config)
    wide = single.analyze_circle_lock_mask(
        make_ring_mask(axes=(250, 155)), config)

    assert circle.quality > wide.quality > disk.quality
    assert circle.radial_cv < disk.radial_cv
    assert circle.inner_fill_ratio < disk.inner_fill_ratio
    assert single.circle_fit_is_locked(circle, config)
    assert not single.circle_fit_is_locked(wide, config)
    assert not single.circle_fit_is_locked(disk, config)


def test_stationary_trace_scores_above_rotation_and_filled_region() -> None:
    config = controller_config()
    ellipse = np.zeros((512, 640), np.uint8)
    cv2.ellipse(
        ellipse, (320, 256), (230, 150), 0, 0, 360,
        255, 3, cv2.LINE_AA)
    line = make_trace_mask(angle_degrees=32.0)
    rotating = [
        make_trace_mask(angle_degrees=angle)
        for angle in (0.0, 60.0, 120.0)
    ]
    disk = make_trace_mask(filled=True)

    stable_ellipse = single.analyze_frequency_trace_masks(
        [ellipse, ellipse, ellipse], config)
    stable_line = single.analyze_frequency_trace_masks(
        [line, line, line], config)
    rotating_fit = single.analyze_frequency_trace_masks(rotating, config)
    filled_fit = single.analyze_frequency_trace_masks(
        [disk, disk, disk], config)

    threshold = config["target"]["circle_sweep"]["minimum_trace_quality"]
    assert stable_ellipse.quality >= threshold
    assert stable_line.quality >= threshold
    assert stable_line.score > rotating_fit.score
    assert stable_ellipse.score > filled_fit.score


def test_fragmented_stationary_trace_keeps_all_meaningful_arcs() -> None:
    config = controller_config()
    fragmented = make_trace_mask(angle_degrees=31.0)
    for column in range(110, 560, 60):
        fragmented[:, column:column + 15] = 0

    component_count, _, _, _ = cv2.connectedComponentsWithStats(
        (fragmented > 0).astype(np.uint8), connectivity=8)
    fit = single.analyze_frequency_trace_masks(
        [fragmented, fragmented], config)

    assert component_count >= 6
    assert fit.valid_frames == 2
    assert fit.quality >= config["target"]["circle_sweep"][
        "minimum_trace_quality"]
    assert fit.span_x_div > 6.5


def test_strong_two_frame_trace_does_not_early_accept() -> None:
    config = controller_config()
    link = RecordingLink()
    controller = single.AutoLissajousController(
        config, link, FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._coarse_frequency_hz = 20_047.0
    controller._start_circle_sweep(1.0)
    line = make_trace_mask(angle_degrees=31.0)
    controller._circle_current_masks = [line, line]

    controller._finish_circle_sweep_candidate(1.3)

    assert len(controller._circle_sweep_frequencies) == 7
    assert controller._circle_sweep_index == 1
    assert controller._circle_sweep_verified is False
    assert controller.mode == "CIRCLE_SWEEP_SETTLE"
    assert [frame.command for frame in link.frames] == [
        single.CMD_TARGET,
        single.CMD_TARGET,
    ]


def test_tiered_sweep_reaches_one_khz_error_on_100hz_grid() -> None:
    tiers = single.circle_sweep_frequency_tiers(19_000.0)

    assert [len(tier) for tier in tiers] == [7, 10, 14]
    assert 20_000.0 in tiers[2]
    assert all(frequency % 100.0 == 0.0
               for tier in tiers for frequency in tier)


def test_trace_selector_chooses_unique_stationary_frequency() -> None:
    config = controller_config()
    base_fit = usable_fit(0.25, 25)
    results = [
        single.CircleSweepResult(
            46_500.0, 1, 103, 64, base_fit, trace_fit(0.78, 78)),
        single.CircleSweepResult(
            46_600.0, 2, 103, 64, base_fit, trace_fit(0.54, 54)),
        single.CircleSweepResult(
            46_400.0, 3, 103, 64, base_fit, trace_fit(0.61, 61)),
    ]

    selected = single.select_circle_sweep_result(
        results, 46_497.345, config)

    assert selected is results[0]


def test_trace_selector_rejects_best_result_at_sweep_boundary() -> None:
    config = controller_config()
    base_fit = usable_fit(0.25, 25)
    results = [
        single.CircleSweepResult(
            48_000.0, 1, 103, 64, base_fit, trace_fit(0.82, 82)),
        single.CircleSweepResult(
            46_500.0, 2, 103, 64, base_fit, trace_fit(0.66, 66)),
    ]

    assert single.select_circle_sweep_result(
        results, 46_497.345, config) is None


def test_circle_sweep_requires_one_clear_best_candidate() -> None:
    config = controller_config()
    config["target"]["circle_sweep"]["minimum_score_margin"] = 0.025
    first = single.CircleSweepResult(10_000.0, 1, 103, 64,
                                     usable_fit(0.82, 82))
    ambiguous = single.CircleSweepResult(10_100.0, 2, 103, 64,
                                         usable_fit(0.81, 81))
    clearly_worse = single.CircleSweepResult(9_900.0, 3, 103, 64,
                                             usable_fit(0.70, 70))

    assert single.select_circle_sweep_result(
        [first, ambiguous], 10_030.0, config) is None
    assert single.select_circle_sweep_result(
        [first, clearly_worse], 10_030.0, config) == first
    assert single.select_circle_sweep_result([
        replace(first, fit=usable_fit(0.30, 30)),
    ], 10_030.0, config) is None


@pytest.mark.parametrize(
    ("settle_mode", "capture_mode", "capture_method"),
    (
        ("CIRCLE_SWEEP_SETTLE", "CIRCLE_SWEEP_CAPTURE",
         "_capture_circle_sweep_frame"),
        ("CIRCLE_CONFIRM_SETTLE", "CIRCLE_CONFIRM_CAPTURE",
         "_capture_circle_confirm_frame"),
    ),
)
def test_poll_drives_all_four_circle_states_and_flushes_old_frames(
        monkeypatch, settle_mode: str, capture_mode: str,
        capture_method: str) -> None:
    camera = FreshFrameCamera()
    config = controller_config()
    config["target"]["control_timeout_s"] = 99.0
    controller = single.AutoLissajousController(
        config, RecordingLink(), camera)
    controller._mode = settle_mode
    controller._run_started = 0.0
    controller._deadline = 1.0
    captures: list[float] = []
    monkeypatch.setattr(
        controller, capture_method, lambda now: captures.append(now))

    controller.poll(1.0)

    assert controller.mode == capture_mode
    assert camera.required_after == [1.0]

    controller.poll(1.01)

    assert captures == [1.01]


def test_confirmation_counter_advances_once_per_complete_block(
        monkeypatch) -> None:
    config = controller_config()
    config["target"]["circle_lock"].update({
        "frames_per_block": 3,
        "maximum_frame_attempts": 3,
        "required_passes": 3,
        "maximum_blocks": 8,
        "amplitude_deadband_div": 1.0,
    })
    controller = single.AutoLissajousController(
        config, RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._mode = "CIRCLE_CONFIRM_CAPTURE"
    controller._circle_frequency_verified = True
    monkeypatch.setattr(controller, "_read_target_mask", make_ring_mask)

    controller._capture_circle_confirm_frame(1.0)
    controller._capture_circle_confirm_frame(1.1)

    assert controller._circle_confirm_blocks == 0
    assert controller._circle_confirm_passes == 0

    controller._capture_circle_confirm_frame(1.2)

    assert controller._circle_confirm_blocks == 1
    assert controller._circle_confirm_passes == 1


def test_reflection_block_is_skipped_then_three_good_blocks_lock(
        monkeypatch) -> None:
    config = controller_config()
    config["target"]["circle_lock"].update({
        "frames_per_block": 1,
        "maximum_frame_attempts": 1,
        "required_passes": 3,
        "maximum_blocks": 4,
    })
    config["target"]["circle_sweep"]["trace_minimum_frames"] = 1
    controller = single.AutoLissajousController(
        config, RecordingLink(), FreshFrameCamera())
    controller._mode = "CIRCLE_CONFIRM_CAPTURE"
    controller._target = single.TARGET_CIRCLE
    controller._circle_frequency_verified = True
    masks = iter((
        make_reflection_corrupted_mask(),
        make_ring_mask(),
        make_ring_mask(),
        make_ring_mask(),
    ))
    monkeypatch.setattr(controller, "_read_target_mask", lambda: next(masks))

    controller._capture_circle_confirm_frame(1.0)

    assert controller._circle_confirm_blocks == 1
    assert controller._circle_confirm_passes == 0
    assert controller.mode == "CIRCLE_CONFIRM_CAPTURE"

    controller._capture_circle_confirm_frame(1.1)
    controller._capture_circle_confirm_frame(1.2)
    controller._capture_circle_confirm_frame(1.3)

    assert controller._circle_confirm_blocks == 4
    assert controller._circle_confirm_passes == 3
    assert controller._circle_locked_announced is True
    assert controller.mode == "CIRCLE_MAINTAIN_SETTLE"


def test_repeated_bad_frames_end_one_skipped_block_instead_of_hanging(
        monkeypatch) -> None:
    config = controller_config()
    config["target"]["circle_lock"].update({
        "frames_per_block": 2,
        "maximum_frame_attempts": 2,
        "maximum_blocks": 8,
    })
    controller = single.AutoLissajousController(
        config, RecordingLink(), FreshFrameCamera())
    controller._mode = "CIRCLE_CONFIRM_CAPTURE"

    def reject_frame() -> np.ndarray:
        raise ValueError("reflection obscured the trace")

    monkeypatch.setattr(controller, "_read_target_mask", reject_frame)

    controller._capture_circle_confirm_frame(1.0)
    assert controller._circle_confirm_blocks == 0

    controller._capture_circle_confirm_frame(1.1)
    assert controller._circle_confirm_blocks == 1
    assert controller._circle_confirm_passes == 0
    assert controller.mode == "CIRCLE_CONFIRM_CAPTURE"


def test_stable_trace_without_circle_fit_does_not_invoke_phase_correction(
        monkeypatch) -> None:
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
    line = make_trace_mask(angle_degrees=0.0)
    corrections: list[tuple[np.ndarray, single.CircleLockFit | None, float]] = []

    monkeypatch.setattr(controller, "_read_target_mask", lambda: line)

    def correct(mask: np.ndarray, fit: single.CircleLockFit | None,
                now: float) -> bool:
        corrections.append((mask, fit, now))
        return True

    monkeypatch.setattr(controller, "_try_adjust_circle_target", correct)

    controller._capture_circle_confirm_frame(2.5)

    assert corrections == []
    assert controller._circle_confirm_passes == 0
    assert controller.mode == "CIRCLE_CONFIRM_CAPTURE"


def test_circle_correction_changes_phase_before_amplitude(monkeypatch) -> None:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._tuning_word = 123
    controller._amplitude = 100
    controller._phase = 0
    fit = replace(usable_fit(0.5, 50), span_x_div=8.0, span_y_div=4.0)
    mask = make_trace_mask(angle_degrees=30.0)
    sent: list[float] = []
    monkeypatch.setattr(
        controller.target_analyzer,
        "analyze",
        lambda _mask, _target: single.TargetFit(0, 0.2, 50, 8.0, 4.0, 0.0),
    )
    monkeypatch.setattr(
        controller,
        "_send_circle_confirm_target",
        lambda now: sent.append(now),
    )
    monkeypatch.setattr(single.time, "monotonic", lambda: 3.1)

    assert controller._try_adjust_circle_target(mask, fit, 3.0)

    maximum_step = controller.config["target"]["circle_lock"][
        "phase_maximum_step"]
    assert controller._phase == maximum_step
    assert controller._circle_phase_trial_baseline == 0
    assert controller._circle_phase_trial_stage == 1
    assert controller._amplitude == 100
    assert sent == [3.1]


def test_circle_phase_mirror_ambiguity_compares_symmetric_ab_trials(
        monkeypatch) -> None:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._tuning_word = 123
    controller._amplitude = 103
    controller._phase = 64
    mask = make_trace_mask(angle_degrees=30.0)
    sent_phases: list[int] = []
    monkeypatch.setattr(
        controller.target_analyzer,
        "analyze",
        lambda _mask, _target: single.TargetFit(
            0, 0.2, 50, 7.8, 5.2, 0.0),
    )
    monkeypatch.setattr(
        controller,
        "_send_circle_confirm_target",
        lambda _now: sent_phases.append(controller._phase),
    )

    baseline_fit = usable_fit(0.50, 50)
    worse_a_fit = usable_fit(0.35, 35)
    better_b_fit = usable_fit(0.80, 80)

    assert controller._try_adjust_circle_target(mask, baseline_fit, 1.0)
    baseline = 64
    trial_step = controller._circle_phase_trial_delta
    assert 0 < trial_step <= controller.config["target"]["circle_lock"][
        "phase_maximum_step"]
    assert controller._phase == (baseline + trial_step) & 0xFF

    assert controller._try_adjust_circle_target(mask, worse_a_fit, 2.0)
    assert controller._circle_phase_trial_stage == 2
    assert controller._phase == (baseline - trial_step) & 0xFF

    assert controller._try_adjust_circle_target(
        mask, better_b_fit, 3.0) == single.CIRCLE_ADJUST_SENT
    assert controller._phase == (baseline - trial_step) & 0xFF
    assert controller._circle_phase_trial_stage == 0
    assert sent_phases == [
        (baseline + trial_step) & 0xFF,
        (baseline - trial_step) & 0xFF,
        (baseline - trial_step) & 0xFF,
    ]


@pytest.mark.parametrize(
    ("estimated_phase", "expected_direction"),
    ((0, 1), (128, -1)),
)
def test_circle_phase_trial_is_bounded_in_both_directions(
        monkeypatch, estimated_phase: int, expected_direction: int) -> None:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._tuning_word = 123
    controller._amplitude = 103
    controller._phase = 64
    mask = make_trace_mask(angle_degrees=30.0)
    monkeypatch.setattr(
        controller.target_analyzer,
        "analyze",
        lambda _mask, _target: single.TargetFit(
            estimated_phase, 0.2, 50, 7.8, 5.2, 0.0),
    )
    monkeypatch.setattr(
        controller, "_send_circle_confirm_target", lambda _now: None)

    assert controller._try_adjust_circle_target(
        mask, usable_fit(0.50, 50), 1.0)

    applied = ((controller._phase - 64 + 128) & 0xFF) - 128
    assert 0 < abs(applied) <= controller.config["target"]["circle_lock"][
        "phase_maximum_step"]
    assert applied * expected_direction > 0


def test_circle_correction_changes_amplitude_after_phase_is_aligned(
        monkeypatch) -> None:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._tuning_word = 123
    controller._amplitude = 100
    controller._phase = 64
    fit = replace(usable_fit(0.5, 50), span_x_div=8.0, span_y_div=4.0)
    mask = make_trace_mask(angle_degrees=90.0)
    monkeypatch.setattr(
        controller.target_analyzer,
        "analyze",
        lambda _mask, _target: single.TargetFit(64, 0.2, 50, 8.0, 4.0, 0.0),
    )
    monkeypatch.setattr(
        controller, "_send_circle_confirm_target", lambda _now: None)

    assert controller._try_adjust_circle_target(mask, fit, 4.0)

    assert controller._phase == 64
    assert controller._amplitude == 103


def test_frequency_preview_uses_one_decimal_khz(monkeypatch) -> None:
    texts: list[str] = []
    original_put_text = cv2.putText

    def record_text(image: np.ndarray, text: str, *args, **kwargs):
        texts.append(text)
        return original_put_text(image, text, *args, **kwargs)

    monkeypatch.setattr(cv2, "putText", record_text)
    references = single.ReferenceLines(
        top_y=100.0,
        bottom_y=410.0,
        top_curve=(0.0, 0.0, 100.0),
        bottom_curve=(0.0, 0.0, 410.0),
        curve_center_x=320.0,
        curve_scale_x=300.0,
        top_band=(98, 102),
        bottom_band=(408, 412),
        left_x=50.0,
        right_x=590.0,
        confidence=1.0,
    )
    points = [
        single.WavePoint(
            100.0 + index * 80.0,
            180.0 + index * 20.0,
            0.1 * index,
            0.1 * index,
            0.0,
            0.1 * index,
            1.0,
        )
        for index in range(5)
    ]

    single.draw_result(
        np.zeros((512, 640, 3), np.uint8),
        references,
        points,
        avg_interval=0.2,
        interval_std=0.01,
        valid_count=3,
        freq_hz=46_497.345,
        ramp_duration_us=100.0,
        width_code=0,
    )

    assert "FREQ 46.5 kHz" in texts
    assert all("46.497" not in text for text in texts)


def test_legacy_stable_state_resumes_continuous_maintenance() -> None:
    config = controller_config()
    config["target"].update({
        "control_timeout_s": 1.0,
        "stability_seconds": 5.0,
    })
    controller = single.AutoLissajousController(
        config, RecordingLink(), FreshFrameCamera())
    controller._mode = "STABLE"
    controller._run_started = 0.0
    controller._stable_since = 0.0

    controller.poll(2.0)
    assert controller.mode == "CIRCLE_MAINTAIN_SETTLE"
    assert controller.active is True
