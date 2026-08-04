import ast
import copy
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

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


def target_fixture(name: str) -> np.ndarray:
    root = Path(single.__file__).resolve().parent
    frame = cv2.imread(str(
        root / "tests" / "fixtures" / "target_shapes" / f"{name}.png"))
    assert frame is not None
    return single.extract_target_trace_mask(frame, single.DEFAULT_CONFIG)


def field_grid_mask() -> np.ndarray:
    root = Path(single.__file__).resolve().parents[1]
    mask = cv2.imread(
        str(root / "diagnostics" / "20260802_user_no_circle" /
            "02_target_mask.png"),
        cv2.IMREAD_GRAYSCALE,
    )
    assert mask is not None
    return mask


def stable_trace_fit() -> single.FrequencyTraceFit:
    return single.FrequencyTraceFit(
        quality=88,
        score=0.88,
        thinness_quality=0.82,
        temporal_overlap=0.90,
        extent_quality=1.0,
        span_x_div=7.8,
        span_y_div=7.8,
        thickness_px=5.0,
        pixel_count=900,
        valid_frames=1,
        aggregate_pixel_count=900,
        total_frames=1,
    )


def circle_fit(*, axis_ratio: float = 1.05,
               fill_ratio: float = 0.12) -> single.CircleLockFit:
    return single.CircleLockFit(
        quality=86,
        score=0.86,
        span_x_div=7.8,
        span_y_div=7.8,
        center_error_div=0.1,
        radial_cv=0.08,
        inner_fill_ratio=0.10,
        angular_coverage=0.90,
        fill_ratio=fill_ratio,
        pixel_count=900,
        ellipse_axis_ratio=axis_ratio,
    )


@pytest.mark.parametrize("name", ("circle", "line", "eight"))
def test_artifact_gate_preserves_real_target_fixtures(name: str) -> None:
    mask = target_fixture(name)
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())

    assert not single.target_mask_has_grid_or_frame_artifact(
        mask, single.DEFAULT_CONFIG)
    assert controller._circle_capture_mask_is_usable(mask)


def test_artifact_gate_rejects_field_dense_grid_before_frequency_score() -> None:
    mask = field_grid_mask()
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())

    assert single.target_mask_foreground_occupancy(mask) > 0.25
    assert single.target_mask_axis_line_flags(
        mask, single.DEFAULT_CONFIG) == (True, True)
    assert single.target_mask_has_grid_or_frame_artifact(
        mask, single.DEFAULT_CONFIG)
    assert not controller._circle_capture_mask_is_usable(mask)
    # The pure scorer intentionally retains its old behavior. The shared
    # capture gate must stop this high-overlap mask before SCREEN/HOLD/CONFIRM
    # can consume it.
    misleading_fit = single.analyze_frequency_trace_masks(
        [mask, mask, mask], single.DEFAULT_CONFIG)
    assert misleading_fit.temporal_overlap > 0.90


def test_artifact_gate_rejects_three_side_crt_frame_and_dense_grid() -> None:
    frame = np.zeros((512, 640), np.uint8)
    cv2.line(frame, (48, 30), (48, 475), 255, 8)
    cv2.line(frame, (590, 30), (590, 475), 255, 8)
    cv2.line(frame, (48, 475), (590, 475), 255, 8)
    for x in range(80, 570, 24):
        for y in range(60, 450, 24):
            cv2.circle(frame, (x, y), 3, 255, -1)

    grid = np.zeros_like(frame)
    for x in range(45, 610, 18):
        cv2.line(grid, (x, 35), (x, 475), 255, 3)
    for y in range(35, 490, 18):
        cv2.line(grid, (30, y), (610, y), 255, 3)

    assert single.target_mask_has_grid_or_frame_artifact(
        frame, single.DEFAULT_CONFIG)
    assert single.target_mask_has_grid_or_frame_artifact(
        grid, single.DEFAULT_CONFIG)


@pytest.mark.parametrize("endpoints", (
    ((30, 256), (610, 256)),
    ((320, 20), (320, 492)),
    ((40, 470), (600, 40)),
))
def test_single_axis_or_diagonal_target_line_is_not_rejected(
    endpoints: tuple[tuple[int, int], tuple[int, int]],
) -> None:
    mask = np.zeros((512, 640), np.uint8)
    cv2.line(mask, endpoints[0], endpoints[1], 255, 8, cv2.LINE_AA)

    flags = single.target_mask_axis_line_flags(mask, single.DEFAULT_CONFIG)
    assert flags != (True, True)
    assert not single.target_mask_has_grid_or_frame_artifact(
        mask, single.DEFAULT_CONFIG)


def test_circle_touching_top_and_bottom_is_not_rejected() -> None:
    mask = np.zeros((512, 640), np.uint8)
    cv2.ellipse(
        mask,
        (320, 256),
        (245, 255),
        0.0,
        0.0,
        360.0,
        255,
        9,
        cv2.LINE_AA,
    )

    assert single.target_mask_axis_line_flags(
        mask, single.DEFAULT_CONFIG) != (True, True)
    assert not single.target_mask_has_grid_or_frame_artifact(
        mask, single.DEFAULT_CONFIG)


def test_final_circle_lock_has_independent_fill_guard() -> None:
    fit = circle_fit(fill_ratio=0.36)

    assert not single.circle_fit_is_locked(fit, single.DEFAULT_CONFIG)
    assert "FILL" in single.circle_fit_lock_failures(
        fit, single.DEFAULT_CONFIG)


def test_bad_block_resets_momentary_circle_passes(monkeypatch) -> None:
    config = controller_config()
    config["target"]["circle_sweep"]["trace_minimum_frames"] = 1
    config["target"]["circle_lock"].update({
        "frames_per_block": 1,
        "maximum_frame_attempts": 1,
        "required_passes": 5,
    })
    controller = single.AutoLissajousController(
        config, RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._mode = "CIRCLE_CONFIRM_CAPTURE"
    controller._circle_frequency_verified = True
    mask = target_fixture("circle")
    fits = iter((circle_fit(), circle_fit(axis_ratio=2.0)))
    monkeypatch.setattr(controller, "_read_target_mask", lambda: mask)
    monkeypatch.setattr(
        controller, "_circle_capture_mask_is_usable", lambda _mask: True)
    monkeypatch.setattr(
        single, "analyze_frequency_trace_masks",
        lambda _masks, _config: stable_trace_fit())
    monkeypatch.setattr(
        single, "analyze_circle_lock_mask", lambda _mask, _config: next(fits))
    monkeypatch.setattr(
        controller.target_analyzer,
        "analyze",
        lambda _mask, _target: single.TargetFit(
            64, 0.01, 90, 7.8, 7.8, 0.1, 0.01),
    )
    monkeypatch.setattr(
        controller,
        "_observe_circle_frequency_drift",
        lambda _fit, _now: single.CIRCLE_DRIFT_READY,
    )
    monkeypatch.setattr(
        controller,
        "_try_adjust_circle_target",
        lambda *_args: single.CIRCLE_ADJUST_NONE,
    )

    controller._capture_circle_confirm_frame(1.0)
    assert controller._circle_confirm_passes == 1

    controller._capture_circle_confirm_frame(2.0)
    assert controller._circle_confirm_passes == 0
    assert not controller._circle_locked_announced


def _definitions(path: Path) -> dict[str, str]:
    source = path.read_text(encoding="utf-8")
    tree = ast.parse(source)
    lines = source.splitlines()
    return {
        node.name: "\n".join(lines[node.lineno - 1:node.end_lineno])
        for node in ast.walk(tree)
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef))
    }


def test_r16_keeps_frequency_measurement_code_identical_to_r15() -> None:
    current = Path(single.__file__).resolve()
    backup = current.parent / "versions" / (
        "20260802_cv_r15_before_target_artifact_fix") / "task5_cv_single.py"
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


def test_r20_keeps_legacy_five_block_gate_available() -> None:
    assert single.TASK5_CV_BUILD_TAG == "CV-R20"
    assert single.DEFAULT_CONFIG["target"]["circle_lock"][
        "required_passes"] >= 5


def test_high_frequency_screen_searches_every_100hz_residue() -> None:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._coarse_frequency_hz = 89_100.0

    controller._start_circle_sweep(1.0)

    assert controller._circle_screen_step_hz == 100.0
    assert controller._circle_sweep_tier_radii == [
        500.0, 1000.0, 1500.0, 2500.0]
    assert any(90_200.0 in tier for tier in controller._circle_sweep_tiers)
    assert all(
        frequency % 100.0 == 0.0
        for tier in controller._circle_sweep_tiers
        for frequency in tier
    )


def test_high_frequency_screen_uses_longer_capture_profile() -> None:
    config = controller_config()
    controller = single.AutoLissajousController(
        config, RecordingLink(), FreshFrameCamera())
    controller._coarse_frequency_hz = 90_000.0
    controller._circle_sweep_stage = "SCREEN"

    required, minimum, maximum, aggregate, intervals = (
        controller._circle_stage_parameters())

    assert (required, minimum, maximum, aggregate) == (4, 3, 7, 100)
    assert intervals == pytest.approx([0.071, 0.113, 0.089])


def test_yaml_cannot_restore_old_sparse_grid_or_three_pass_lock() -> None:
    config_path = Path(single.__file__).resolve().parent / "config.yaml"
    yaml_config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    embedded_sweep = single.DEFAULT_CONFIG["target"]["circle_sweep"]
    yaml_sweep = yaml_config["target"]["circle_sweep"]
    sweep_keys = (
        "high_frequency_screen_step_hz",
        "high_frequency_tier_radii_hz",
        "high_frequency_screen_settle_s",
        "high_frequency_screen_frames_per_candidate",
        "high_frequency_screen_minimum_frames",
        "high_frequency_screen_maximum_frame_attempts",
        "high_frequency_screen_minimum_aggregate_pixels",
        "high_frequency_screen_frame_intervals_s",
        "artifact_maximum_bbox_occupancy",
        "artifact_minimum_bidirectional_span_fraction",
        "artifact_axis_line_minimum_span_fraction",
    )

    assert {key: yaml_sweep[key] for key in sweep_keys} == {
        key: embedded_sweep[key] for key in sweep_keys}
    assert yaml_config["target"]["circle_lock"]["required_passes"] == 5
    assert yaml_config["target"]["circle_lock"]["maximum_fill_ratio"] == 0.35
