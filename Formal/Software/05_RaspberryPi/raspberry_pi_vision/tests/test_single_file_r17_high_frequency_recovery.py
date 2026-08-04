import ast
import copy
from pathlib import Path

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


def circle_fit() -> single.CircleLockFit:
    return single.CircleLockFit(
        quality=90,
        score=0.90,
        span_x_div=7.2,
        span_y_div=7.3,
        center_error_div=0.1,
        radial_cv=0.03,
        inner_fill_ratio=0.02,
        angular_coverage=0.95,
        fill_ratio=0.10,
        pixel_count=18_000,
        ellipse_axis_ratio=1.05,
    )


def trace_fit(
    *,
    pixel_count: int = 18_000,
    aggregate_pixel_count: int = 20_000,
) -> single.FrequencyTraceFit:
    return single.FrequencyTraceFit(
        quality=88,
        score=0.88,
        thinness_quality=0.75,
        temporal_overlap=0.90,
        extent_quality=1.0,
        span_x_div=7.2,
        span_y_div=7.3,
        thickness_px=7.0,
        pixel_count=pixel_count,
        valid_frames=4,
        aggregate_pixel_count=aggregate_pixel_count,
        total_frames=4,
    )


def sweep_result(
    frequency_hz: float,
    *,
    occupancy: float,
    trace: single.FrequencyTraceFit | None = None,
) -> single.CircleSweepResult:
    return single.CircleSweepResult(
        frequency_hz,
        single.dds_tuning_word_for_frequency(frequency_hz),
        103,
        64,
        circle_fit(),
        trace or trace_fit(),
        single.TargetFit(64, 0.01, 90, 7.2, 7.3, 0.1, 0.01),
        occupancy,
    )


def test_r17_guard_replaces_old_500hz_yaml_values() -> None:
    config = controller_config()
    sweep = config["target"]["circle_sweep"]
    circle = config["target"]["circle_lock"]
    sweep.update({
        "high_frequency_positive_first": False,
        "high_frequency_screen_step_hz": 500.0,
        "high_frequency_tier_radii_hz": [500.0, 1000.0],
        "high_frequency_screen_frames_per_candidate": 3,
        "high_frequency_seed_maximum_foreground_occupancy": 0.30,
        "high_frequency_seed_minimum_union_stability": 0.10,
    })
    circle.update({
        "required_passes": 3,
        "high_frequency_confirmation_no_pass_blocks": 80,
        "maximum_fill_ratio": 0.90,
    })
    config["target"]["control_timeout_s"] = 20.0

    changes = single.enforce_r17_runtime_contract(config)

    assert changes
    assert sweep["high_frequency_positive_first"] is True
    assert sweep["high_frequency_screen_step_hz"] == 100.0
    assert sweep["high_frequency_tier_radii_hz"] == [
        500.0, 1000.0, 1500.0, 2500.0]
    assert sweep["high_frequency_screen_frames_per_candidate"] == 4
    assert sweep["high_frequency_seed_maximum_foreground_occupancy"] == 0.11
    assert sweep["high_frequency_seed_minimum_union_stability"] == 0.45
    assert circle["required_passes"] == 5
    assert circle["high_frequency_confirmation_no_pass_blocks"] == 12
    assert circle["maximum_fill_ratio"] == 0.35
    assert config["target"]["control_timeout_s"] == 75.0


def test_47_9khz_measurement_searches_50khz_on_positive_side_first() -> None:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._coarse_frequency_hz = 47_900.0

    controller._start_circle_sweep(1.0)

    positive_tiers = controller._circle_sweep_tiers[:4]
    positive_frequencies = [value for tier in positive_tiers for value in tier]
    negative_frequencies = [
        value for tier in controller._circle_sweep_tiers[4:] for value in tier]
    assert 50_000.0 in positive_frequencies
    assert min(positive_frequencies) >= 47_900.0
    assert max(positive_frequencies) == 50_400.0
    assert negative_frequencies and max(negative_frequencies) < 47_900.0


def test_high_frequency_does_not_stop_after_first_false_tier() -> None:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._coarse_frequency_hz = 47_900.0
    controller._start_circle_sweep(1.0)
    controller._circle_sweep_results = [
        sweep_result(48_000.0, occupancy=0.08),
    ]

    controller._finish_circle_screen_tier(2.0)

    assert controller._circle_sweep_stage == "SCREEN"
    assert controller._circle_sweep_tier_index == 1
    assert controller._circle_sweep_frequencies == [
        48_500.0, 48_600.0, 48_700.0, 48_800.0, 48_900.0]


def test_dense_48khz_alias_is_rejected_but_real_circle_is_kept() -> None:
    alias = sweep_result(
        48_000.0,
        occupancy=0.118,
        trace=trace_fit(pixel_count=25_000, aggregate_pixel_count=126_819),
    )
    real = sweep_result(
        50_000.0,
        occupancy=0.082,
        trace=trace_fit(pixel_count=18_000, aggregate_pixel_count=20_000),
    )

    assert single.frequency_trace_union_stability(alias.trace_fit) < 0.20
    assert not single.high_frequency_sweep_result_is_clean(
        alias, single.DEFAULT_CONFIG)
    assert single.high_frequency_sweep_result_is_clean(
        real, single.DEFAULT_CONFIG)


def test_high_frequency_no_pass_confirmation_returns_to_frequency_search(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._coarse_frequency_hz = 50_000.0
    controller._circle_frequency_verified = True
    controller._circle_confirm_blocks = 12
    controller._circle_confirm_passes = 0
    controller._circle_phase_trial_stage = 0
    controller._circle_amplitude_trial_stage = 0
    rejected: list[str] = []
    monkeypatch.setattr(
        controller,
        "_reject_circle_confirmation_frequency",
        lambda _now, reason: rejected.append(reason),
    )

    controller._capture_circle_confirm_frame(3.0)

    assert rejected == ["no lock pass in 12 high-frequency blocks"]


def _definitions(path: Path) -> dict[str, str]:
    source = path.read_text(encoding="utf-8")
    tree = ast.parse(source)
    lines = source.splitlines()
    return {
        node.name: "\n".join(lines[node.lineno - 1:node.end_lineno])
        for node in ast.walk(tree)
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef))
    }


def test_r17_does_not_change_frequency_measurement_core() -> None:
    current = Path(single.__file__).resolve()
    backup = current.parent / "versions" / (
        "20260802_cv_r16_before_high_frequency_recovery") / "task5_cv_single.py"
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
