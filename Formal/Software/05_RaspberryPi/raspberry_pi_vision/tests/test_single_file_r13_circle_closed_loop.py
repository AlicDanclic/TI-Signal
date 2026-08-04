import ast
import copy
from pathlib import Path

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


def circle_fit(
    score: float,
    *,
    span_x: float = 8.0,
    span_y: float = 5.0,
) -> single.CircleLockFit:
    return single.CircleLockFit(
        quality=int(round(score * 100.0)),
        score=score,
        span_x_div=span_x,
        span_y_div=span_y,
        center_error_div=0.1,
        radial_cv=0.10,
        inner_fill_ratio=0.10,
        angular_coverage=0.75,
        fill_ratio=0.18,
        pixel_count=900,
        ellipse_axis_ratio=max(span_x, span_y) / min(span_x, span_y),
    )


def phase_fit(phase: int = 64) -> single.TargetFit:
    return single.TargetFit(phase, 0.02, 70, 8.0, 5.0, 0.1, 0.02)


def test_partial_ellipse_can_be_corrected_without_relaxing_lock_gate() -> None:
    config = controller_config()
    partial = single.CircleLockFit(
        quality=72,
        score=0.72,
        span_x_div=7.5,
        span_y_div=5.5,
        center_error_div=0.2,
        radial_cv=0.18,
        inner_fill_ratio=0.12,
        angular_coverage=0.32,
        fill_ratio=0.20,
        pixel_count=800,
        ellipse_axis_ratio=1.36,
    )

    assert single.circle_fit_is_usable_for_correction(partial, config)
    assert not single.circle_fit_is_locked(partial, config)
    assert "COVERAGE" in single.circle_fit_lock_failures(partial, config)


def test_phase_trial_completion_always_requests_a_fresh_block(
    monkeypatch,
) -> None:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._phase = 64
    controller._amplitude = 103
    controller._tuning_word = single.dds_tuning_word_for_frequency(20_000.0)
    controller._circle_phase_trial_baseline = 64
    controller._circle_phase_trial_baseline_score = 0.40
    controller._circle_phase_trial_first_phase = 72
    controller._circle_phase_trial_first_score = 0.30
    controller._circle_phase_trial_delta = 8
    controller._circle_phase_trial_stage = 2
    controller._phase = 56
    sent: list[int] = []
    monkeypatch.setattr(
        controller,
        "_send_circle_confirm_target",
        lambda _now: sent.append(controller._phase),
    )

    outcome = controller._try_adjust_circle_target(
        np.ones((32, 32), np.uint8),
        circle_fit(0.35),
        2.0,
        phase_fit(64),
    )

    assert outcome == single.CIRCLE_ADJUST_SENT
    assert controller._phase == 64
    assert controller._circle_phase_trial_stage == 0
    assert sent == [64]


def test_amplitude_ab_trial_selects_the_measured_better_direction(
    monkeypatch,
) -> None:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._phase = 64
    controller._amplitude = 100
    controller._tuning_word = single.dds_tuning_word_for_frequency(20_000.0)
    sent: list[int] = []
    monkeypatch.setattr(
        controller,
        "_send_circle_confirm_target",
        lambda _now: sent.append(controller._amplitude),
    )

    assert controller._try_adjust_circle_target(
        np.ones((32, 32), np.uint8),
        circle_fit(0.50),
        1.0,
        phase_fit(),
    ) == single.CIRCLE_ADJUST_SENT
    assert controller._amplitude == 103
    assert controller._try_adjust_circle_target(
        np.ones((32, 32), np.uint8),
        circle_fit(0.45),
        2.0,
        phase_fit(),
    ) == single.CIRCLE_ADJUST_SENT
    assert controller._amplitude == 97
    assert controller._try_adjust_circle_target(
        np.ones((32, 32), np.uint8),
        circle_fit(0.80),
        3.0,
        phase_fit(),
    ) == single.CIRCLE_ADJUST_SENT

    assert controller._amplitude == 97
    assert controller._circle_amplitude_trial_stage == 0
    assert sent == [103, 97, 97]


def test_one_sided_amplitude_trial_keeps_improved_a_at_lower_limit(
    monkeypatch,
) -> None:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._phase = 64
    controller._amplitude = controller.config["target"]["amplitude_min"]
    controller._tuning_word = single.dds_tuning_word_for_frequency(20_000.0)
    sent: list[int] = []
    monkeypatch.setattr(
        controller,
        "_send_circle_confirm_target",
        lambda _now: sent.append(controller._amplitude),
    )

    baseline = controller._amplitude
    assert controller._try_adjust_circle_target(
        np.ones((32, 32), np.uint8),
        circle_fit(0.50),
        1.0,
        phase_fit(),
    ) == single.CIRCLE_ADJUST_SENT
    improved = controller._amplitude
    assert improved > baseline
    assert controller._try_adjust_circle_target(
        np.ones((32, 32), np.uint8),
        circle_fit(0.80),
        2.0,
        phase_fit(),
    ) == single.CIRCLE_ADJUST_SENT

    assert controller._amplitude == improved
    assert controller._circle_amplitude_trial_stage == 0
    assert sent == [improved, improved]


def test_one_complete_ab_trial_consumes_one_correction_budget(
    monkeypatch,
) -> None:
    config = controller_config()
    config["target"]["circle_lock"]["maximum_corrections"] = 24
    controller = single.AutoLissajousController(
        config, RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._phase = 64
    controller._amplitude = 100
    controller._circle_corrections = 23
    controller._tuning_word = single.dds_tuning_word_for_frequency(20_000.0)
    monkeypatch.setattr(
        controller, "_send_circle_confirm_target", lambda _now: None)

    assert controller._try_adjust_circle_target(
        np.ones((32, 32), np.uint8), circle_fit(0.50), 1.0, phase_fit()
    ) == single.CIRCLE_ADJUST_SENT
    assert controller._circle_corrections == 24
    assert controller._try_adjust_circle_target(
        np.ones((32, 32), np.uint8), circle_fit(0.45), 2.0, phase_fit()
    ) == single.CIRCLE_ADJUST_SENT
    assert controller._circle_corrections == 24
    assert controller._try_adjust_circle_target(
        np.ones((32, 32), np.uint8), circle_fit(0.80), 3.0, phase_fit()
    ) == single.CIRCLE_ADJUST_SENT
    assert controller._circle_corrections == 24


def test_correction_limit_is_explicit() -> None:
    config = controller_config()
    config["target"]["circle_lock"]["maximum_corrections"] = 2
    controller = single.AutoLissajousController(
        config, RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._circle_corrections = 2

    outcome = controller._try_adjust_circle_target(
        np.ones((32, 32), np.uint8),
        circle_fit(0.50),
        1.0,
        phase_fit(0),
    )

    assert outcome == single.CIRCLE_ADJUST_LIMIT


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


def test_r13_does_not_change_frequency_measurement_definitions() -> None:
    current = Path(single.__file__).resolve()
    backup = current.parent / "versions" / (
        "20260802_cv_r12_before_circle_closed_loop_fix") / "task5_cv_single.py"
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
        assert current_definitions[name] == backup_definitions[name], name
