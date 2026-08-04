import ast
import copy
from pathlib import Path

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


def controller_config() -> dict:
    return copy.deepcopy(single.DEFAULT_CONFIG)


def target_fit(phase: int, score: float = 0.08) -> single.TargetFit:
    return single.TargetFit(
        estimated_phase=phase,
        desired_score=score,
        quality=25,
        span_x_div=7.5,
        span_y_div=7.7,
        center_error_div=0.2,
        model_score=0.008,
    )


def usable_circle_fit() -> single.CircleLockFit:
    return single.CircleLockFit(
        quality=65,
        score=0.65,
        span_x_div=7.5,
        span_y_div=5.5,
        center_error_div=0.2,
        radial_cv=0.15,
        inner_fill_ratio=0.10,
        angular_coverage=0.55,
        fill_ratio=0.20,
        pixel_count=800,
        ellipse_axis_ratio=1.36,
    )


@pytest.mark.parametrize("target", (
    single.TARGET_DIAGONAL,
    single.TARGET_EIGHT,
))
def test_non_circle_far_phase_uses_full_measured_error(
    target: int,
    monkeypatch,
) -> None:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = target
    controller._phase = 0
    controller._amplitude = 103
    controller._tuning_word = single.dds_tuning_word_for_frequency(10_000.0)
    sent: list[int] = []
    monkeypatch.setattr(
        controller,
        "_send_circle_confirm_target",
        lambda _now: sent.append(controller._phase),
    )

    outcome = controller._try_adjust_circle_target(
        np.ones((32, 32), np.uint8),
        None,
        1.0,
        target_fit(64),
    )

    assert outcome == single.CIRCLE_ADJUST_SENT
    assert controller._circle_phase_trial_delta == -64
    assert controller._phase == 192
    assert sent == [192]


def test_circle_still_uses_bounded_phase_step(monkeypatch) -> None:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_CIRCLE
    controller._phase = 0
    controller._amplitude = 103
    controller._tuning_word = single.dds_tuning_word_for_frequency(10_000.0)
    monkeypatch.setattr(
        controller,
        "_send_circle_confirm_target",
        lambda _now: None,
    )

    outcome = controller._try_adjust_circle_target(
        np.ones((32, 32), np.uint8),
        usable_circle_fit(),
        1.0,
        target_fit(0, 0.02),
    )

    assert outcome == single.CIRCLE_ADJUST_SENT
    assert abs(controller._circle_phase_trial_delta) == 16


def test_non_circle_small_score_noise_restores_baseline(monkeypatch) -> None:
    controller = single.AutoLissajousController(
        controller_config(), RecordingLink(), FreshFrameCamera())
    controller._target = single.TARGET_DIAGONAL
    controller._phase = 0
    controller._amplitude = 103
    controller._tuning_word = single.dds_tuning_word_for_frequency(10_000.0)
    sent: list[int] = []
    monkeypatch.setattr(
        controller,
        "_send_circle_confirm_target",
        lambda _now: sent.append(controller._phase),
    )

    assert controller._try_adjust_circle_target(
        np.ones((32, 32), np.uint8), None, 1.0, target_fit(64, 0.083)
    ) == single.CIRCLE_ADJUST_SENT
    assert controller._try_adjust_circle_target(
        np.ones((32, 32), np.uint8), None, 2.0, target_fit(60, 0.082)
    ) == single.CIRCLE_ADJUST_SENT
    assert controller._try_adjust_circle_target(
        np.ones((32, 32), np.uint8), None, 3.0, target_fit(68, 0.081)
    ) == single.CIRCLE_ADJUST_SENT

    assert controller._phase == 0
    assert sent[-1] == 0


def _definitions(path: Path) -> dict[str, str]:
    source = path.read_text(encoding="utf-8")
    lines = source.splitlines()
    tree = ast.parse(source)
    definitions: dict[str, str] = {}
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            definitions[node.name] = "\n".join(
                lines[node.lineno - 1:node.end_lineno])
    return definitions


def test_r14_keeps_frequency_measurement_code_identical_to_r13() -> None:
    current = Path(single.__file__).resolve()
    backup = current.parent / "versions" / (
        "20260802_cv_r13_before_non_circle_phase_fix") / "task5_cv_single.py"
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

    for name in protected:
        assert current_definitions[name] == backup_definitions[name], name
