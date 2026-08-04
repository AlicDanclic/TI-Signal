import copy
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

import task5_cv_single as single


def stable_coarse_observation() -> single.CoarseFrameObservation:
    return single.CoarseFrameObservation(
        point_count=6,
        left_periods=(0.200, 0.201),
        right_periods=(0.199, 0.200),
        confidence=0.90,
        raw_turn_count=6,
    )


def test_two_of_three_stable_frames_pass_the_r7_valid_ratio() -> None:
    observations = [
        stable_coarse_observation(),
        stable_coarse_observation(),
        single.CoarseFrameObservation(0, (), (), 0.0, 0),
    ]

    accepted = single.summarize_coarse_observations(
        observations,
        width_us=500.0,
        minimum_valid_ratio=0.60,
        minimum_confidence=0.10,
    )
    rejected_by_old_threshold = single.summarize_coarse_observations(
        observations,
        width_us=500.0,
        minimum_valid_ratio=0.70,
        minimum_confidence=0.10,
    )

    assert single.DEFAULT_CONFIG["coarse"]["minimum_valid_ratio"] == 0.60
    assert accepted.accepted
    assert accepted.reason == "OK"
    assert accepted.valid_frame_ratio == pytest.approx(2.0 / 3.0)
    assert 9_900.0 < accepted.frequency_hz < 10_100.0
    assert not rejected_by_old_threshold.accepted
    assert rejected_by_old_threshold.reason == "LOW_VALID_FRAME_RATIO"


def test_field_20khz_two_of_three_frames_enters_coarse_success() -> None:
    period = 1_000_000.0 / (20_400.0 * 500.0)
    stable = single.CoarseFrameObservation(
        point_count=8,
        left_periods=(period, period * 1.001, period * 0.999),
        right_periods=(period * 1.0005, period, period * 0.9995),
        confidence=0.80,
        raw_turn_count=8,
    )
    summary = single.summarize_coarse_observations(
        [stable, stable, single.CoarseFrameObservation(0, (), (), 0.0, 0)],
        width_us=500.0,
        minimum_valid_ratio=single.DEFAULT_CONFIG["coarse"][
            "minimum_valid_ratio"],
        minimum_confidence=0.10,
    )

    assert summary.accepted
    assert summary.frequency_hz == pytest.approx(20_400.0)
    assert summary.valid_frame_ratio == pytest.approx(2.0 / 3.0)


def make_short_stationary_trace(extent_fraction: float) -> np.ndarray:
    height, width = 512, 640
    span = max(2, int(round(width * extent_fraction)))
    x0 = (width - span) // 2
    mask = np.zeros((height, width), np.uint8)
    cv2.rectangle(mask, (x0, 254), (x0 + span - 1, 257), 255, -1)
    return mask


@pytest.mark.parametrize("extent_fraction", (0.045, 0.10, 0.20))
def test_short_one_axis_trace_is_retained_but_cannot_seed_lock(
        extent_fraction: float) -> None:
    config = copy.deepcopy(single.DEFAULT_CONFIG)
    sweep = config["target"]["circle_sweep"]
    sweep["trace_minimum_pixels"] = 20
    sweep["trace_minimum_frames"] = 2
    trace = make_short_stationary_trace(extent_fraction)

    fit = single.analyze_frequency_trace_masks([trace, trace], config)

    assert sweep["trace_minimum_extent_fraction"] == pytest.approx(0.18)
    assert sweep["trace_hard_minimum_extent_fraction"] == pytest.approx(0.04)
    assert fit.valid_frames == 2
    assert fit.temporal_overlap == pytest.approx(1.0)
    assert fit.extent_quality < 0.10
    assert fit.quality < sweep["minimum_trace_quality"]


def test_trace_below_about_four_percent_extent_is_still_rejected() -> None:
    config = copy.deepcopy(single.DEFAULT_CONFIG)
    sweep = config["target"]["circle_sweep"]
    sweep["trace_minimum_pixels"] = 20
    sweep["trace_minimum_frames"] = 2
    trace = make_short_stationary_trace(0.03)

    with pytest.raises(ValueError, match=r"extent=2"):
        single.analyze_frequency_trace_masks([trace, trace], config)


class EventLink:
    def __init__(self, frame: single.Frame) -> None:
        self.frame = frame
        self.poll_count = 0
        self.replies: list[tuple[bool, int]] = []
        self.closed = False

    def poll(self) -> list[single.Frame]:
        self.poll_count += 1
        if self.poll_count == 1:
            return [self.frame]
        raise KeyboardInterrupt

    def reply(self, _frame: single.Frame, *, accepted: bool,
              result: int) -> None:
        self.replies.append((accepted, result))

    def close(self) -> None:
        self.closed = True


class EventCamera:
    def __init__(self) -> None:
        self.closed = False

    def close(self) -> None:
        self.closed = True


class ProbeRestoreLink:
    def __init__(self) -> None:
        self.frames: list[single.Frame] = []

    def send_frame(self, command: int, payload: bytes, flags: int = 0
                   ) -> single.Frame:
        frame = single.Frame(len(self.frames), command, payload, flags)
        self.frames.append(frame)
        return frame


def test_target_failure_restores_default_100us_probe() -> None:
    link = ProbeRestoreLink()
    controller = single.AutoLissajousController(
        copy.deepcopy(single.DEFAULT_CONFIG), link, EventCamera())
    controller._mode = "CIRCLE_SWEEP_CAPTURE"
    controller._coarse_width_code = 1

    controller._fail(single.ERROR_TIMEOUT, "synthetic sweep timeout")

    assert controller.mode == "ERROR"
    assert link.frames[-1].command == single.CMD_PROBE_SINGLE
    assert link.frames[-1].payload == bytes(8)


class RestartableController:
    def __init__(self, initial_mode: str) -> None:
        self.mode = initial_mode
        self.starts: list[tuple[int, float]] = []

    @property
    def active(self) -> bool:
        return self.mode not in ("IDLE", "ERROR", "LOCKED_HOLD")

    def start(self, target: int, now: float) -> bool:
        self.starts.append((target, now))
        self.mode = "COARSE_SETTLE"
        return True

    def poll(self, _now: float) -> None:
        pass

    def handle_frame(self, _frame: single.Frame, _now: float) -> bool:
        return False

    def cancel(self, _target: int = 0) -> None:
        self.mode = "IDLE"

    @staticmethod
    def _coarse_command_width_us(code: int) -> float:
        return (100.0, 500.0, 2000.0, 5000.0)[code]

    @staticmethod
    def _coarse_calculation_width_us(code: int) -> float:
        return (100.0, 500.0, 2000.0, 5000.0)[code]


@pytest.mark.parametrize(
    "initial_mode",
    ("COARSE_CAPTURE", "ERROR", "LOCKED_HOLD"),
)
def test_event_start_restarts_controller_from_any_previous_state(
        monkeypatch, initial_mode: str) -> None:
    request = single.Frame(
        17,
        single.EVENT_START,
        bytes((single.TARGET_CIRCLE, 0, 0, 0, 0, 0, 0, 0)),
        single.FLAG_ACK_REQUEST,
    )
    link = EventLink(request)
    camera = EventCamera()
    controller = RestartableController(initial_mode)
    args = SimpleNamespace(
        config=None,
        preview=False,
        preview_dir="unused",
        port=None,
        source=None,
        log_level="INFO",
    )
    monkeypatch.setattr(single, "parse_args", lambda: args)
    monkeypatch.setattr(single, "SerialLink", lambda *_args, **_kwargs: link)
    monkeypatch.setattr(single, "ScopeCamera", lambda *_args, **_kwargs: camera)
    monkeypatch.setattr(
        single,
        "AutoLissajousController",
        lambda *_args, **_kwargs: controller,
    )

    assert single.main() == 0

    assert len(controller.starts) == 1
    assert controller.starts[0][0] == single.TARGET_CIRCLE
    assert link.replies == [(True, single.RESULT_ACCEPTED)]
    assert link.closed
    assert camera.closed
