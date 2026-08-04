import cv2
import numpy as np

from controller import AutoLissajousController
from protocol import (
    CMD_ACK,
    CMD_PROBE_SINGLE,
    CMD_TARGET,
    FLAG_ACK,
    FLAG_ACK_REQUEST,
    Frame,
    RESULT_ACCEPTED,
)
from vision import CoarseMeasurement, TargetAnalyzer


class FakeLink:
    def __init__(self) -> None:
        self.phase = 0
        self.amplitude = 255
        self.target = 1
        self.frames = []
        self.sequence = 0

    def send(self, command: int, payload: bytes = bytes(8)) -> None:
        self.frames.append((command, payload))
        if command == CMD_TARGET:
            self.target = payload[0]
            self.amplitude = payload[1]
            self.phase = payload[2]

    def send_frame(self, command: int, payload: bytes = bytes(8), *,
                   flags: int = 0) -> Frame:
        self.send(command, payload)
        frame = Frame(self.sequence, command, payload, flags)
        self.sequence = (self.sequence + 1) & 0xFF
        return frame

    def resend(self, frame: Frame) -> Frame:
        self.send(frame.command, frame.payload)
        return Frame(frame.sequence, frame.command, frame.payload,
                     frame.flags | 0x02)


def acknowledge(controller: AutoLissajousController, now: float) -> None:
    pending = controller._pending_command
    assert pending is not None
    ack = Frame(200, CMD_ACK,
                bytes((pending.frame.sequence, pending.frame.command,
                       RESULT_ACCEPTED, 0, 0, 0, 0, 0)), FLAG_ACK)
    assert controller.handle_frame(ack, now)


class FakeCamera:
    def __init__(self, link: FakeLink, unknown_phase: int) -> None:
        self.link = link
        self.unknown_phase = unknown_phase
        self.analyzer = TargetAnalyzer({"vision": {"phase_search_step": 4}})

    def read(self) -> np.ndarray:
        image = np.zeros((480, 640, 3), np.uint8)
        relative_phase = (self.link.phase + self.unknown_phase) & 0xFF
        amplitude_y = 228.0 * self.link.amplitude / 255.0
        points = self.analyzer._model_points(
            self.link.target, relative_phase, 320, 240, 250, amplitude_y)
        for first, second in zip(points[:-1], points[1:]):
            cv2.line(image, tuple(first), tuple(second), (0, 255, 255), 3)
        return image


def test_phase_correction_resolves_static_xy_sign_ambiguity() -> None:
    config = {
        "vision": {
            "hsv_low": [25, 60, 90],
            "hsv_high": [100, 255, 255],
            "phase_search_step": 4,
        },
        "runtime": {"settle_s": 0, "aggregate_frames": 1, "frame_interval_s": 0},
        "target": {"initial_corrections": 2, "amplitude_min": 96},
    }
    link = FakeLink()
    camera = FakeCamera(link, unknown_phase=-28)
    controller = AutoLissajousController(config, link, camera)
    fit, phase, _ = controller._adjust_target(1, 123456, 0, 255)
    actual_phase = (phase - 28) & 0xFF
    distance = min(abs(((desired - actual_phase + 128) & 0xFF) - 128)
                   for desired in (0, 128))
    assert distance <= 6
    assert fit.quality >= 70


def test_controller_starts_with_100us_and_waits_for_probe_ack() -> None:
    config = {"coarse": {"settle_s": 0.18}}
    link = FakeLink()
    controller = AutoLissajousController(config, link, object())
    assert controller.start(2, now=10.0)
    assert controller.mode == "COARSE_WAIT_ACK"
    assert link.frames[-2][1][0] == 0
    assert controller._pending_command.frame.flags == FLAG_ACK_REQUEST
    controller.poll(10.17)
    assert controller.mode == "COARSE_WAIT_ACK"
    acknowledge(controller, 10.18)
    assert controller.mode == "COARSE_SETTLE"
    controller.poll(10.36)
    assert controller.mode == "COARSE_CAPTURE"


def test_probe_ack_timeout_retries_the_same_request_sequence() -> None:
    config = {"protocol": {"ack_timeout_s": 0.1, "ack_retries": 1}}
    link = FakeLink()
    controller = AutoLissajousController(config, link, object())
    assert controller.start(1, now=1.0)
    original = controller._pending_command.frame

    controller.poll(1.11)
    retried = controller._pending_command
    assert retried is not None
    assert retried.retries == 1
    assert retried.frame.sequence == original.sequence
    probes = [payload for command, payload in link.frames
              if command == CMD_PROBE_SINGLE]
    assert probes == [original.payload, original.payload]

    acknowledge(controller, 1.12)
    assert controller.mode == "COARSE_SETTLE"


def test_too_few_points_advances_100us_500us_then_2ms() -> None:
    config = {"coarse": {"widths": [0, 1, 2], "minimum_points": 5}}
    link = FakeLink()
    controller = AutoLissajousController(config, link, object())
    controller.start(1, now=1.0)

    acknowledge(controller, 1.0)

    too_few = CoarseMeasurement(
        False, 0.0, 1.0, 0.0, 4, 0, 0.0, "TOO_FEW_POINTS")
    controller._coarse_summary = lambda: too_few
    controller._finish_coarse(2.0)
    acknowledge(controller, 2.0)
    controller._finish_coarse(3.0)

    widths = [payload[0] for command, payload in link.frames
              if command == CMD_PROBE_SINGLE]
    assert widths == [0, 1, 2]
