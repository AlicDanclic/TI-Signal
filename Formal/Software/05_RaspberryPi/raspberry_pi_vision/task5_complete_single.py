#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TI Cup 电赛 F题 - 第五问自动锁圆完整单文件版本
Task5 Complete Single File with Auto Lock Circle

此文件整合了所有功能模块，可以独立运行。

包含功能：
- 串口协议处理（与STM32/FPGA通信）
- 摄像头接口（线程安全的帧缓冲）
- OpenCV图像处理（固定机位拐点提取）
- 视觉算法（频率估计、相位分析、目标检测）
- 圆形检测与质量评估（新增）
- 频率扫描（100Hz步进）（新增）
- 自动控制器（完整状态机）

Version: 1.0-AutoCircle
Build Date: 2026-08-01
Author: Claude (Kiro)

使用方法：
    python3 task5_complete_single.py --config config.yaml
    python3 task5_complete_single.py --config config.yaml --preview
"""

from __future__ import annotations

import argparse
import logging
import math
import threading
import time
import csv
from collections import deque
from dataclasses import dataclass, replace
from math import ceil, floor, pi
from pathlib import Path
from typing import Any, Iterable

import cv2
import numpy as np

LOGGER = logging.getLogger(__name__)




# ========== 协议模块 ==========


from dataclasses import dataclass
from typing import Iterable



SYNC = bytes((0xA5, 0x5A))
PROTOCOL_MARKER = 0x51
FRAME_SIZE = 16

# Task5 uses this frame only.  Task1--Task4 retain their original UART
# protocol, therefore their byte layout is deliberately not accepted here.
FLAG_ACK_REQUEST = 0x01
FLAG_RETRY = 0x02
FLAG_NACK = 0x40
FLAG_ACK = 0x80

CMD_DISABLE = 0x00
CMD_STANDBY = 0x01
CMD_PROBE_SINGLE = 0x10
CMD_PROBE_DUAL = 0x11
CMD_TARGET = 0x20

EVENT_START = 0x40
EVENT_CANCEL = 0x41

STATUS_PROGRESS = 0x80
STATUS_LOCKED = 0x81
STATUS_ERROR = 0x82
STATUS_HEARTBEAT = 0x83

CMD_ACK = 0x70
CMD_NACK = 0x71

# Explicit names make the new Task5 state machine easier to read while old
# entry points and tests can continue to use the CMD_/EVENT_ names above.
OP_STOP = CMD_DISABLE
OP_ARM = CMD_STANDBY
OP_PROBE_SINGLE = CMD_PROBE_SINGLE
OP_PROBE_DUAL = CMD_PROBE_DUAL
OP_TARGET = CMD_TARGET
OP_START = EVENT_START
OP_CANCEL = EVENT_CANCEL
OP_PROGRESS = STATUS_PROGRESS
OP_LOCKED = STATUS_LOCKED
OP_ERROR = STATUS_ERROR
OP_HEARTBEAT = STATUS_HEARTBEAT
OP_ACK = CMD_ACK
OP_NACK = CMD_NACK

STATE_IDLE = 0
STATE_WAIT_PI = 1
STATE_COARSE = 2
STATE_FINE_PHASE = 3
STATE_TRACK = 4
STATE_LOCKED = 5
STATE_ERROR = 6

TARGET_DIAGONAL = 1
TARGET_CIRCLE = 2
TARGET_EIGHT = 3

# Backward-compatible names used by older tests and helper scripts.
STATE_PROBE = STATE_COARSE
STATE_DUAL = STATE_FINE_PHASE

ERROR_VISUAL_RANGE = 1
ERROR_COARSE_FAILED = 2
ERROR_PHASE_UNSTABLE = 3
ERROR_TIMEOUT = 4
ERROR_CANCELLED = 5
ERROR_CAMERA = 6

RESULT_ACCEPTED = 0
RESULT_DUPLICATE = 1
RESULT_BAD_ARGUMENT = 2
RESULT_UNSUPPORTED = 3
RESULT_BUSY = 4


def crc16_ccitt_false(data: bytes | bytearray | Iterable[int]) -> int:
    """Return CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)."""
    crc = 0xFFFF
    for value in data:
        crc ^= (int(value) & 0xFF) << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (
                crc << 1) & 0xFFFF
    return crc


@dataclass(frozen=True)
class Frame:
    sequence: int
    command: int
    payload: bytes
    flags: int = 0

    def __post_init__(self) -> None:
        if len(self.payload) != 8:
            raise ValueError("payload must contain exactly 8 bytes")
        if not 0 <= int(self.sequence) <= 0xFF:
            raise ValueError("sequence must fit in one byte")
        if not 0 <= int(self.command) <= 0xFF:
            raise ValueError("command must fit in one byte")
        if not 0 <= int(self.flags) <= 0xFF:
            raise ValueError("flags must fit in one byte")

    @property
    def opcode(self) -> int:
        """Alias used by the wire-protocol documentation."""
        return self.command

    @property
    def requests_ack(self) -> bool:
        return bool(self.flags & FLAG_ACK_REQUEST)

    @property
    def is_retry(self) -> bool:
        return bool(self.flags & FLAG_RETRY)

    def encode(self) -> bytes:
        body = (SYNC + bytes((PROTOCOL_MARKER, self.flags & 0xFF,
                              self.sequence & 0xFF, self.command & 0xFF)) +
                self.payload)
        return body + crc16_ccitt_false(body).to_bytes(2, "little")


class FrameParser:
    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes | bytearray | Iterable[int]) -> list[Frame]:
        self._buffer.extend(data)
        frames: list[Frame] = []
        while True:
            sync_index = self._buffer.find(SYNC)
            if sync_index < 0:
                self._buffer[:] = self._buffer[-1:] if self._buffer[-1:] == SYNC[:1] else b""
                break
            if sync_index:
                del self._buffer[:sync_index]
            if len(self._buffer) < 3:
                break
            if self._buffer[2] != PROTOCOL_MARKER:
                # Drop one byte instead of the whole prefix.  A valid A5 5A
                # sequence beginning inside corrupt input is then preserved.
                del self._buffer[0]
                continue
            if len(self._buffer) < FRAME_SIZE:
                break
            raw = bytes(self._buffer[:FRAME_SIZE])
            expected_crc = int.from_bytes(raw[14:16], "little")
            if crc16_ccitt_false(raw[:14]) != expected_crc:
                del self._buffer[0]
                continue
            frames.append(Frame(raw[4], raw[5], raw[6:14], raw[3]))
            del self._buffer[:FRAME_SIZE]
        return frames


class SequenceTracker:
    """Reject only explicit retry duplicates while allowing restart/wrap reuse."""

    def __init__(self) -> None:
        self._last: Frame | None = None

    def reset(self) -> None:
        self._last = None

    def accept(self, frame: Frame) -> bool:
        if (self._last is not None and frame.is_retry and
                frame.sequence == self._last.sequence):
            return False
        self._last = frame
        return True


class SerialLink:
    def __init__(self, port: str, baudrate: int = 115200,
                 reconnect_interval_s: float = 1.0) -> None:
        try:
        except ImportError as exc:
            raise RuntimeError("pyserial is required for the STM32 link") from exc
        self._serial_module = serial
        self._serial_exception = getattr(serial, "SerialException", OSError)
        self._port = port
        self._baudrate = baudrate
        self._reconnect_interval_s = max(0.0, float(reconnect_interval_s))
        self._serial = None
        self._next_reconnect_at = 0.0
        self._last_warning_at = 0.0
        self._last_error = ""
        self._parser = FrameParser()
        self._sequence = 0
        self._open_if_due(force=True)

    def close(self) -> None:
        serial_port = self._serial
        self._serial = None
        if serial_port is not None:
            try:
                serial_port.close()
            except (self._serial_exception, OSError):
                pass

    def _warn(self, message: str) -> None:
        now = time.monotonic()
        if message != self._last_error or now - self._last_warning_at >= 5.0:
            LOGGER.warning("serial %s unavailable: %s; retrying", self._port, message)
            self._last_error = message
            self._last_warning_at = now

    def _disconnect(self, error: BaseException) -> None:
        self.close()
        self._next_reconnect_at = time.monotonic() + self._reconnect_interval_s
        self._warn(str(error))

    def _open_if_due(self, force: bool = False) -> bool:
        if self._serial is not None and self._serial.is_open:
            return True
        now = time.monotonic()
        if not force and now < self._next_reconnect_at:
            return False
        self._next_reconnect_at = now + self._reconnect_interval_s
        try:
            self._serial = self._serial_module.Serial(
                self._port, baudrate=self._baudrate, timeout=0)
        except (self._serial_exception, OSError) as exc:
            self._serial = None
            self._warn(str(exc))
            return False
        self._parser = FrameParser()
        self._last_error = ""
        LOGGER.info("serial %s connected at %d baud", self._port, self._baudrate)
        return True

    def poll(self) -> list[Frame]:
        if not self._open_if_due():
            return []
        try:
            waiting = self._serial.in_waiting
            data = self._serial.read(waiting or 1)
        except (self._serial_exception, OSError) as exc:
            self._disconnect(exc)
            return []
        # Callers must see a retransmitted START/CANCEL so they can ACK it
        # again.  Idempotence belongs to the state machine, not this byte
        # transport layer.
        return self._parser.feed(data)

    def _write_frame(self, frame: Frame) -> bool:
        if not self._open_if_due():
            return False
        try:
            self._serial.write(frame.encode())
        except (self._serial_exception, OSError) as exc:
            self._disconnect(exc)
            return False
        return True

    def send_frame(self, command: int, payload: bytes = bytes(8), *,
                   flags: int = 0) -> Frame | None:
        """Transmit a new frame and return its immutable on-wire identity."""
        frame = Frame(self._sequence, command, payload, flags)
        if not self._write_frame(frame):
            return None
        self._sequence = (self._sequence + 1) & 0xFF
        return frame

    def resend(self, frame: Frame) -> Frame | None:
        """Repeat a request with its original sequence and payload."""
        retry = Frame(frame.sequence, frame.command, frame.payload,
                      frame.flags | FLAG_RETRY)
        return retry if self._write_frame(retry) else None

    def send(self, command: int, payload: bytes = bytes(8), *,
             flags: int = 0) -> bool:
        """Compatibility helper for unsolicited status frames."""
        return self.send_frame(command, payload, flags=flags) is not None

    def reply(self, request: Frame, *, accepted: bool,
              result: int = RESULT_ACCEPTED,
              active_mode: int = 0, active_width: int = 0,
              manual_dds: int = 0) -> bool:
        """Send the standard ACK/NACK payload for an incoming request."""
        payload = bytes((request.sequence & 0xFF, request.command & 0xFF,
                         result & 0xFF, active_mode & 0xFF,
                         active_width & 0xFF, manual_dds & 0xFF, 0, 0))
        return self.send(
            CMD_ACK if accepted else CMD_NACK,
            payload,
            flags=FLAG_ACK if accepted else FLAG_NACK,
        )


def u32le(value: int) -> bytes:
    return int(value).to_bytes(4, "little", signed=False)


def progress_payload(state: int, stage: int, quality: int, point_count: int,
                     frequency_millihz: int) -> bytes:
    return bytes((state & 0xFF, stage & 0xFF,
                  max(0, min(100, quality)),
                  max(0, min(255, point_count)))) + u32le(frequency_millihz)


def locked_payload(target: int, quality: int, coarse_width_code: int,
                   frequency_millihz: int) -> bytes:
    return bytes((target & 0xFF, max(0, min(100, quality)),
                  coarse_width_code & 0xFF, 0)) + u32le(frequency_millihz)



# ========== 摄像头模块 ==========


from typing import Any



class ScopeCamera:
    def __init__(self, config: dict[str, Any], source: str | int | None = None) -> None:
        camera_config = config.get("camera", {})
        selected = camera_config.get("device", 0) if source is None else source
        if isinstance(selected, str) and selected.isdigit():
            selected = int(selected)
        self._is_file = isinstance(selected, str) and Path(selected).exists()
        self._capture = cv2.VideoCapture(selected)
        if not self._capture.isOpened():
            raise RuntimeError(f"cannot open camera/video source: {selected}")
        if not self._is_file:
            self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, camera_config.get("width", 1280))
            self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_config.get("height", 720))
            self._capture.set(cv2.CAP_PROP_FPS, camera_config.get("fps", 30))
            self._capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, camera_config.get("auto_exposure", 1))
            exposure = camera_config.get("exposure")
            if exposure is not None:
                self._capture.set(cv2.CAP_PROP_EXPOSURE, exposure)
        self._config = camera_config
        output = config.get("vision", {}).get("canonical_size", [640, 480])
        self._output_size = (int(output[0]), int(output[1]))

    def close(self) -> None:
        self._capture.release()

    def _read_capture(self) -> np.ndarray:
        ok, frame = self._capture.read()
        if not ok and self._is_file:
            self._capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self._capture.read()
        if not ok or frame is None:
            raise RuntimeError("camera returned no frame")
        return frame

    def read_raw(self) -> np.ndarray:
        """Return one camera frame before the runtime ROI transformation."""
        return self._read_capture()

    def read(self) -> np.ndarray:
        return self._rectify(self._read_capture())

    def _rectify(self, frame: np.ndarray) -> np.ndarray:
        points = self._config.get("perspective_points")
        if points and len(points) == 4:
            source = np.asarray(points, dtype=np.float32)
            width, height = self._output_size
            destination = np.asarray(
                [[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]],
                dtype=np.float32,
            )
            transform = cv2.getPerspectiveTransform(source, destination)
            return cv2.warpPerspective(frame, transform, self._output_size)

        roi = self._config.get("roi", [0.0, 0.0, 1.0, 1.0])
        height, width = frame.shape[:2]
        x, y, roi_width, roi_height = [float(value) for value in roi]
        if max(x, y, roi_width, roi_height) <= 1.0:
            x, roi_width = x * width, roi_width * width
            y, roi_height = y * height, roi_height * height
        x0 = max(0, min(width - 1, int(round(x))))
        y0 = max(0, min(height - 1, int(round(y))))
        x1 = max(x0 + 1, min(width, int(round(x + roi_width))))
        y1 = max(y0 + 1, min(height, int(round(y + roi_height))))
        return cv2.resize(frame[y0:y1, x0:x1], self._output_size,
                          interpolation=cv2.INTER_AREA)



# ========== 视觉算法模块 ==========


from dataclasses import dataclass
from typing import Any



@dataclass(frozen=True)
class ProbeFit:
    cycles: float
    phase_radians: float
    confidence: float
    sample_count: int


@dataclass(frozen=True)
class DualProbeFit:
    grid_frequency_hz: int
    tuning_word: int
    phase_difference_cycles: float
    confidence: float
    fit_a: ProbeFit
    fit_b: ProbeFit


@dataclass(frozen=True)
class DualPhaseFit:
    phase_difference_cycles: float
    confidence: float
    fit_a: ProbeFit
    fit_b: ProbeFit


@dataclass(frozen=True)
class CoarseFrameObservation:
    point_count: int
    left_periods: tuple[float, ...]
    right_periods: tuple[float, ...]
    confidence: float


@dataclass(frozen=True)
class CoarseMeasurement:
    accepted: bool
    frequency_hz: float
    period_cv: float
    valid_frame_ratio: float
    median_point_count: int
    complete_period_count: int
    confidence: float
    reason: str


@dataclass(frozen=True)
class FineFrequencyFit:
    frequency_hz: float
    correction_hz: float
    residual_cycles: float
    confidence: float


def wrap_cycles(value: float) -> float:
    return (float(value) + 0.5) % 1.0 - 0.5


def circular_mean_cycles(values: list[float] | tuple[float, ...]) -> float:
    if not values:
        raise ValueError("at least one phase sample is required")
    angles = np.asarray(values, np.float64) * (2.0 * pi)
    vector = np.mean(np.exp(1j * angles))
    if abs(vector) < 1e-9:
        raise ValueError("phase samples have no circular consensus")
    return wrap_cycles(float(np.angle(vector) / (2.0 * pi)))


def reject_integer_multiple_periods(
        periods: list[float] | tuple[float, ...],
        tolerance: float = 0.20,
        maximum_multiple: int = 5) -> list[float]:
    """Keep the direct-period cluster and discard missed-point multiples."""
    values = np.asarray([value for value in periods if value > 0.0], np.float64)
    if values.size == 0:
        return []
    candidates = np.concatenate(
        [values / multiple for multiple in range(1, maximum_multiple + 1)])
    best_key: tuple[float, int, float] | None = None
    best_center = 0.0
    for candidate in candidates:
        if candidate <= 0.0:
            continue
        ratios = values / candidate
        nearest = np.rint(ratios)
        harmonic = ((nearest >= 1.0) & (nearest <= maximum_multiple) &
                    (np.abs(ratios - nearest) <= tolerance))
        direct = np.abs(ratios - 1.0) <= tolerance
        # A real direct-period cluster outranks a hypothetical sub-harmonic.
        score = 3.0 * float(np.count_nonzero(direct)) + float(np.count_nonzero(harmonic))
        key = (score, int(np.count_nonzero(direct)), float(candidate))
        if best_key is None or key > best_key:
            best_key = key
            best_center = float(candidate)
    direct_values = values[np.abs(values / best_center - 1.0) <= tolerance]
    if direct_values.size == 0:
        return []
    center = float(np.median(direct_values))
    return [float(value) for value in values
            if abs(value / center - 1.0) <= tolerance]


def coarse_observation_from_points(
        points: list[Any], ramp_height_px: float = 469.05) -> CoarseFrameObservation:
    if ramp_height_px <= 0.0:
        raise ValueError("ramp height must be positive")
    sides: dict[int, list[float]] = {0: [], 1: []}
    strengths: list[float] = []
    for point in points:
        side = 0 if float(point.x_normalized) < 0.0 else 1
        sides[side].append(float(point.y_px))
        strengths.append(float(point.strength))

    periods: dict[int, tuple[float, ...]] = {}
    for side, times in sides.items():
        ordered = sorted(times)
        periods[side] = tuple(
            (second - first) / ramp_height_px
            for first, second in zip(ordered, ordered[1:])
            if second > first)
    confidence = float(np.median(strengths)) if strengths else 0.0
    return CoarseFrameObservation(
        len(points), periods[0], periods[1], confidence)


def summarize_coarse_observations(
        observations: list[CoarseFrameObservation], width_us: float,
        minimum_points: int = 5, minimum_periods: int = 3,
        minimum_valid_ratio: float = 0.70, maximum_cv: float = 0.08,
        minimum_confidence: float = 0.35,
        maximum_points: int = 22) -> CoarseMeasurement:
    if not observations:
        return CoarseMeasurement(False, 0.0, 1.0, 0.0, 0, 0, 0.0,
                                 "NO_FRAMES")
    point_counts = np.asarray([item.point_count for item in observations])
    median_points = int(round(float(np.median(point_counts))))
    if median_points > maximum_points:
        return CoarseMeasurement(False, 0.0, 1.0, 0.0, median_points, 0, 0.0,
                                 "VISUAL_RANGE_HIGH")

    frame_periods: list[float] = []
    frame_confidences: list[float] = []
    complete_period_count = 0
    for observation in observations:
        left = reject_integer_multiple_periods(observation.left_periods)
        right = reject_integer_multiple_periods(observation.right_periods)
        combined_count = len(left) + len(right)
        if (observation.point_count < minimum_points or not left or not right or
                combined_count < minimum_periods or
                observation.confidence < minimum_confidence):
            continue
        side_periods = [float(np.median(left)), float(np.median(right))]
        frame_periods.append(float(np.median(side_periods)))
        frame_confidences.append(observation.confidence)
        complete_period_count += combined_count

    valid_ratio = len(frame_periods) / len(observations)
    if not frame_periods:
        return CoarseMeasurement(False, 0.0, 1.0, valid_ratio, median_points,
                                 complete_period_count, 0.0, "NO_VALID_PERIODS")
    periods = np.asarray(frame_periods, np.float64)
    period = float(np.median(periods))
    mad = float(np.median(np.abs(periods - period)))
    robust_sigma = 1.4826 * mad
    cv = robust_sigma / max(period, 1e-12)
    frequency = 1_000_000.0 / (period * width_us)
    visual_confidence = float(np.median(frame_confidences))
    ratio_score = min(1.0, valid_ratio / max(minimum_valid_ratio, 1e-6))
    cv_score = max(0.0, 1.0 - cv / max(maximum_cv, 1e-6))
    confidence = float(np.clip(
        0.45 * visual_confidence + 0.35 * ratio_score + 0.20 * cv_score,
        0.0, 1.0))

    reason = "OK"
    accepted = True
    if median_points < minimum_points:
        accepted, reason = False, "TOO_FEW_POINTS"
    elif complete_period_count < minimum_periods:
        accepted, reason = False, "TOO_FEW_PERIODS"
    elif valid_ratio < minimum_valid_ratio:
        accepted, reason = False, "LOW_VALID_FRAME_RATIO"
    elif cv > maximum_cv:
        accepted, reason = False, "PERIOD_UNSTABLE"
    elif confidence < minimum_confidence:
        accepted, reason = False, "LOW_CONFIDENCE"
    return CoarseMeasurement(
        accepted, frequency, cv, valid_ratio, median_points,
        complete_period_count, confidence, reason)


def resolve_dual_interval_frequency(
        coarse_frequency_hz: float, phase_3ms_cycles: float,
        phase_7ms_cycles: float, coarse_uncertainty_hz: float = 450.0,
        confidence_3ms: float = 1.0, confidence_7ms: float = 1.0,
        maximum_residual_cycles: float = 0.08) -> FineFrequencyFit:
    """Jointly unwrap the FPGA-timed 3 ms and 7 ms phase measurements."""
    if coarse_frequency_hz <= 0.0 or coarse_uncertainty_hz <= 0.0:
        raise ValueError("coarse frequency and uncertainty must be positive")
    phase_3 = wrap_cycles(phase_3ms_cycles)
    phase_7 = wrap_cycles(phase_7ms_cycles)
    low = coarse_frequency_hz - coarse_uncertainty_hz
    high = coarse_frequency_hz + coarse_uncertainty_hz
    dt_3 = 0.003
    dt_7 = 0.007
    first_n = floor(low * dt_7 - phase_7) - 1
    last_n = ceil(high * dt_7 - phase_7) + 1
    candidates: list[tuple[float, float, float]] = []
    weight_3 = max(0.05, confidence_3ms)
    weight_7 = max(0.05, confidence_7ms)
    for cycle_count in range(first_n, last_n + 1):
        frequency = (cycle_count + phase_7) / dt_7
        if not low <= frequency <= high:
            continue
        residual_3 = abs(wrap_cycles(frequency * dt_3 - phase_3))
        residual_7 = abs(wrap_cycles(frequency * dt_7 - phase_7))
        phase_cost = residual_3 * residual_3 / weight_3 + residual_7 * residual_7 / weight_7
        coarse_cost = 1e-4 * (
            (frequency - coarse_frequency_hz) / coarse_uncertainty_hz) ** 2
        candidates.append((phase_cost + coarse_cost, frequency,
                           max(residual_3, residual_7)))
    if not candidates:
        raise ValueError("no 3/7 ms frequency candidate in coarse range")
    candidates.sort()
    _, frequency, residual = candidates[0]
    if residual > maximum_residual_cycles:
        raise ValueError(f"3/7 ms phase residual is too large: {residual:.4f} cycles")
    if len(candidates) > 1 and abs(candidates[1][0] - candidates[0][0]) < 1e-6:
        raise ValueError("3/7 ms phase result is ambiguous")
    confidence = min(confidence_3ms, confidence_7ms)
    confidence *= max(0.0, 1.0 - residual / maximum_residual_cycles)
    return FineFrequencyFit(
        frequency, frequency - coarse_frequency_hz, residual,
        float(np.clip(confidence, 0.0, 1.0)))


@dataclass(frozen=True)
class TargetFit:
    estimated_phase: int
    desired_score: float
    quality: int
    span_x_div: float
    span_y_div: float
    center_error_div: float


@dataclass(frozen=True)
class ReferenceCalibration:
    top_y: float
    bottom_y: float
    left_x: float
    right_x: float
    top_band: tuple[int, int]
    bottom_band: tuple[int, int]
    confidence: float


@dataclass(frozen=True)
class WaveformPoint:
    x_px: float
    y_px: float
    x_normalized: float
    y_normalized: float
    y_volts: float
    time_normalized: float
    strength: float


@dataclass
class WaveformPointResult:
    calibration: ReferenceCalibration
    points: list[WaveformPoint]
    trace_mask: np.ndarray


class TraceExtractor:
    def __init__(self, config: dict[str, Any]) -> None:
        vision = config.get("vision", {})
        self._hsv_low = np.asarray(vision.get("hsv_low", [25, 60, 90]), np.uint8)
        self._hsv_high = np.asarray(vision.get("hsv_high", [100, 255, 255]), np.uint8)
        self._minimum_pixels = int(vision.get("minimum_trace_pixels", 150))
        self._brightness_threshold = int(vision.get("brightness_threshold", 165))

    def extract(self, frame: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        color_mask = cv2.inRange(hsv, self._hsv_low, self._hsv_high)
        if cv2.countNonZero(color_mask) >= self._minimum_pixels:
            mask = color_mask
        else:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            threshold = max(self._brightness_threshold,
                            int(np.percentile(gray, 97.5)))
            mask = cv2.inRange(gray, threshold, 255)

        height, width = mask.shape
        if cv2.countNonZero(mask) > height * width // 8:
            horizontal = cv2.morphologyEx(
                mask, cv2.MORPH_OPEN,
                cv2.getStructuringElement(cv2.MORPH_RECT, (max(15, width // 12), 1)))
            vertical = cv2.morphologyEx(
                mask, cv2.MORPH_OPEN,
                cv2.getStructuringElement(cv2.MORPH_RECT, (1, max(15, height // 10))))
            mask = cv2.subtract(mask, cv2.bitwise_or(horizontal, vertical))

        mask = cv2.morphologyEx(
            mask, cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2)))
        return mask


class WaveformPointExtractor:
    """Extract sparse points from the Task5 pulse-ramp XY pattern.

    The two bright idle lines are useful calibration features rather than
    waveform samples. They define Y=+2 V, Y=-2 V, and the full horizontal X
    sweep. The extractor masks those bands, enhances the remaining phosphor
    trace, finds separated row-activity peaks, and returns one weighted point
    for each visible pulse-ramp sample group.
    """

    def __init__(self, config: dict[str, Any] | None = None) -> None:
        vision = (config or {}).get("vision", {})
        settings = vision.get("point_extraction", {})
        self._reference_search_fraction = float(
            settings.get("reference_search_fraction", 0.35))
        self._reference_margin_px = int(
            settings.get("reference_margin_px", 8))
        self._horizontal_crop_fraction = float(
            settings.get("horizontal_crop_fraction", 0.05))
        self._activity_fraction = float(
            settings.get("activity_fraction", 0.04))
        self._peak_floor_percentile = float(
            settings.get("peak_floor_percentile", 58.0))
        self._profile_percentile = float(
            settings.get("profile_percentile", 85.0))
        self._maximum_points = int(settings.get("maximum_points", 64))
        self._minimum_points = int(settings.get("minimum_points", 5))
        self._minimum_green_excess = float(
            settings.get("minimum_green_excess", 30.0))
        self._minimum_brightness = float(
            settings.get("minimum_brightness", 155.0))
        self._minimum_reference_green_excess = float(
            settings.get("minimum_reference_green_excess", 35.0))
        self._minimum_reference_contrast = float(
            settings.get("minimum_reference_contrast", 12.0))
        self._minimum_reference_confidence = float(
            settings.get("minimum_reference_confidence", 0.55))

    @staticmethod
    def _green_excess(frame: np.ndarray) -> np.ndarray:
        blue, green, red = cv2.split(frame.astype(np.int16))
        return np.clip(green - np.maximum(blue, red), 0, 255).astype(np.uint8)

    @staticmethod
    def _top_fraction_mean(values: np.ndarray, fraction: float,
                           axis: int) -> np.ndarray:
        count = max(1, int(round(values.shape[axis] * fraction)))
        partitioned = np.partition(values, -count, axis=axis)
        indices = [slice(None)] * values.ndim
        indices[axis] = slice(-count, None)
        return np.mean(partitioned[tuple(indices)], axis=axis)

    @staticmethod
    def _smooth_1d(values: np.ndarray, size: int) -> np.ndarray:
        size = max(3, int(size) | 1)
        return cv2.GaussianBlur(
            values.astype(np.float32).reshape(-1, 1), (1, size), 0
        ).reshape(-1)

    @staticmethod
    def _reference_band(activity: np.ndarray, start: int,
                        stop: int) -> tuple[int, int, float, float]:
        region = activity[start:stop]
        if region.size < 3:
            raise ValueError("reference search region is too small")
        peak = start + int(np.argmax(region))
        baseline = float(np.percentile(region, 35.0))
        peak_value = float(activity[peak])
        threshold = baseline + 0.35 * max(0.0, peak_value - baseline)
        lower = peak
        upper = peak
        while lower > start and activity[lower - 1] >= threshold:
            lower -= 1
        while upper + 1 < stop and activity[upper + 1] >= threshold:
            upper += 1
        rows = np.arange(lower, upper + 1, dtype=np.float32)
        weights = np.maximum(activity[lower:upper + 1] - baseline, 0.0)
        center = (float(np.sum(rows * weights) / np.sum(weights))
                  if float(np.sum(weights)) > 0.0 else float(peak))
        prominence = ((peak_value - baseline) /
                      max(1.0, float(np.percentile(region, 95.0))))
        return lower, upper, center, max(0.0, prominence)

    def detect_reference_lines(self, frame: np.ndarray) -> ReferenceCalibration:
        score = self._green_excess(frame).astype(np.float32)
        height, width = score.shape
        x_margin = max(2, int(round(width * self._horizontal_crop_fraction)))
        row_source = score[:, x_margin:width - x_margin]
        row_activity = self._top_fraction_mean(row_source, 0.35, axis=1)
        row_activity = self._smooth_1d(row_activity, max(5, height // 58))

        search = min(0.48, max(0.2, self._reference_search_fraction))
        top_start = max(0, int(round(height * 0.015)))
        top_stop = max(top_start + 3, int(round(height * search)))
        bottom_start = min(height - 3, int(round(height * (1.0 - search))))
        bottom_stop = min(height, int(round(height * 0.985)))
        top_lower, top_upper, top_y, top_prominence = self._reference_band(
            row_activity, top_start, top_stop)
        bottom_lower, bottom_upper, bottom_y, bottom_prominence = (
            self._reference_band(row_activity, bottom_start, bottom_stop))
        if bottom_y - top_y < height * 0.45:
            raise ValueError("upper and lower reference lines are too close")

        reference_rows = np.concatenate((
            np.arange(top_lower, top_upper + 1),
            np.arange(bottom_lower, bottom_upper + 1),
        ))
        reference_pixels = score[reference_rows, :]
        baseline = float(np.percentile(reference_pixels, 35.0))
        bright = float(np.percentile(reference_pixels, 98.0))
        reference_level = float(np.percentile(reference_pixels, 75.0))
        if (reference_level < self._minimum_reference_green_excess or
                bright - baseline < self._minimum_reference_contrast):
            raise ValueError(
                "the +/-2 V reference lines are missing or too dim; "
                "shorten the Task5 ramp below 10 ms and lock exposure")
        threshold = baseline + 0.35 * max(1.0, bright - baseline)
        coverage = np.mean(reference_pixels >= threshold, axis=0)
        columns = np.flatnonzero(coverage >= 0.12)
        if columns.size >= max(20, width // 5):
            left_x, right_x = np.percentile(columns, [1.0, 99.0])
        else:
            raise ValueError(
                "the +/-2 V reference lines do not have enough horizontal "
                "coverage")
        if right_x - left_x < width * 0.35:
            raise ValueError("reference lines do not span enough screen width")

        separation_score = min(1.0, (bottom_y - top_y) / (height * 0.7))
        span_score = min(1.0, (right_x - left_x) / (width * 0.65))
        confidence = min(1.0, 0.35 * top_prominence +
                         0.35 * bottom_prominence +
                         0.15 * separation_score + 0.15 * span_score)
        if confidence < self._minimum_reference_confidence:
            raise ValueError(
                f"reference-line confidence is too low: {confidence:.3f}")
        return ReferenceCalibration(
            top_y=top_y,
            bottom_y=bottom_y,
            left_x=float(left_x),
            right_x=float(right_x),
            top_band=(top_lower, top_upper),
            bottom_band=(bottom_lower, bottom_upper),
            confidence=float(confidence),
        )

    @staticmethod
    def _runs(profile: np.ndarray, threshold: float) -> list[tuple[int, int]]:
        active = profile >= threshold
        padded = np.pad(active.astype(np.int8), (1, 1))
        changes = np.diff(padded)
        starts = np.flatnonzero(changes == 1)
        stops = np.flatnonzero(changes == -1)
        return list(zip(starts.tolist(), stops.tolist()))

    def extract(self, frame: np.ndarray,
                maximum_points: int | None = None) -> WaveformPointResult:
        calibration = self.detect_reference_lines(frame)
        score = self._green_excess(frame)
        brightness = np.max(frame, axis=2).astype(np.uint8)
        height, width = score.shape

        kernel_size = max(15, int(round(min(height, width) * 0.045)) | 1)
        background = cv2.morphologyEx(
            score,
            cv2.MORPH_OPEN,
            cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)),
        )
        enhanced = cv2.subtract(score, background).astype(np.float32)

        y_start = min(height - 2, calibration.top_band[1] +
                      self._reference_margin_px + 1)
        y_stop = max(y_start + 2, calibration.bottom_band[0] -
                     self._reference_margin_px)
        horizontal_span = calibration.right_x - calibration.left_x
        x_start = max(1, int(round(calibration.left_x -
                                  horizontal_span * 0.03)))
        x_stop = min(width - 1, int(round(calibration.right_x +
                                         horizontal_span * 0.03)))
        if y_stop - y_start < 20 or x_stop - x_start < 20:
            raise ValueError("reference-line mask leaves too little trace area")

        valid_values = enhanced[y_start:y_stop, x_start:x_stop]
        valid_green = score[y_start:y_stop, x_start:x_stop]
        valid_brightness = brightness[y_start:y_stop, x_start:x_stop]
        binary_threshold = max(2.0, float(np.percentile(valid_values, 94.0)))
        trace_mask = np.zeros((height, width), np.uint8)
        trace_mask[y_start:y_stop, x_start:x_stop] = np.where(
            (valid_values >= binary_threshold) &
            (valid_green >= self._minimum_green_excess) &
            (valid_brightness >= self._minimum_brightness),
            255, 0).astype(np.uint8)
        trace_mask = cv2.morphologyEx(
            trace_mask, cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_RECT, (3, 1)))

        row_source = enhanced[:, x_start:x_stop]
        row_activity = self._top_fraction_mean(
            row_source, self._activity_fraction, axis=1)
        row_activity = self._smooth_1d(row_activity, max(3, height // 128))
        active_rows = row_activity[y_start:y_stop]
        peak_floor = float(np.percentile(
            active_rows, self._peak_floor_percentile))
        candidates: list[tuple[float, int]] = []
        for row in range(y_start + 1, y_stop - 1):
            value = float(row_activity[row])
            if (value >= peak_floor and
                    value >= float(row_activity[row - 1]) and
                    value >= float(row_activity[row + 1])):
                candidates.append((value, row))

        point_limit = max(1, int(maximum_points or self._maximum_points))
        minimum_spacing = max(4, height // 80)
        selected_rows: list[tuple[float, int]] = []
        for value, row in sorted(candidates, reverse=True):
            if all(abs(row - selected) > minimum_spacing
                   for _, selected in selected_rows):
                selected_rows.append((value, row))
        selected_rows.sort(key=lambda item: item[1])

        points: list[WaveformPoint] = []
        band_radius = max(1, height // 256)
        minimum_run_width = max(2, width // 400)
        activity_high = max(peak_floor + 1e-6,
                            float(np.percentile(active_rows, 95.0)))
        for row_value, row in selected_rows:
            profile = np.max(
                enhanced[max(y_start, row - band_radius):
                         min(y_stop, row + band_radius + 1),
                         x_start:x_stop],
                axis=0,
            )
            green_profile = np.max(
                score[max(y_start, row - band_radius):
                      min(y_stop, row + band_radius + 1),
                      x_start:x_stop],
                axis=0,
            )
            brightness_profile = np.max(
                brightness[max(y_start, row - band_radius):
                           min(y_stop, row + band_radius + 1),
                           x_start:x_stop],
                axis=0,
            )
            profile = np.where(
                (green_profile >= self._minimum_green_excess) &
                (brightness_profile >= self._minimum_brightness),
                profile, 0.0)
            profile_threshold = max(
                2.0, float(np.percentile(profile, self._profile_percentile)))
            runs = self._runs(profile, profile_threshold)
            scored_runs: list[tuple[float, float, int]] = []
            for run_start, run_stop in runs:
                run_width = run_stop - run_start
                if run_width < minimum_run_width:
                    continue
                values = profile[run_start:run_stop]
                weights = np.maximum(values - profile_threshold + 1.0, 1.0)
                columns = np.arange(run_start, run_stop, dtype=np.float32)
                power = float(np.sum(values))
                center = float(np.sum(columns * weights) / np.sum(weights))
                scored_runs.append((power, center, run_width))
            if not scored_runs:
                continue
            scored_runs.sort(reverse=True)
            best_power, center, _ = scored_runs[0]
            competing_power = sum(item[0] for item in scored_runs[:3])
            dominance = best_power / max(best_power, competing_power)
            peak_strength = (row_value - peak_floor) / (activity_high - peak_floor)
            strength = float(np.clip(
                0.55 * peak_strength + 0.45 * dominance, 0.0, 1.0))
            x_px = center + x_start
            y_px = float(row)
            x_normalized = 2.0 * (x_px - calibration.left_x) / horizontal_span - 1.0
            y_normalized = 1.0 - 2.0 * (
                y_px - calibration.top_y) / (
                    calibration.bottom_y - calibration.top_y)
            points.append(WaveformPoint(
                x_px=x_px,
                y_px=y_px,
                x_normalized=float(np.clip(x_normalized, -1.25, 1.25)),
                y_normalized=float(np.clip(y_normalized, -1.1, 1.1)),
                y_volts=float(np.clip(2.0 * y_normalized, -2.2, 2.2)),
                time_normalized=float(np.clip(
                    (y_normalized + 1.0) * 0.5, 0.0, 1.0)),
                strength=strength,
            ))

        # The ramp starts at -2 V and rises to +2 V, so descending image Y is
        # the real time order. CSV/JSON indices therefore run from ramp start
        # to ramp end instead of top-to-bottom screen order.
        points.sort(key=lambda point: point.y_px, reverse=True)
        if len(points) > point_limit:
            # Apply the limit only after color/brightness validation. Limiting
            # candidate rows earlier can select a rejected grid edge and leave
            # fewer points than the caller requested. Even spacing here keeps
            # the complete -2 V to +2 V time range represented.
            sample_indices = np.linspace(
                0, len(points) - 1, point_limit).round().astype(int)
            points = [points[index] for index in np.unique(sample_indices)]
        required_points = min(self._minimum_points, point_limit)
        if len(points) < required_points:
            raise ValueError(
                f"only {len(points)} waveform points found; check focus, "
                "exposure, perspective points, and green thresholds")
        return WaveformPointResult(calibration, points, trace_mask)

    @staticmethod
    def render_overlay(frame: np.ndarray,
                       result: WaveformPointResult) -> np.ndarray:
        overlay = frame.copy()
        calibration = result.calibration
        cv2.line(overlay, (round(calibration.left_x), round(calibration.top_y)),
                 (round(calibration.right_x), round(calibration.top_y)),
                 (0, 80, 255), 2)
        cv2.line(overlay, (round(calibration.left_x), round(calibration.bottom_y)),
                 (round(calibration.right_x), round(calibration.bottom_y)),
                 (255, 80, 0), 2)
        for index, point in enumerate(result.points):
            center = (round(point.x_px), round(point.y_px))
            cv2.circle(overlay, center, 5, (0, 0, 255), -1, cv2.LINE_AA)
            cv2.putText(overlay, str(index), (center[0] + 6, center[1] - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (20, 20, 255), 1,
                        cv2.LINE_AA)
        cv2.putText(overlay, "+2 V reference",
                    (round(calibration.left_x),
                     max(14, round(calibration.top_y) - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 80, 255), 1,
                    cv2.LINE_AA)
        cv2.putText(overlay, "-2 V reference",
                    (round(calibration.left_x),
                     min(frame.shape[0] - 5, round(calibration.bottom_y) + 18)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 80, 0), 1,
                    cv2.LINE_AA)
        return overlay


class FrequencyEstimator:
    WIDTHS_US = {0: 100.0, 1: 500.0, 2: 2000.0, 3: 5000.0}

    @staticmethod
    def _centerline(mask: np.ndarray, y_top: float, y_bottom: float) -> tuple[np.ndarray, np.ndarray]:
        height, width = mask.shape
        top = max(0, min(height - 1, int(round(y_top * height))))
        bottom = max(top + 1, min(height, int(round(y_bottom * height))))
        q_values: list[float] = []
        x_values: list[float] = []
        span = max(1, bottom - top - 1)
        for row in range(top, bottom):
            columns = np.flatnonzero(mask[row])
            columns = columns[(columns > width * 0.02) & (columns < width * 0.98)]
            if columns.size:
                q_values.append((bottom - 1 - row) / span)
                x_values.append(float(np.median(columns)) / max(1, width - 1))
        if len(q_values) < 30:
            raise ValueError("not enough trace rows for sinusoid fitting")
        order = np.argsort(q_values)
        return np.asarray(q_values)[order], np.asarray(x_values)[order]

    @staticmethod
    def _fit(q: np.ndarray, x: np.ndarray, minimum_cycles: float,
             maximum_cycles: float, expected_cycles: float | None = None) -> ProbeFit:
        if expected_cycles is not None:
            minimum_cycles = max(minimum_cycles, expected_cycles - 0.8)
            maximum_cycles = min(maximum_cycles, expected_cycles + 0.8)
        if maximum_cycles <= minimum_cycles:
            raise ValueError("invalid cycle search range")

        centered = x - np.mean(x)
        trend = np.polyfit(q, centered, 1)
        centered = centered - np.polyval(trend, q)
        coarse = np.arange(minimum_cycles, maximum_cycles + 0.0101, 0.01)
        best_cycle = float(coarse[0])
        best_power = -1.0
        for start in range(0, coarse.size, 256):
            candidates = coarse[start:start + 256]
            angle = 2.0 * pi * candidates[:, None] * q[None, :]
            projection = np.exp(-1j * angle) @ centered
            powers = np.abs(projection) ** 2
            index = int(np.argmax(powers))
            if float(powers[index]) > best_power:
                best_power = float(powers[index])
                best_cycle = float(candidates[index])

        fine = np.arange(max(minimum_cycles, best_cycle - 0.025),
                         min(maximum_cycles, best_cycle + 0.025) + 0.000251,
                         0.00025)
        best_residual = float("inf")
        best_coefficients: np.ndarray | None = None
        for cycles in fine:
            angle = 2.0 * pi * cycles * q
            matrix = np.column_stack((np.ones_like(q), q, np.sin(angle), np.cos(angle)))
            coefficients, _, _, _ = np.linalg.lstsq(matrix, x, rcond=None)
            residual = float(np.mean((x - matrix @ coefficients) ** 2))
            if residual < best_residual:
                best_residual = residual
                best_cycle = float(cycles)
                best_coefficients = coefficients

        assert best_coefficients is not None
        signal_std = max(1e-6, float(np.std(x)))
        confidence = max(0.0, min(1.0, 1.0 - best_residual ** 0.5 / signal_std))
        phase = float(np.arctan2(best_coefficients[3], best_coefficients[2]))
        return ProbeFit(best_cycle, phase, confidence, int(q.size))

    def estimate_single(self, mask: np.ndarray, width_code: int) -> ProbeFit:
        q, x = self._centerline(mask, 0.05, 0.95)
        maximum = {0: 12.0, 1: 55.0, 2: 205.0, 3: 505.0}.get(width_code, 55.0)
        return self._fit(q, x, 0.05, maximum)

    def estimate_dual_phase(self, mask: np.ndarray, width_code: int,
                            coarse_frequency_hz: float) -> DualPhaseFit:
        width_us = self.WIDTHS_US[width_code]
        expected_cycles = coarse_frequency_hz * width_us / 1_000_000.0
        q_a, x_a = self._centerline(mask, 0.55, 0.95)
        q_b, x_b = self._centerline(mask, 0.05, 0.45)
        fit_a = self._fit(q_a, x_a, 0.05, 205.0, expected_cycles)
        fit_b = self._fit(q_b, x_b, 0.05, 205.0, expected_cycles)
        phase_cycles = wrap_cycles(
            (fit_b.phase_radians - fit_a.phase_radians) / (2.0 * pi))
        return DualPhaseFit(
            phase_cycles, min(fit_a.confidence, fit_b.confidence), fit_a, fit_b)

    def estimate_dual(self, mask: np.ndarray, width_code: int,
                      coarse_frequency_hz: float, offset_us: int) -> DualProbeFit:
        phase_fit = self.estimate_dual_phase(
            mask, width_code, coarse_frequency_hz)
        fit_a = phase_fit.fit_a
        fit_b = phase_fit.fit_b
        phase_cycles = phase_fit.phase_difference_cycles
        coarse_grid = int(round(coarse_frequency_hz / 100.0) * 100)
        candidates = range(max(1000, coarse_grid - 600),
                           min(100000, coarse_grid + 600) + 1, 100)

        def candidate_error(frequency: int) -> float:
            expected_fraction = (frequency * offset_us / 1_000_000.0 + 0.5) % 1.0 - 0.5
            circular = abs((expected_fraction - phase_cycles + 0.5) % 1.0 - 0.5)
            coarse_penalty = abs(frequency - coarse_frequency_hz) / 5000.0
            return circular + coarse_penalty

        grid_frequency = min(candidates, key=candidate_error)
        expected_offset_cycles = grid_frequency * offset_us / 1_000_000.0
        integer_cycles = round(expected_offset_cycles - phase_cycles)
        measured_offset_cycles = integer_cycles + phase_cycles
        offset_clock_cycles = offset_us * 50
        tuning_word = int(round(measured_offset_cycles * (2**32) / offset_clock_cycles))
        nominal_word = int(round(grid_frequency * (2**32) / 50_000_000.0))
        if tuning_word <= 0 or abs(tuning_word - nominal_word) > nominal_word * 0.002:
            tuning_word = nominal_word

        confidence = min(fit_a.confidence, fit_b.confidence)
        confidence *= max(0.0, 1.0 - candidate_error(grid_frequency) * 1.5)
        return DualProbeFit(grid_frequency, tuning_word, phase_cycles,
                            confidence, fit_a, fit_b)


class TargetAnalyzer:
    def __init__(self, config: dict[str, Any]) -> None:
        vision = config.get("vision", {})
        self._phase_step = max(1, int(vision.get("phase_search_step", 4)))

    @staticmethod
    def _model_points(shape: int, phase: int, center_x: float, center_y: float,
                      amplitude_x: float, amplitude_y: float) -> np.ndarray:
        parameter = np.linspace(0.0, 2.0 * pi, 1400, endpoint=False)
        ratio = 2 if shape == 3 else 1
        phase_radians = phase * 2.0 * pi / 256.0
        x = center_x + amplitude_x * np.sin(parameter)
        y = center_y - amplitude_y * np.sin(ratio * parameter + phase_radians)
        return np.column_stack((x, y)).round().astype(np.int32)

    @staticmethod
    def _chamfer(distance: np.ndarray, points: np.ndarray) -> float:
        height, width = distance.shape
        valid = ((points[:, 0] >= 0) & (points[:, 0] < width) &
                 (points[:, 1] >= 0) & (points[:, 1] < height))
        points = points[valid]
        if not points.size:
            return 1.0
        return float(np.mean(distance[points[:, 1], points[:, 0]]) / min(width, height))

    def analyze(self, mask: np.ndarray, shape: int) -> TargetFit:
        rows, columns = np.nonzero(mask)
        if rows.size < 100:
            raise ValueError("not enough target trace pixels")
        x_low, x_high = np.percentile(columns, [1.0, 99.0])
        y_low, y_high = np.percentile(rows, [1.0, 99.0])
        center_x = (x_low + x_high) * 0.5
        center_y = (y_low + y_high) * 0.5
        amplitude_x = max(4.0, (x_high - x_low) * 0.5)
        amplitude_y = max(4.0, (y_high - y_low) * 0.5)

        inverse = cv2.bitwise_not(mask)
        distance = cv2.distanceTransform(inverse, cv2.DIST_L2, 3)
        phase_scores: list[tuple[float, int]] = []
        for phase in range(0, 256, self._phase_step):
            points = self._model_points(shape, phase, center_x, center_y,
                                        amplitude_x, amplitude_y)
            phase_scores.append((self._chamfer(distance, points), phase))
        best_score, best_phase = min(phase_scores)

        desired_phases = (64, 192) if shape == 2 else (0, 128)
        desired_score = min(
            self._chamfer(distance, self._model_points(
                shape, phase, center_x, center_y, amplitude_x, amplitude_y))
            for phase in desired_phases
        )
        height, width = mask.shape
        span_x_div = (x_high - x_low) / (width / 10.0)
        span_y_div = (y_high - y_low) / (height / 8.0)
        center_error = (((center_x - width * 0.5) / (width / 10.0)) ** 2 +
                        ((center_y - height * 0.5) / (height / 8.0)) ** 2) ** 0.5
        shape_quality = max(0.0, 1.0 - desired_score / 0.055)
        amplitude_quality = max(0.0, 1.0 - abs(span_y_div - 8.0) / 4.0)
        quality = int(round(100.0 * (0.8 * shape_quality + 0.2 * amplitude_quality)))
        return TargetFit(best_phase, desired_score, max(0, min(100, quality)),
                         float(span_x_div), float(span_y_div), float(center_error))


def aggregate_masks(masks: list[np.ndarray]) -> np.ndarray:
    if not masks:
        raise ValueError("at least one mask is required")
    stack = np.stack([(mask > 0).astype(np.uint8) for mask in masks], axis=0)
    required = max(1, (len(masks) + 1) // 2)
    return ((np.sum(stack, axis=0) >= required) * 255).astype(np.uint8)



# ========== 圆形检测模块 ==========

# -*- coding: utf-8 -*-
"""圆形检测与质量评估模块

用于第五问自动锁圆功能，检测李萨如图形是否为圆形，
并评估圆度质量以判断是否达到锁定条件。
"""


from dataclasses import dataclass
from typing import Any



@dataclass(frozen=True)
class CircleQuality:
    """圆形质量评估结果"""
    is_circle: bool           # 是否检测为圆形
    circularity: float        # 圆度 (0.0-1.0)
    quality_score: int        # 综合质量分数 (0-100)
    radius_ratio: float       # 长短轴比 (接近1.0为圆)
    center_x: float           # 圆心X坐标（归一化）
    center_y: float           # 圆心Y坐标（归一化）
    coverage: float           # 轨迹覆盖率
    symmetry: float           # 对称性评分
    reason: str               # 判断原因


class CircleDetector:
    """圆形检测器

    检测李萨如图形是否为圆形，评估圆度质量。
    当两个正弦波频率相同且相位差为90度时，形成圆形。
    """

    def __init__(self, config: dict[str, Any]) -> None:
        circle_cfg = config.get("circle_detection", {})

        # 圆度阈值
        self.min_circularity = float(circle_cfg.get("min_circularity", 0.75))
        self.good_circularity = float(circle_cfg.get("good_circularity", 0.85))

        # 轴比阈值（长轴/短轴应接近1.0）
        self.max_axis_ratio = float(circle_cfg.get("max_axis_ratio", 1.25))
        self.good_axis_ratio = float(circle_cfg.get("good_axis_ratio", 1.10))

        # 覆盖率阈值（轨迹应覆盖完整圆周）
        self.min_coverage = float(circle_cfg.get("min_coverage", 0.70))
        self.good_coverage = float(circle_cfg.get("good_coverage", 0.85))

        # 对称性阈值
        self.min_symmetry = float(circle_cfg.get("min_symmetry", 0.65))

        # 质量评分阈值
        self.lock_quality_threshold = int(circle_cfg.get("lock_quality_threshold", 75))

    def detect_circle(self, mask: np.ndarray) -> CircleQuality:
        """检测掩膜中的圆形并评估质量

        Args:
            mask: 轨迹掩膜 (H x W 的二值图像)

        Returns:
            CircleQuality: 圆形质量评估结果
        """
        if mask.size == 0 or np.count_nonzero(mask) < 50:
            return CircleQuality(
                False, 0.0, 0, 0.0, 0.5, 0.5, 0.0, 0.0,
                "insufficient trace pixels"
            )

        height, width = mask.shape[:2]

        # 提取轮廓
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
        )

        if not contours:
            return CircleQuality(
                False, 0.0, 0, 0.0, 0.5, 0.5, 0.0, 0.0,
                "no contours found"
            )

        # 使用最大轮廓
        main_contour = max(contours, key=cv2.contourArea)

        if len(main_contour) < 10:
            return CircleQuality(
                False, 0.0, 0, 0.0, 0.5, 0.5, 0.0, 0.0,
                "contour too small"
            )

        # 拟合椭圆（需要至少5个点）
        if len(main_contour) < 5:
            return CircleQuality(
                False, 0.0, 0, 0.0, 0.5, 0.5, 0.0, 0.0,
                "insufficient points for ellipse fit"
            )

        try:
            ellipse = cv2.fitEllipse(main_contour)
            center, axes, angle = ellipse
            major_axis = max(axes)
            minor_axis = min(axes)
        except cv2.error:
            return CircleQuality(
                False, 0.0, 0, 0.0, 0.5, 0.5, 0.0, 0.0,
                "ellipse fit failed"
            )

        # 计算轴比
        if minor_axis > 0:
            axis_ratio = major_axis / minor_axis
        else:
            axis_ratio = 10.0

        # 计算圆度（使用周长和面积）
        perimeter = cv2.arcLength(main_contour, True)
        area = cv2.contourArea(main_contour)

        if perimeter > 0:
            # 圆形的圆度 = 4π * 面积 / 周长^2，完美圆形为1.0
            circularity = (4.0 * np.pi * area) / (perimeter * perimeter)
            circularity = min(1.0, circularity)
        else:
            circularity = 0.0

        # 归一化圆心坐标
        center_x_norm = center[0] / max(width, 1.0)
        center_y_norm = center[1] / max(height, 1.0)

        # 计算覆盖率（轨迹点在拟合椭圆附近的比例）
        coverage = self._calculate_coverage(main_contour, ellipse)

        # 计算对称性
        symmetry = self._calculate_symmetry(mask, center)

        # 综合评分
        quality_score = self._calculate_quality_score(
            circularity, axis_ratio, coverage, symmetry
        )

        # 判断是否为圆形
        is_circle = (
            circularity >= self.min_circularity and
            axis_ratio <= self.max_axis_ratio and
            coverage >= self.min_coverage and
            symmetry >= self.min_symmetry
        )

        # 生成原因说明
        if not is_circle:
            reasons = []
            if circularity < self.min_circularity:
                reasons.append(f"circularity {circularity:.3f} < {self.min_circularity}")
            if axis_ratio > self.max_axis_ratio:
                reasons.append(f"axis_ratio {axis_ratio:.3f} > {self.max_axis_ratio}")
            if coverage < self.min_coverage:
                reasons.append(f"coverage {coverage:.3f} < {self.min_coverage}")
            if symmetry < self.min_symmetry:
                reasons.append(f"symmetry {symmetry:.3f} < {self.min_symmetry}")
            reason = "; ".join(reasons)
        else:
            reason = "circle detected"

        return CircleQuality(
            is_circle=is_circle,
            circularity=float(circularity),
            quality_score=quality_score,
            radius_ratio=float(axis_ratio),
            center_x=float(center_x_norm),
            center_y=float(center_y_norm),
            coverage=float(coverage),
            symmetry=float(symmetry),
            reason=reason
        )

    def _calculate_coverage(
        self, contour: np.ndarray, ellipse: tuple
    ) -> float:
        """计算轨迹在拟合椭圆上的覆盖率"""
        center, axes, angle = ellipse
        cx, cy = center
        a, b = axes[0] / 2.0, axes[1] / 2.0

        if a <= 0 or b <= 0:
            return 0.0

        # 将轮廓点转换到椭圆坐标系
        angle_rad = np.deg2rad(angle)
        cos_a = np.cos(angle_rad)
        sin_a = np.sin(angle_rad)

        points = contour.reshape(-1, 2).astype(np.float64)
        dx = points[:, 0] - cx
        dy = points[:, 1] - cy

        # 旋转到椭圆主轴
        x_rot = dx * cos_a + dy * sin_a
        y_rot = -dx * sin_a + dy * cos_a

        # 计算点到椭圆的归一化距离
        distances = np.sqrt((x_rot / a) ** 2 + (y_rot / b) ** 2)

        # 在椭圆附近的点（距离在0.8到1.2之间）
        on_ellipse = np.sum((distances >= 0.8) & (distances <= 1.2))

        return float(on_ellipse) / max(len(points), 1.0)

    def _calculate_symmetry(
        self, mask: np.ndarray, center: tuple[float, float]
    ) -> float:
        """计算图形的对称性"""
        height, width = mask.shape[:2]
        cx, cy = int(center[0]), int(center[1])

        # 确保中心在图像内
        if not (0 <= cx < width and 0 <= cy < height):
            return 0.0

        # 计算四个象限的对称性
        radius = min(cx, cy, width - cx, height - cy)
        if radius < 10:
            return 0.0

        # 水平对称性
        left_region = mask[max(0, cy - radius):min(height, cy + radius),
                           max(0, cx - radius):cx]
        right_region = mask[max(0, cy - radius):min(height, cy + radius),
                            cx:min(width, cx + radius)]
        right_flipped = cv2.flip(right_region, 1)

        # 匹配左右区域
        min_width = min(left_region.shape[1], right_flipped.shape[1])
        if min_width > 0:
            h_match = np.sum(
                left_region[:, -min_width:] == right_flipped[:, :min_width]
            )
            h_total = left_region[:, -min_width:].size
            h_symmetry = h_match / max(h_total, 1.0)
        else:
            h_symmetry = 0.0

        # 垂直对称性
        top_region = mask[max(0, cy - radius):cy,
                          max(0, cx - radius):min(width, cx + radius)]
        bottom_region = mask[cy:min(height, cy + radius),
                             max(0, cx - radius):min(width, cx + radius)]
        bottom_flipped = cv2.flip(bottom_region, 0)

        min_height = min(top_region.shape[0], bottom_flipped.shape[0])
        if min_height > 0:
            v_match = np.sum(
                top_region[-min_height:, :] == bottom_flipped[:min_height, :]
            )
            v_total = top_region[-min_height:, :].size
            v_symmetry = v_match / max(v_total, 1.0)
        else:
            v_symmetry = 0.0

        # 综合对称性
        return float((h_symmetry + v_symmetry) / 2.0)

    def _calculate_quality_score(
        self,
        circularity: float,
        axis_ratio: float,
        coverage: float,
        symmetry: float
    ) -> int:
        """计算综合质量分数 (0-100)"""
        # 圆度评分 (40分)
        circ_score = circularity * 40.0

        # 轴比评分 (25分)
        # 完美圆形轴比为1.0，超过阈值线性下降
        if axis_ratio <= 1.0:
            axis_score = 25.0
        elif axis_ratio <= self.good_axis_ratio:
            axis_score = 25.0 * (self.good_axis_ratio - axis_ratio) / (
                self.good_axis_ratio - 1.0
            )
        else:
            axis_score = max(0.0, 25.0 * (self.max_axis_ratio - axis_ratio) / (
                self.max_axis_ratio - self.good_axis_ratio
            ))

        # 覆盖率评分 (20分)
        coverage_score = coverage * 20.0

        # 对称性评分 (15分)
        symmetry_score = symmetry * 15.0

        total = circ_score + axis_score + coverage_score + symmetry_score
        return int(np.clip(total, 0, 100))



# ========== 频率扫描模块 ==========

# -*- coding: utf-8 -*-
"""频率扫描模块

实现100Hz步进的频率扫描，用于第五问自动锁圆功能。
在粗测频率基础上进行精细扫频，寻找最佳圆形。
"""


from dataclasses import dataclass
from typing import Any




@dataclass(frozen=True)
class ScanResult:
    """扫描结果"""
    frequency_hz: float       # 最佳频率
    quality_score: int        # 质量分数
    tuning_word: int          # DDS调谐字
    scan_count: int           # 扫描次数
    found_circle: bool        # 是否找到圆形
    reason: str               # 结果说明


class FrequencyScanner:
    """频率扫描器

    从粗测频率开始，以100Hz为步进进行扫频，
    寻找能够形成最佳圆形的频率点。
    """

    def __init__(self, config: dict[str, Any]) -> None:
        scan_cfg = config.get("frequency_scan", {})

        # 扫描参数
        self.step_hz = float(scan_cfg.get("step_hz", 100.0))  # 步进频率
        self.initial_range_hz = float(scan_cfg.get("initial_range_hz", 500.0))
        self.extended_range_hz = float(scan_cfg.get("extended_range_hz", 1000.0))
        self.max_scan_points = int(scan_cfg.get("max_scan_points", 20))

        # 质量阈值
        self.min_quality_for_lock = int(scan_cfg.get("min_quality_for_lock", 75))
        self.good_quality_threshold = int(scan_cfg.get("good_quality_threshold", 85))

        # DDS参数
        self.dds_clock_hz = float(scan_cfg.get("dds_clock_hz", 50_000_000.0))

        # 扫描策略
        self.scan_strategy = scan_cfg.get("scan_strategy", "bidirectional")

    def plan_scan_points(
        self,
        coarse_frequency_hz: float,
        uncertainty_hz: float = 500.0
    ) -> list[float]:
        """规划扫描点序列

        Args:
            coarse_frequency_hz: 粗测频率
            uncertainty_hz: 不确定度

        Returns:
            扫描频率点列表（已按100Hz对齐）
        """
        # 确保粗测频率对齐到100Hz
        center_freq = self._align_to_grid(coarse_frequency_hz)

        # 初始扫描范围
        scan_range = max(uncertainty_hz, self.initial_range_hz)
        half_range = scan_range / 2.0

        # 生成扫描点
        points = []

        if self.scan_strategy == "bidirectional":
            # 双向扫描：中心 -> +step -> -step -> +2*step -> -2*step ...
            points.append(center_freq)

            offset = self.step_hz
            while offset <= half_range and len(points) < self.max_scan_points:
                if center_freq + offset <= 100_000.0:
                    points.append(center_freq + offset)
                if center_freq - offset >= 1000.0 and len(points) < self.max_scan_points:
                    points.append(center_freq - offset)
                offset += self.step_hz

        elif self.scan_strategy == "sweep_up":
            # 向上扫描
            freq = center_freq - half_range
            while freq <= center_freq + half_range and len(points) < self.max_scan_points:
                if 1000.0 <= freq <= 100_000.0:
                    points.append(freq)
                freq += self.step_hz

        else:  # "fine_grid"
            # 密集网格
            freq = center_freq - half_range
            while freq <= center_freq + half_range and len(points) < self.max_scan_points:
                if 1000.0 <= freq <= 100_000.0:
                    points.append(freq)
                freq += self.step_hz

        # 确保所有点都对齐到100Hz网格
        points = [self._align_to_grid(f) for f in points]

        # 去重并排序（如果需要）
        points = sorted(set(points))

        LOGGER.info(
            f"Planned {len(points)} scan points around {center_freq:.1f} Hz "
            f"(range: ±{half_range:.1f} Hz, step: {self.step_hz:.1f} Hz)"
        )

        return points

    def _align_to_grid(self, frequency_hz: float) -> float:
        """将频率对齐到100Hz网格

        Args:
            frequency_hz: 输入频率

        Returns:
            对齐后的频率（100Hz的整数倍）
        """
        return round(frequency_hz / 100.0) * 100.0

    def frequency_to_tuning_word(self, frequency_hz: float) -> int:
        """将频率转换为DDS调谐字

        Args:
            frequency_hz: 目标频率

        Returns:
            32位DDS调谐字
        """
        if frequency_hz <= 0.0 or self.dds_clock_hz <= 0.0:
            return 0

        # DDS调谐字 = freq * 2^32 / clock
        tuning_word = int(round(frequency_hz * (2**32) / self.dds_clock_hz))

        # 限制在有效范围内
        tuning_word = max(1, min(0xFFFFFFFF, tuning_word))

        return tuning_word

    def tuning_word_to_frequency(self, tuning_word: int) -> float:
        """将DDS调谐字转换回频率

        Args:
            tuning_word: 32位DDS调谐字

        Returns:
            频率 (Hz)
        """
        if tuning_word <= 0 or self.dds_clock_hz <= 0.0:
            return 0.0

        # freq = tuning_word * clock / 2^32
        frequency_hz = float(tuning_word) * self.dds_clock_hz / (2**32)

        return frequency_hz

    def select_best_frequency(
        self,
        scan_results: list[tuple[float, int, bool]]
    ) -> ScanResult:
        """从扫描结果中选择最佳频率

        Args:
            scan_results: 扫描结果列表 [(frequency_hz, quality_score, is_circle), ...]

        Returns:
            ScanResult: 最佳扫描结果
        """
        if not scan_results:
            return ScanResult(
                0.0, 0, 0, 0, False,
                "no scan results available"
            )

        # 筛选出检测到圆形的结果
        circle_results = [
            (freq, quality, is_circle)
            for freq, quality, is_circle in scan_results
            if is_circle
        ]

        if circle_results:
            # 选择质量最高的
            best = max(circle_results, key=lambda x: x[1])
            best_freq, best_quality, _ = best

            tuning_word = self.frequency_to_tuning_word(best_freq)

            return ScanResult(
                frequency_hz=best_freq,
                quality_score=best_quality,
                tuning_word=tuning_word,
                scan_count=len(scan_results),
                found_circle=True,
                reason=f"best quality: {best_quality}"
            )
        else:
            # 没有找到圆形，返回质量最高的点
            best = max(scan_results, key=lambda x: x[1])
            best_freq, best_quality, _ = best

            tuning_word = self.frequency_to_tuning_word(best_freq)

            return ScanResult(
                frequency_hz=best_freq,
                quality_score=best_quality,
                tuning_word=tuning_word,
                scan_count=len(scan_results),
                found_circle=False,
                reason=f"no circle found, best quality: {best_quality}"
            )

    def should_extend_scan(
        self,
        current_results: list[tuple[float, int, bool]],
        scan_points_completed: int
    ) -> bool:
        """判断是否需要扩展扫描范围

        Args:
            current_results: 当前扫描结果
            scan_points_completed: 已完成的扫描点数

        Returns:
            是否需要扩展扫描
        """
        # 如果已经找到高质量圆形，不需要扩展
        for _, quality, is_circle in current_results:
            if is_circle and quality >= self.good_quality_threshold:
                return False

        # 如果已经扫描了足够多的点，不再扩展
        if scan_points_completed >= self.max_scan_points:
            return False

        # 如果没有找到任何圆形，可以考虑扩展
        has_circle = any(is_circle for _, _, is_circle in current_results)

        if not has_circle and scan_points_completed >= 10:
            return True

        return False

    def generate_extended_points(
        self,
        original_center_hz: float,
        existing_points: list[float]
    ) -> list[float]:
        """生成扩展扫描点

        Args:
            original_center_hz: 原始中心频率
            existing_points: 已扫描的频率点

        Returns:
            新的扫描点列表
        """
        existing_set = set(existing_points)
        new_points = []

        center = self._align_to_grid(original_center_hz)
        half_range = self.extended_range_hz / 2.0

        # 生成更大范围的点
        freq = center - half_range
        while freq <= center + half_range:
            freq_aligned = self._align_to_grid(freq)
            if (1000.0 <= freq_aligned <= 100_000.0 and
                freq_aligned not in existing_set):
                new_points.append(freq_aligned)
            freq += self.step_hz

        LOGGER.info(
            f"Extended scan: generated {len(new_points)} additional points "
            f"(range: ±{half_range:.1f} Hz)"
        )

        return new_points



# ========== OpenCV处理模块 ==========

# -*- coding: utf-8 -*-
"""单文件 OpenCV 示波器侧边拐点提取（固定机位版）。

摄像头、示波器和焦距固定后，程序直接使用预先标定的屏幕四角和电压标尺，
每帧只在左右窄带内提取高亮拐点，不再搜索屏幕边框或上下参考亮线。
按 q 或 ESC 退出，按 s 保存当前帧结果。
"""


from dataclasses import dataclass, replace



# ============================== 可调参数 ==============================

# 有效斜坡宽度（微秒），对应 FPGA 输出的 τ。运行时可用 --ramp-us
# 在 100 / 500 / 2000 / 5000 μs 四档中选择。
EFFECTIVE_RAMP_DURATION_US = 500.0
RAMP_DURATION_CHOICES_US = (100.0, 500.0, 2000.0, 5000.0)

# 拐点不需要很高的像素密度，640x512 可显著降低树莓派计算量。
DEFAULT_SCREEN_SIZE = (640, 512)

# 最多保留的侧边拐点数量。0.1 ms 档配合 100 kHz 输入时理论上会出现
# 20 个拐点，因此保留 24 个余量；仍只处理左右窄带，不会拟合整条波形。
DEFAULT_MAX_POINTS = 32

# 树莓派实拍图只需在每侧约 12% 的窄带中寻找正弦极值。搜索带过宽会把
# 人物胸口、衣服边缘和中央网格纳入候选，既增加误检也浪费计算时间。
SIDE_SEARCH_FRACTION = 0.12
SIDE_SCORE_PERCENTILE = 94.0
SIDE_GREEN_PERCENTILE = 65.0
HORIZONTAL_ROI_FRACTION = 0.85

# 每帧会先自动估计 X 正弦波当前的左右极值线，再保留其附近的候选。这里的
# 4% 是相对“本帧检测宽度”的容差，不再依赖固定的示波器水平位置或幅度。
TURNING_POINT_EDGE_TOLERANCE_FRACTION = 0.04

# 下面参数只影响人眼观察的结果窗口，不参与拐点识别和频率计算。
# 模拟示波器与摄像头不同步时，单帧会出现密集横向扫描带；低权重多帧平均
# 可以在不拖慢识别算法的前提下，让静止波形逐帧变清楚。
DISPLAY_HEADER_HEIGHT = 82
DISPLAY_TEMPORAL_ALPHA = 0.16

# 仅供文件中保留的离线自动标定工具函数使用；实时入口不会调用自动标定。
REFERENCE_SEARCH_FRACTION = 0.36

# -------------------------- 固定机位标定 --------------------------
# 树莓派实拍标定图：Camera_screenshot_31.07.2026.png（640 x 480）。
# 上边框位于画面外，下面四角由左右内边框、下边框及 10 x 8 方格比例拟合。
# 角点允许为负数，透视变换会把没有拍到的顶部保留为黑色区域。
# 若比赛现场移动了摄像头，只需重新填写下面四个角点，不要重新启用逐帧搜索。
FIXED_CALIBRATION_FRAME_SIZE = (640, 480)
FIXED_SCREEN_CORNERS = (
    (17.0, -27.0),
    (556.0, -15.0),
    (549.0, 418.0),
    (4.0, 407.0),
)

# 以下参数均对应矫正后的 640 x 512 屏幕。
# 上下二次曲线不是运行时检测结果，而是固定机位的一次性标定数据；它们只用于
# 把拐点 Y 像素换算为锯齿扫描时间，不要求 FPGA 再输出两条参考亮线。
# 左右值只是动态检测失败前的几何初值；正常处理时会被当前帧结果替换。
FIXED_REFERENCE_LEFT_X = 84.0
FIXED_REFERENCE_RIGHT_X = 552.0
FIXED_REFERENCE_CENTER_X = 318.0
FIXED_REFERENCE_SCALE_X = 234.0

# Measured calibration from TI_code_main.py. Keep these curves fixed: they
# define the proven point-search region and the pixel/time conversion.
FIXED_TOP_CURVE = (-1.4984422, 1.6279430, 104.05)
FIXED_BOTTOM_CURVE = (0.5968569, -0.3133652, 469.05)

# 有效锯齿仍为 -2 V 到 +2 V。空闲段改为 +/-3 V 后位于有效标尺之外，
# 因此这里只留少量边缘余量，直接在整个有效扫描高度内寻找拐点。
FIXED_TRACE_EDGE_MARGIN = 5

# Ignore the upper/lower 7.5% of the active ramp while retaining the full
# calibrated height for time normalization.
POINT_SEARCH_CENTER_FRACTION = 0.85

# 640 x 512 矫正图中，-2 V 到 +2 V 有效锯齿的完整高度。
# 频率单独使用这个标尺和同侧完整周期，不受左右纵向剪切影响。
FREQUENCY_RAMP_HEIGHT_PX = 469.05

# 标准周期占多数；漏检一个同侧点会产生接近 2 倍的长间距。
STANDARD_PERIOD_TOLERANCE = 0.20
LONG_PERIOD_RATIO_MIN = 1.70

# 对最近 9 帧的完整周期取中位数，抑制摄像头 1～3 px 的单帧定位抖动。
TEMPORAL_PERIOD_WINDOW = 9


@dataclass(frozen=True)
class ReferenceLines:
    """上下参考曲线及其有效水平范围。

    CRT 屏幕存在几何失真，摄像头透视矫正后参考线仍可能倾斜或弯曲。
    top_curve / bottom_curve 保存归一化 X 坐标下的二次曲线系数
    ``a*x*x + b*x + c``；top_y / bottom_y 仅表示屏幕中央的高度，
    供状态显示和基本几何检查使用。
    """

    top_y: float
    bottom_y: float
    top_curve: tuple[float, float, float]
    bottom_curve: tuple[float, float, float]
    curve_center_x: float
    curve_scale_x: float
    top_band: tuple[int, int]
    bottom_band: tuple[int, int]
    left_x: float
    right_x: float
    confidence: float

    def _curve_y(self, coefficients: tuple[float, float, float], x_px: float) -> float:
        """计算指定 X 位置的参考线中心 Y 坐标。"""

        normalized_x = (float(x_px) - self.curve_center_x) / max(
            self.curve_scale_x, 1.0)
        a, b, c = coefficients
        return float((a * normalized_x + b) * normalized_x + c)

    def top_y_at(self, x_px: float) -> float:
        """返回指定 X 位置的上参考线中心。"""

        return self._curve_y(self.top_curve, x_px)

    def bottom_y_at(self, x_px: float) -> float:
        """返回指定 X 位置的下参考线中心。"""

        return self._curve_y(self.bottom_curve, x_px)


@dataclass(frozen=True)
class WavePoint:
    """一个最终输出的波形采样点。"""

    x_px: float
    y_px: float
    x_normalized: float
    y_normalized: float
    y_volts: float
    time_normalized: float
    strength: float


@dataclass
class ProcessResult:
    """单帧处理结果。"""

    corners: np.ndarray
    rectified: np.ndarray
    trace_mask: np.ndarray
    overlay: np.ndarray
    points: list[WavePoint]
    references: ReferenceLines
    avg_phase_interval: float    # 稳健估计的完整周期归一化间隔
    phase_interval_std: float    # 标准差
    valid_interval_count: int    # 有效间隔数
    frequency_hz: float          # 估计的频率（Hz）


class TemporalPeriodFilter:
    """用短窗口中位数稳定连续帧的完整周期。"""

    def __init__(self, window_size: int = TEMPORAL_PERIOD_WINDOW) -> None:
        self._periods: deque[float] = deque(maxlen=max(1, int(window_size)))

    def update(
        self,
        period_normalized: float,
        valid_count: int,
        ramp_duration_us: float,
    ) -> tuple[float, float]:
        if (
            valid_count < 2
            or period_normalized <= 0.0
            or ramp_duration_us <= 0.0
        ):
            return period_normalized, 0.0

        self._periods.append(float(period_normalized))
        stable_period = float(np.median(np.asarray(self._periods, np.float64)))
        period_sec = stable_period * ramp_duration_us / 1_000_000.0
        stable_frequency_hz = 1.0 / period_sec if period_sec > 0.0 else 0.0
        return stable_period, stable_frequency_hz


def write_image(path: Path, image: np.ndarray) -> None:
    """兼容 Windows 中文路径保存图片。"""

    path.parent.mkdir(parents=True, exist_ok=True)
    suffix = path.suffix if path.suffix else ".png"
    ok, encoded = cv2.imencode(suffix, image)
    if not ok:
        raise RuntimeError(f"无法编码图片：{path}")
    encoded.tofile(path)


def order_corners(points: np.ndarray) -> np.ndarray:
    """把四个角点统一整理为：左上、右上、右下、左下。"""

    points = np.asarray(points, dtype=np.float32).reshape(4, 2)
    ordered = np.empty((4, 2), dtype=np.float32)
    sums = points.sum(axis=1)
    differences = np.diff(points, axis=1).reshape(-1)
    ordered[0] = points[np.argmin(sums)]
    ordered[2] = points[np.argmax(sums)]
    ordered[1] = points[np.argmin(differences)]
    ordered[3] = points[np.argmax(differences)]
    return ordered


def get_fixed_screen_corners(frame: np.ndarray) -> np.ndarray:
    """按当前帧尺寸缩放固定机位的四角标定值。

    标定基准为 780 x 564。摄像头驱动若只做等比例像素缩放，程序仍可适配
    其他输出分辨率；若改变了裁切范围、焦距或相机位置，则必须重新标定四角。
    """

    frame_height, frame_width = frame.shape[:2]
    calibration_width, calibration_height = FIXED_CALIBRATION_FRAME_SIZE
    if frame_width < 2 or frame_height < 2:
        raise ValueError("摄像头帧尺寸无效")

    scale_x = (frame_width - 1) / max(1, calibration_width - 1)
    scale_y = (frame_height - 1) / max(1, calibration_height - 1)
    corners = np.asarray(FIXED_SCREEN_CORNERS, np.float32).copy()
    corners[:, 0] *= scale_x
    corners[:, 1] *= scale_y
    return order_corners(corners)


def get_fixed_reference_calibration(
    screen_size: tuple[int, int],
) -> ReferenceLines:
    """返回固定的像素/电压标尺，不读取当前帧中的上下亮线。

    二次曲线补偿模拟示波器的轻微几何失真。所有系数随矫正图尺寸缩放，
    默认 640 x 512 时即为文件顶部记录的一次性标定值。
    """

    width, height = screen_size
    base_width, base_height = DEFAULT_SCREEN_SIZE
    scale_x = (width - 1) / max(1, base_width - 1)
    scale_y = (height - 1) / max(1, base_height - 1)

    top_curve = tuple(value * scale_y for value in FIXED_TOP_CURVE)
    bottom_curve = tuple(value * scale_y for value in FIXED_BOTTOM_CURVE)
    left_x = FIXED_REFERENCE_LEFT_X * scale_x
    right_x = FIXED_REFERENCE_RIGHT_X * scale_x
    center_x = FIXED_REFERENCE_CENTER_X * scale_x
    curve_scale_x = FIXED_REFERENCE_SCALE_X * scale_x

    # band 字段只保留数据结构兼容性；直接提点流程不会根据图像搜索它们。
    top_values = [
        (top_curve[0] * normalized_x + top_curve[1]) * normalized_x + top_curve[2]
        for normalized_x in (-1.0, 0.0, 1.0)
    ]
    bottom_values = [
        (bottom_curve[0] * normalized_x + bottom_curve[1]) * normalized_x
        + bottom_curve[2]
        for normalized_x in (-1.0, 0.0, 1.0)
    ]
    top_band = (
        max(0, int(math.floor(min(top_values)))),
        min(height - 1, int(math.ceil(max(top_values)))),
    )
    bottom_band = (
        max(0, int(math.floor(min(bottom_values)))),
        min(height - 1, int(math.ceil(max(bottom_values)))),
    )

    return ReferenceLines(
        top_y=float(top_curve[2]),
        bottom_y=float(bottom_curve[2]),
        top_curve=top_curve,
        bottom_curve=bottom_curve,
        curve_center_x=center_x,
        curve_scale_x=curve_scale_x,
        top_band=top_band,
        bottom_band=bottom_band,
        left_x=left_x,
        right_x=right_x,
        confidence=1.0,
    )


def line_equation(segment: tuple[int, int, int, int]) -> np.ndarray:
    """把线段转换为 ax + by + c = 0。"""

    x1, y1, x2, y2 = [float(value) for value in segment]
    return np.asarray([y1 - y2, x2 - x1, x1 * y2 - x2 * y1], np.float64)


def line_intersection(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    """计算两条直线的交点。"""

    cross = np.cross(first, second)
    if abs(float(cross[2])) < 1e-8:
        raise ValueError("屏幕边界直线近似平行，无法求交点")
    return (cross[:2] / cross[2]).astype(np.float32)


def detect_dark_screen_corners(gray: np.ndarray) -> np.ndarray:
    """利用“暗色屏幕区域”的凸包优先定位内屏四角。

    老式模拟示波器的内屏通常明显暗于浅色机壳。先找暗区再取凸包，
    可以避免把左侧机壳斜边误认为屏幕边界。若现场光照不满足这一特征，
    调用者仍会继续使用后面的霍夫直线方法作为兜底。
    """

    height, width = gray.shape
    frame_area = float(height * width)

    # 使用亮度分位数而不是固定阈值，兼容不同曝光和摄像头。
    dark_limit = int(np.clip(np.percentile(gray, 18.0), 35, 115))
    dark_mask = cv2.threshold(
        gray, dark_limit, 255, cv2.THRESH_BINARY_INV)[1]

    # 补齐屏幕内部被亮参考线、波形和网格切断的小空洞。
    kernel_size = max(9, int(round(min(height, width) * 0.035)) | 1)
    dark_mask = cv2.morphologyEx(
        dark_mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(
            cv2.MORPH_RECT, (kernel_size, kernel_size)),
    )

    contours = cv2.findContours(
        dark_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    candidates: list[tuple[float, np.ndarray]] = []
    expected_ratio = DEFAULT_SCREEN_SIZE[0] / DEFAULT_SCREEN_SIZE[1]

    for contour in contours:
        x, y, box_width, box_height = cv2.boundingRect(contour)
        if not (width * 0.50 <= box_width <= width * 0.90):
            continue
        if not (height * 0.52 <= box_height <= height * 0.93):
            continue
        if x < width * 0.05:
            # 贴住画面左边的暗区通常是背景或机壳外部，不是内屏。
            continue
        if not (x <= width * 0.5 <= x + box_width):
            continue
        if not (y <= height * 0.5 <= y + box_height):
            continue

        hull = cv2.convexHull(contour)
        perimeter = cv2.arcLength(hull, True)
        if perimeter <= 0.0:
            continue

        quadrilateral: np.ndarray | None = None
        for epsilon_ratio in (0.012, 0.018, 0.025, 0.035, 0.050):
            approximation = cv2.approxPolyDP(
                hull, epsilon_ratio * perimeter, True)
            if len(approximation) == 4 and cv2.isContourConvex(approximation):
                quadrilateral = order_corners(approximation.reshape(4, 2))
                break
        if quadrilateral is None:
            continue

        area = abs(float(cv2.contourArea(quadrilateral)))
        area_ratio = area / frame_area
        if not 0.30 <= area_ratio <= 0.82:
            continue

        top_width = float(np.linalg.norm(quadrilateral[1] - quadrilateral[0]))
        bottom_width = float(np.linalg.norm(quadrilateral[2] - quadrilateral[3]))
        left_height = float(np.linalg.norm(quadrilateral[3] - quadrilateral[0]))
        right_height = float(np.linalg.norm(quadrilateral[2] - quadrilateral[1]))
        mean_width = 0.5 * (top_width + bottom_width)
        mean_height = 0.5 * (left_height + right_height)
        if mean_width < width * 0.48 or mean_height < height * 0.50:
            continue

        aspect_ratio = mean_width / max(mean_height, 1.0)
        if not 0.85 <= aspect_ratio <= 1.85:
            continue

        center = np.mean(quadrilateral, axis=0)
        center_error = (
            abs(float(center[0]) - width * 0.5) / width +
            abs(float(center[1]) - height * 0.5) / height
        )
        ratio_error = abs(math.log(max(aspect_ratio, 1e-6) / expected_ratio))
        score = area_ratio - 0.18 * center_error - 0.12 * ratio_error
        candidates.append((score, quadrilateral))

    if not candidates:
        raise ValueError("暗区法没有找到可信的内屏四边形")
    return max(candidates, key=lambda item: item[0])[1]


def _detect_screen_corners_once(
    frame: np.ndarray,
    prefer_dark_outline: bool,
) -> np.ndarray:
    """利用屏幕四周长边自动检测屏幕四角。

    这里只把霍夫变换用于寻找屏幕外框，不用它检测上下参考线。
    参考线采用后面的亮度行投影算法，因此不会因为线条太粗而漏检。
    """

    height, width = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (7, 7), 0)

    if prefer_dark_outline:
        # 暗区法速度更快，作为霍夫边界无法闭合时的兜底。
        try:
            return detect_dark_screen_corners(gray)
        except ValueError:
            pass

    # 根据图像中位亮度自动设置 Canny 阈值，适应不同曝光。
    median = float(np.median(gray))
    lower = int(max(15, 0.45 * median))
    upper = int(min(220, max(lower + 25, 1.35 * median)))
    edges = cv2.Canny(gray, lower, upper)

    lines = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 360.0,
        threshold=max(45, min(height, width) // 8),
        minLineLength=max(120, int(min(height, width) * 0.27)),
        maxLineGap=max(20, int(min(height, width) * 0.08)),
    )
    if lines is None:
        raise ValueError("没有检测到足够长的屏幕边界")

    horizontal: list[tuple[float, float, tuple[int, int, int, int]]] = []
    vertical: list[tuple[float, float, tuple[int, int, int, int]]] = []
    for raw in lines[:, 0]:
        x1, y1, x2, y2 = [int(value) for value in raw]
        dx = x2 - x1
        dy = y2 - y1
        length = math.hypot(dx, dy)
        angle = abs(math.degrees(math.atan2(dy, dx)))
        angle = min(angle, abs(180.0 - angle))
        midpoint_x = 0.5 * (x1 + x2)
        midpoint_y = 0.5 * (y1 + y2)
        segment = (x1, y1, x2, y2)
        if angle <= 14.0:
            horizontal.append((length, midpoint_y, segment))
        elif angle >= 72.0:
            vertical.append((length, midpoint_x, segment))

    if len(horizontal) < 2 or len(vertical) < 2:
        raise ValueError("没有找到完整的屏幕横边和竖边")

    # 左边界优先选择靠近画面内部且较长的竖线，避开机壳外轮廓。
    left_candidates = [
        item for item in vertical
        if width * 0.07 <= item[1] <= width * 0.42
    ]
    right_candidates = [
        item for item in vertical
        if width * 0.58 <= item[1] <= width * 0.98
    ]
    if not left_candidates or not right_candidates:
        raise ValueError("无法确定屏幕左右边界")

    def left_edge_polarity(segment: tuple[int, int, int, int]) -> float:
        """判断左边界是否满足“左侧亮机壳、右侧暗屏幕”。"""

        x1, y1, x2, y2 = [float(value) for value in segment]
        sample_count = 56
        offset = max(6, int(round(width * 0.012)))
        radius = 2
        left_values: list[float] = []
        right_values: list[float] = []
        for factor in np.linspace(0.08, 0.92, sample_count):
            x = int(round(x1 + factor * (x2 - x1)))
            y = int(round(y1 + factor * (y2 - y1)))
            if not (radius <= y < height - radius):
                continue
            left_x = x - offset
            right_x = x + offset
            if not (radius <= left_x < width - radius):
                continue
            if not (radius <= right_x < width - radius):
                continue
            left_patch = gray[
                y - radius:y + radius + 1,
                left_x - radius:left_x + radius + 1,
            ]
            right_patch = gray[
                y - radius:y + radius + 1,
                right_x - radius:right_x + radius + 1,
            ]
            left_values.append(float(np.median(left_patch)))
            right_values.append(float(np.median(right_patch)))
        if not left_values:
            return -255.0
        return float(np.median(left_values) - np.median(right_values))

    # 外壳斜边常比内屏边界更长，不能只按线长选择。
    inner_left_candidates = [
        item for item in left_candidates
        if left_edge_polarity(item[2]) > 10.0
    ]
    left_pool = inner_left_candidates or left_candidates
    left = max(left_pool, key=lambda item: item[0] + 0.18 * item[1])
    right = max(right_candidates, key=lambda item: item[0] - 0.05 * item[1])
    left_x = left[1]
    right_x = right[1]
    if right_x - left_x < width * 0.45:
        raise ValueError("检测到的屏幕宽度过小")

    # 水平边必须大部分位于左右屏幕边界之间。
    def overlaps_screen(segment: tuple[int, int, int, int]) -> bool:
        x1, _, x2, _ = segment
        segment_left = min(x1, x2)
        segment_right = max(x1, x2)
        overlap = min(segment_right, right_x) - max(segment_left, left_x)
        return overlap >= width * 0.22

    top_candidates = [
        item for item in horizontal
        if height * 0.035 <= item[1] <= height * 0.34
        and overlaps_screen(item[2])
    ]
    bottom_candidates = [
        item for item in horizontal
        if height * 0.62 <= item[1] <= height * 0.93
        and overlaps_screen(item[2])
    ]
    if not top_candidates or not bottom_candidates:
        raise ValueError("无法确定屏幕上下边界")

    # 顶边取最靠上的长线，底边取 93% 高度以内最靠下的长线。
    top = min(top_candidates, key=lambda item: item[1] - 0.001 * item[0])
    bottom = max(bottom_candidates, key=lambda item: item[1] + 0.001 * item[0])

    left_line = line_equation(left[2])
    right_line = line_equation(right[2])
    top_line = line_equation(top[2])
    bottom_line = line_equation(bottom[2])
    corners = order_corners(np.asarray([
        line_intersection(left_line, top_line),
        line_intersection(right_line, top_line),
        line_intersection(right_line, bottom_line),
        line_intersection(left_line, bottom_line),
    ]))

    # 基本几何检查，防止把机壳边缘误当成屏幕。
    area = abs(float(cv2.contourArea(corners)))
    frame_area = float(height * width)
    if not frame_area * 0.25 <= area <= frame_area * 0.88:
        raise ValueError(f"屏幕四边形面积异常：{area:.0f}")
    return corners


def detect_screen_corners(frame: np.ndarray) -> np.ndarray:
    """优先按真实外框直线定位内屏，失败时再使用暗区轮廓。

    粗亮参考线紧贴屏幕边缘时，暗区轮廓可能在亮线处提前结束并裁掉
    一部分参考线；霍夫外框不会受到这个问题影响。
    """

    try:
        return _detect_screen_corners_once(frame, prefer_dark_outline=False)
    except ValueError:
        return _detect_screen_corners_once(frame, prefer_dark_outline=True)


def rectify_screen(
    frame: np.ndarray,
    corners: np.ndarray,
    size: tuple[int, int],
) -> np.ndarray:
    """把倾斜屏幕矫正为固定大小的正视图。"""

    width, height = size
    destination = np.asarray([
        [0, 0],
        [width - 1, 0],
        [width - 1, height - 1],
        [0, height - 1],
    ], dtype=np.float32)
    transform = cv2.getPerspectiveTransform(order_corners(corners), destination)
    return cv2.warpPerspective(frame, transform, (width, height))


def smooth_profile(values: np.ndarray, size: int) -> np.ndarray:
    """对一维曲线做高斯平滑。"""

    size = max(3, int(size) | 1)
    return cv2.GaussianBlur(
        values.astype(np.float32).reshape(-1, 1),
        (1, size),
        0,
    ).reshape(-1)


def top_fraction_mean(values: np.ndarray, fraction: float) -> np.ndarray:
    """每一行只统计最亮的一部分像素，减小暗网格的影响。"""

    count = max(1, int(round(values.shape[1] * fraction)))
    selected = np.partition(values, -count, axis=1)[:, -count:]
    return np.mean(selected, axis=1)


def detect_band(
    activity: np.ndarray,
    start: int,
    stop: int,
) -> tuple[int, int, float, float]:
    """在指定纵向范围内寻找一条粗亮线及其上下边界。"""

    region = activity[start:stop]
    if region.size < 5:
        raise ValueError("参考线搜索范围太小")
    peak = start + int(np.argmax(region))
    baseline = float(np.percentile(region, 35.0))
    peak_value = float(activity[peak])
    contrast = peak_value - baseline
    if contrast < 7.0:
        raise ValueError("参考线与背景的亮度差太小")

    threshold = baseline + 0.34 * contrast
    lower = peak
    upper = peak
    while lower > start and activity[lower - 1] >= threshold:
        lower -= 1
    while upper + 1 < stop and activity[upper + 1] >= threshold:
        upper += 1

    rows = np.arange(lower, upper + 1, dtype=np.float32)
    weights = np.maximum(activity[lower:upper + 1] - baseline, 0.0)
    center = (
        float(np.sum(rows * weights) / np.sum(weights))
        if float(np.sum(weights)) > 0.0
        else float(peak)
    )
    return lower, upper, center, contrast


def longest_run(binary: np.ndarray) -> tuple[int, int] | None:
    """返回一维布尔数组中最长的连续真值区间。"""

    padded = np.pad(binary.astype(np.int8), (1, 1))
    changes = np.diff(padded)
    starts = np.flatnonzero(changes == 1)
    stops = np.flatnonzero(changes == -1)
    if starts.size == 0:
        return None
    lengths = stops - starts
    index = int(np.argmax(lengths))
    return int(starts[index]), int(stops[index])


def fit_reference_curve(
    score: np.ndarray,
    rough_band: tuple[int, int],
    rough_center_y: float,
    left_x: float,
    right_x: float,
    curve_center_x: float,
    curve_scale_x: float,
) -> tuple[tuple[float, float, float], tuple[int, int], float, float]:
    """逐列定位粗亮线中心，并做带离群点剔除的二次曲线拟合。

    返回值依次为：曲线系数、覆盖整条曲线的纵向包络、拟合残差、
    有效列覆盖率。逐列搜索使用局部连续亮带，而不是单个最亮像素，
    因此参考线较粗、局部过曝或穿过网格时仍能得到稳定中心。
    """

    height, width = score.shape
    rough_low, rough_high = rough_band
    x_start = max(1, int(math.floor(left_x)))
    x_stop = min(width - 2, int(math.ceil(right_x)))
    if x_stop - x_start < width * 0.25:
        raise ValueError("参考线曲线拟合的水平范围过小")

    rough_thickness = max(1, rough_high - rough_low + 1)
    vertical_padding = max(
        12,
        int(round(height * 0.055)),
        int(round(rough_thickness * 0.9)),
    )
    search_start = max(1, rough_low - vertical_padding)
    search_stop = min(height - 2, rough_high + vertical_padding)
    if search_stop - search_start < 12:
        raise ValueError("参考线曲线拟合的纵向范围过小")

    # 小范围二维平滑只用于参考线定位；原图和后续拐点检测不受影响。
    roi = score[search_start:search_stop + 1, x_start:x_stop + 1]
    smoothed = cv2.GaussianBlur(roi.astype(np.float32), (5, 3), 0)
    candidates: list[tuple[float, float, float, float, float, float, float]] = []

    for local_x in range(smoothed.shape[1]):
        column = smoothed[:, local_x]
        baseline = float(np.percentile(column, 28.0))
        peak_index = int(np.argmax(column))
        peak_value = float(column[peak_index])
        contrast = peak_value - baseline
        if contrast < 5.0:
            continue

        # 取包含峰值的连续亮带，避免同列的网格或波形参与中心计算。
        threshold = baseline + 0.38 * contrast
        low = peak_index
        high = peak_index
        while low > 0 and float(column[low - 1]) >= threshold:
            low -= 1
        while high + 1 < column.size and float(column[high + 1]) >= threshold:
            high += 1

        thickness = high - low + 1
        clipped_low = low == 0
        clipped_high = high == column.size - 1
        if clipped_low and clipped_high:
            continue
        if thickness < 2 or thickness > max(10, int(round(height * 0.11))):
            continue

        rows = np.arange(low, high + 1, dtype=np.float32)
        weights = np.maximum(column[low:high + 1] - baseline, 0.0)
        weight_sum = float(np.sum(weights))
        if weight_sum <= 0.0:
            continue
        center_y = search_start + float(np.sum(rows * weights) / weight_sum)
        candidates.append((
            float(x_start + local_x),
            center_y,
            float(search_start + low),
            float(search_start + high),
            contrast,
            float(thickness),
            float(clipped_low),
            float(clipped_high),
        ))

    minimum_columns = max(24, int(round((x_stop - x_start + 1) * 0.32)))
    if len(candidates) < minimum_columns:
        raise ValueError("参考线有效采样列不足")

    samples = np.asarray(candidates, dtype=np.float64)
    # 两端没有参考线时，普通波形也可能形成局部峰；参考线覆盖大多数列且更亮，
    # 先按整体峰值和对比度中位数剔除这些弱候选。
    contrast_limit = max(6.0, float(np.median(samples[:, 4])) * 0.42)
    strong = samples[:, 4] >= contrast_limit
    if int(np.count_nonzero(strong)) >= minimum_columns:
        samples = samples[strong]

    normalized_x = (samples[:, 0] - curve_center_x) / max(curve_scale_x, 1.0)
    clipped_low_fraction = float(np.mean(samples[:, 6]))
    clipped_high_fraction = float(np.mean(samples[:, 7]))
    if clipped_high_fraction >= 0.35:
        # 下参考线常紧贴内屏下边缘。此时拟合朝屏幕内部的上边缘，
        # 最后再用全局行投影中心恢复其真实纵向位置。
        y_values = samples[:, 2]
    elif clipped_low_fraction >= 0.35:
        y_values = samples[:, 3]
    else:
        y_values = samples[:, 1]
    contrast_values = samples[:, 4]
    weights = np.sqrt(np.clip(
        contrast_values / max(float(np.median(contrast_values)), 1.0),
        0.35,
        3.0,
    ))
    keep = np.ones(samples.shape[0], dtype=bool)
    coefficients = np.asarray([0.0, 0.0, float(np.median(y_values))])

    # 反复拟合并按 MAD 剔除离群列，可抵抗网格交点、反光和波形交叉。
    for _ in range(6):
        if int(np.count_nonzero(keep)) < minimum_columns:
            break
        coefficients = np.polyfit(
            normalized_x[keep], y_values[keep], 2, w=weights[keep])
        residuals = y_values - np.polyval(coefficients, normalized_x)
        residual_center = float(np.median(residuals[keep]))
        mad = float(np.median(np.abs(residuals[keep] - residual_center)))
        residual_limit = max(1.5, 3.5 * 1.4826 * mad)
        new_keep = np.abs(residuals - residual_center) <= residual_limit
        if int(np.count_nonzero(new_keep)) < minimum_columns:
            break
        if np.array_equal(new_keep, keep):
            keep = new_keep
            break
        keep = new_keep

    if int(np.count_nonzero(keep)) < minimum_columns:
        raise ValueError("参考线曲线拟合后的有效采样列不足")

    # 极端二次项通常来自局部反光，而不是 CRT 的正常几何弯曲；此时退回直线。
    if abs(float(coefficients[0])) > height * 0.10:
        linear = np.polyfit(
            normalized_x[keep], y_values[keep], 1, w=weights[keep])
        coefficients = np.asarray([0.0, float(linear[0]), float(linear[1])])

    # 边缘拟合只负责斜率和曲率；纵向绝对位置仍以全宽行投影为准。
    # 这一步也可消除粗亮线局部过曝造成的固定中心偏差。
    sample_curve_y = np.polyval(coefficients, normalized_x[keep])
    coefficients[2] += float(rough_center_y) - float(np.median(sample_curve_y))

    fitted_y = np.polyval(coefficients, normalized_x[keep])
    vertical_offset = float(rough_center_y) - float(np.median(sample_curve_y))
    absolute_residuals = np.abs((y_values[keep] + vertical_offset) - fitted_y)
    fit_residual = float(np.median(absolute_residuals))
    typical_thickness = float(np.percentile(samples[keep, 5], 75.0))

    curve_x = np.linspace(left_x, right_x, max(32, x_stop - x_start + 1))
    curve_normalized_x = (
        (curve_x - curve_center_x) / max(curve_scale_x, 1.0)
    )
    curve_y = np.polyval(coefficients, curve_normalized_x)
    envelope_margin = max(
        3.0,
        0.55 * typical_thickness + 2.0,
        3.0 * fit_residual + 1.0,
    )
    band = (
        max(0, int(math.floor(float(np.min(curve_y)) - envelope_margin))),
        min(height - 1, int(math.ceil(float(np.max(curve_y)) + envelope_margin))),
    )
    coverage = float(np.count_nonzero(keep)) / max(1.0, right_x - left_x + 1.0)
    return (
        tuple(float(value) for value in coefficients),
        band,
        fit_residual,
        coverage,
    )


def detect_reference_lines(screen: np.ndarray) -> ReferenceLines:
    """检测上、下参考线。

    先用行亮度投影找到上下粗带，再逐列提取中心并拟合二次曲线。
    这样既不怕参考线过粗，也能补偿模拟示波器的倾斜和桶形失真。
    """

    blue, green, red = cv2.split(screen.astype(np.int16))
    brightness = np.max(screen, axis=2).astype(np.float32)
    green_excess = np.clip(green - np.maximum(blue, red), 0, 255).astype(np.float32)

    # 白绿色参考线可能不是纯绿色，因此同时使用亮度和绿色占优量。
    score = 0.72 * brightness + 0.85 * green_excess
    height, width = score.shape
    x_margin = max(4, int(round(width * 0.04)))
    row_activity = top_fraction_mean(score[:, x_margin:width - x_margin], 0.28)
    row_activity = smooth_profile(row_activity, max(5, height // 70))

    search = min(0.45, max(0.25, REFERENCE_SEARCH_FRACTION))
    top_start = max(1, int(height * 0.02))
    top_stop = max(top_start + 5, int(height * search))
    bottom_start = min(height - 6, int(height * (1.0 - search)))
    bottom_stop = min(height - 1, int(height * 0.98))

    top_low, top_high, top_y, top_contrast = detect_band(
        row_activity, top_start, top_stop)
    bottom_low, bottom_high, bottom_y, bottom_contrast = detect_band(
        row_activity, bottom_start, bottom_stop)

    if bottom_y - top_y < height * 0.48:
        raise ValueError("上下参考线距离过近")

    def horizontal_span(low: int, high: int) -> tuple[int, int]:
        band = score[low:high + 1]
        # 只使用该带内较亮像素，并对列覆盖率做闭运算补齐小缺口。
        threshold = max(
            float(np.percentile(band, 70.0)),
            float(np.percentile(score, 91.0)),
        )
        coverage = np.mean(band >= threshold, axis=0)
        active = (coverage >= 0.10).astype(np.uint8) * 255
        close_width = max(9, width // 35)
        active = cv2.morphologyEx(
            active.reshape(1, -1),
            cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_RECT, (close_width, 1)),
        ).reshape(-1) > 0
        active[:x_margin] = False
        active[width - x_margin:] = False
        run = longest_run(active)
        if run is None or run[1] - run[0] < width * 0.30:
            raise ValueError("参考线横向长度不足")
        return run

    top_left, top_right = horizontal_span(top_low, top_high)
    bottom_left, bottom_right = horizontal_span(bottom_low, bottom_high)
    # 两条水平线都由同一个 X 信号扫出，本应具有相同的左右范围。
    # CRT 几何失真会让倾斜线的一端落出检测带，因此取二者联合范围更可靠。
    left_x = float(min(top_left, bottom_left))
    right_x = float(max(top_right - 1, bottom_right - 1))
    horizontal_width = right_x - left_x
    if horizontal_width < width * 0.38:
        raise ValueError("上下参考线的水平范围过小")

    curve_center_x = 0.5 * (left_x + right_x)
    curve_scale_x = max(1.0, 0.5 * horizontal_width)
    top_curve, top_band, top_residual, top_coverage = fit_reference_curve(
        score,
        (top_low, top_high),
        top_y,
        left_x,
        right_x,
        curve_center_x,
        curve_scale_x,
    )
    bottom_curve, bottom_band, bottom_residual, bottom_coverage = fit_reference_curve(
        score,
        (bottom_low, bottom_high),
        bottom_y,
        left_x,
        right_x,
        curve_center_x,
        curve_scale_x,
    )
    top_y = float(top_curve[2])
    bottom_y = float(bottom_curve[2])
    if bottom_y - top_y < height * 0.48:
        raise ValueError("拟合后的上下参考线距离过近")

    contrast_score = min(1.0, min(top_contrast, bottom_contrast) / 45.0)
    span_score = min(1.0, horizontal_width / (width * 0.65))
    separation_score = min(1.0, (bottom_y - top_y) / (height * 0.75))
    curve_score = min(
        1.0,
        min(top_coverage, bottom_coverage) / 0.72,
    ) * math.exp(-max(top_residual, bottom_residual) / 5.0)
    confidence = (
        0.38 * contrast_score +
        0.22 * span_score +
        0.16 * separation_score +
        0.24 * curve_score
    )

    return ReferenceLines(
        top_y=top_y,
        bottom_y=bottom_y,
        top_curve=top_curve,
        bottom_curve=bottom_curve,
        curve_center_x=curve_center_x,
        curve_scale_x=curve_scale_x,
        top_band=top_band,
        bottom_band=bottom_band,
        left_x=left_x,
        right_x=right_x,
        confidence=float(np.clip(confidence, 0.0, 1.0)),
    )


def build_side_trace_score(
    screen: np.ndarray,
    y_start: int,
    y_stop: int,
    side_bands: tuple[tuple[int, int], tuple[int, int]],
) -> tuple[np.ndarray, np.ndarray]:
    """只增强左右窄带中的绿色亮轨迹。

    旧算法会在整幅图上做大核形态学运算。这里改为对两个小区域使用
    9x9 均值背景差，中央波形和中央反光都不会参与计算。
    """

    height, width = screen.shape[:2]
    score = np.zeros((height, width), np.float32)
    green_excess = np.zeros((height, width), np.uint8)

    for x_start, x_stop in side_bands:
        crop = screen[y_start:y_stop, x_start:x_stop]
        blue, green, red = cv2.split(crop)
        green_i16 = green.astype(np.int16)
        local_excess = np.clip(
            green_i16 - np.maximum(blue, red).astype(np.int16), 0, 255
        ).astype(np.uint8)

        # 小核均值滤波估计局部背景，绿色细亮轨迹保留为正差值。
        background = cv2.boxFilter(
            green, cv2.CV_8U, (9, 9), normalize=True)
        detail = cv2.subtract(green, background).astype(np.float32)
        local_score = (
            0.72 * detail +
            0.45 * local_excess.astype(np.float32)
        )
        local_score = cv2.GaussianBlur(local_score, (3, 3), 0)
        score[y_start:y_stop, x_start:x_stop] = local_score
        green_excess[y_start:y_stop, x_start:x_stop] = local_excess

    return score, green_excess


def estimate_waveform_edges(
    screen: np.ndarray,
    y_start: int,
    y_stop: int,
) -> tuple[float, float, np.ndarray, np.ndarray]:
    """从当前帧自动估计正弦波的左右极值线。

    真正的正弦极值在多处 Y 位置形成短小的竖直回转段。先在左右半屏做绿色
    局部高通，再用竖向开运算累加这些重复回转段；人物反光虽然面积大，但不
    会在同一 X 坐标形成多次细窄回转，因此不会主导列峰值。

    返回 ``(left_x, right_x, score, green_excess)``。后两个数组会由正式提点
    流程复用，避免为了动态定位重复计算整幅图的局部高通。
    """

    height, width = screen.shape[:2]
    center_x = width // 2
    # The camera is fixed. Ignore the outer 7.5% on both sides and search only
    # the requested middle 85% of the rectified oscilloscope image.
    outer_margin = max(10, int(round(
        width * (1.0 - HORIZONTAL_ROI_FRACTION) * 0.5)))
    center_gap = max(12, int(round(width * 0.035)))
    broad_bands = (
        (outer_margin, center_x - center_gap),
        (center_x + center_gap, width - outer_margin),
    )
    if min(stop - start for start, stop in broad_bands) < 40:
        raise ValueError("动态极值搜索区域太窄")

    score, green_excess = build_side_trace_score(
        screen,
        y_start,
        y_stop,
        broad_bands,
    )
    broad_scores = np.concatenate([
        score[y_start:y_stop, start:stop].reshape(-1)
        for start, stop in broad_bands
    ])
    broad_green = np.concatenate([
        green_excess[y_start:y_stop, start:stop].reshape(-1)
        for start, stop in broad_bands
    ])
    score_threshold = max(
        3.0,
        float(np.percentile(broad_scores, SIDE_SCORE_PERCENTILE)),
    )
    green_threshold = max(
        4.0,
        float(np.percentile(broad_green, SIDE_GREEN_PERCENTILE)),
    )

    broad_mask = np.zeros((height, width), np.uint8)
    for start, stop in broad_bands:
        local_score = score[y_start:y_stop, start:stop]
        local_green = green_excess[y_start:y_stop, start:stop]
        broad_mask[y_start:y_stop, start:stop] = np.where(
            (local_score >= score_threshold)
            & (local_green >= green_threshold),
            255,
            0,
        ).astype(np.uint8)
    broad_mask = cv2.morphologyEx(
        broad_mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
    )

    vertical_size = max(7, int(round((y_stop - y_start) * 0.021)) | 1)
    vertical_turns = cv2.morphologyEx(
        broad_mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_RECT, (3, vertical_size)),
    )
    weighted_turns = np.where(vertical_turns > 0, score, 0.0)
    column_profile = np.sum(weighted_turns, axis=0).astype(np.float32)
    smooth_width = max(9, int(round(width * 0.033)) | 1)
    column_profile = cv2.GaussianBlur(
        column_profile.reshape(1, -1),
        (smooth_width, 1),
        0,
    ).reshape(-1)

    def strongest_column(start: int, stop: int) -> tuple[float, float]:
        region = column_profile[start:stop]
        if region.size == 0:
            return 0.0, 0.0
        local_index = int(np.argmax(region))
        return float(start + local_index), float(region[local_index])

    left_x, left_strength = strongest_column(*broad_bands[0])
    right_x, right_strength = strongest_column(*broad_bands[1])
    if min(left_strength, right_strength) <= 0.0:
        raise ValueError("当前帧无法定位正弦波左右极值")
    if right_x - left_x < width * 0.38:
        raise ValueError("当前帧检测到的正弦波水平幅度过小")

    return left_x, right_x, score, green_excess


def find_profile_peaks(
    profile: np.ndarray,
    start: int,
    stop: int,
    minimum_distance: int,
    maximum_count: int,
) -> list[tuple[int, float]]:
    """从纵向亮度曲线中寻找少量互相分离的峰值。"""

    region = profile[start:stop].astype(np.float32)
    if region.size < 5 or float(np.max(region)) <= 0.0:
        return []

    # 平滑只在一维数组上进行，计算量远低于逐行拟合完整轨迹。
    smooth_size = max(5, int(round(region.size * 0.018)) | 1)
    smoothed = smooth_profile(region, smooth_size)
    positive = smoothed[smoothed > 0.0]
    if positive.size < 3:
        return []

    low_level = float(np.percentile(positive, 35.0))
    high_level = float(np.percentile(positive, 92.0))
    threshold = low_level + 0.24 * max(0.0, high_level - low_level)

    # 先找三点局部极大值，再按强度做纵向非极大值抑制。
    local_maximum = np.zeros(region.size, dtype=bool)
    local_maximum[1:-1] = (
        (smoothed[1:-1] >= smoothed[:-2]) &
        (smoothed[1:-1] >= smoothed[2:]) &
        (smoothed[1:-1] >= threshold)
    )
    candidate_indices = np.flatnonzero(local_maximum)
    if candidate_indices.size == 0:
        candidate_indices = np.asarray([int(np.argmax(smoothed))], np.int32)

    ordered = sorted(
        candidate_indices.tolist(),
        key=lambda index: float(smoothed[index]),
        reverse=True,
    )
    selected: list[int] = []
    for index in ordered:
        if all(abs(index - previous) >= minimum_distance for previous in selected):
            selected.append(index)
            if len(selected) >= maximum_count:
                break

    return sorted(
        [(start + index, float(smoothed[index])) for index in selected],
        key=lambda item: item[0],
    )


def localize_turning_point(
    score: np.ndarray,
    mask: np.ndarray,
    peak_y: int,
    x_start: int,
    x_stop: int,
    side: str,
    vertical_radius: int,
) -> tuple[float, float, float] | None:
    """在一个侧边亮峰附近定位真正的左/右极值点。"""

    height = score.shape[0]
    y_start = max(0, peak_y - vertical_radius)
    y_stop = min(height, peak_y + vertical_radius + 1)
    local_mask = mask[y_start:y_stop, x_start:x_stop] > 0
    rows, columns = np.nonzero(local_mask)
    if rows.size < 3:
        return None

    values = score[y_start:y_stop, x_start:x_stop][rows, columns]
    # 只使用局部较亮像素，避免稀疏噪点把拐点拉向中间。
    brightness_limit = float(np.percentile(values, 45.0))
    bright = values >= brightness_limit
    rows = rows[bright]
    columns = columns[bright]
    values = values[bright]
    if rows.size < 2:
        return None

    global_x = columns.astype(np.float32) + float(x_start)
    global_y = rows.astype(np.float32) + float(y_start)
    edge_quantile = 18.0 if side == "left" else 82.0
    edge_x = float(np.percentile(global_x, edge_quantile))
    edge_margin = max(2.0, (x_stop - x_start) * 0.035)
    if side == "left":
        edge_pixels = global_x <= edge_x + edge_margin
    else:
        edge_pixels = global_x >= edge_x - edge_margin

    global_x = global_x[edge_pixels]
    global_y = global_y[edge_pixels]
    values = values[edge_pixels]
    if global_x.size == 0:
        return None

    weights = np.maximum(values - float(np.min(values)) + 1.0, 1.0)
    weight_sum = float(np.sum(weights))
    x_px = float(np.sum(global_x * weights) / weight_sum)
    y_px = float(np.sum(global_y * weights) / weight_sum)
    strength = float(np.max(values))
    return x_px, y_px, strength


def select_alternating_edge_points(
    candidates: list[tuple[float, float, float]],
    left_x: float,
    right_x: float,
) -> list[tuple[float, float, float]]:
    """选择左右极值交替、点数最多且总强度最高的候选序列。

    X 轴为正弦波时，相邻真实拐点一定在左右两侧交替出现。动态规划允许从
    任意一侧开始，也不假定最终点数，因此能删除靠近边界的局部强反光，而
    不需要为 0.1/0.5/2 ms 三档分别写死点数。
    """

    ordered = sorted(candidates, key=lambda item: item[1], reverse=True)
    if len(ordered) < 2:
        return ordered

    sides = [
        0 if abs(item[0] - left_x) <= abs(item[0] - right_x) else 1
        for item in ordered
    ]
    # 每个状态保存：序列长度、累计强度、候选下标路径。
    states: list[tuple[int, float, list[int]]] = []
    for index, candidate in enumerate(ordered):
        best = (1, float(candidate[2]), [index])
        for previous in range(index):
            if sides[previous] == sides[index]:
                continue
            previous_state = states[previous]
            proposal = (
                previous_state[0] + 1,
                previous_state[1] + float(candidate[2]),
                previous_state[2] + [index],
            )
            if proposal[:2] > best[:2]:
                best = proposal
        states.append(best)

    winner = max(states, key=lambda state: (state[0], state[1]))
    return [ordered[index] for index in winner[2]]


def extract_waveform_points(
    screen: np.ndarray,
    references: ReferenceLines,
    maximum_points: int,
) -> tuple[list[WavePoint], np.ndarray, float, float]:
    """只提取左右侧高亮拐点，不拟合中间的完整波形。"""

    height, width = screen.shape[:2]
    # 固定机位下不再先寻找并屏蔽两条参考亮线。直接根据一次性标定曲线确定
    # -2 V 到 +2 V 的有效锯齿高度，只排除边缘少量像素。
    boundary_x = (width * 0.08, width * 0.92)
    top_limit = min(references.top_y_at(x_px) for x_px in boundary_x)
    bottom_limit = max(references.bottom_y_at(x_px) for x_px in boundary_x)
    edge_margin = max(2, int(round(FIXED_TRACE_EDGE_MARGIN * height / DEFAULT_SCREEN_SIZE[1])))
    full_y_start = max(1, int(math.floor(top_limit)) + edge_margin)
    full_y_stop = min(height - 1, int(math.ceil(bottom_limit)) - edge_margin)
    if not 0.0 < POINT_SEARCH_CENTER_FRACTION <= 1.0:
        raise ValueError("拐点中心搜索比例必须在 0～1 之间")
    full_y_span = full_y_stop - full_y_start
    search_y_span = max(1, int(round(
        full_y_span * POINT_SEARCH_CENTER_FRACTION)))
    trim_top = (full_y_span - search_y_span) // 2
    y_start = full_y_start + trim_top
    y_stop = y_start + search_y_span
    detected_left_x, detected_right_x, score, green_excess = (
        estimate_waveform_edges(screen, y_start, y_stop)
    )
    line_span = detected_right_x - detected_left_x
    side_width = max(24, int(round(line_span * SIDE_SEARCH_FRACTION)))
    left_start = max(1, int(round(detected_left_x - line_span * 0.025)))
    left_stop = min(width - 1, int(round(detected_left_x + side_width)))
    right_start = max(1, int(round(detected_right_x - side_width)))
    right_stop = min(width - 1, int(round(detected_right_x + line_span * 0.025)))
    if y_stop - y_start < 40 or min(left_stop - left_start, right_stop - right_start) < 20:
        raise ValueError("固定标定后的波形区域太小")

    side_bands = ((left_start, left_stop), (right_start, right_stop))

    # 动态定位完成后，阈值只统计左右窄带；中央反光不再进入正式候选集合。
    side_score_values = np.concatenate([
        score[y_start:y_stop, left_start:left_stop].reshape(-1),
        score[y_start:y_stop, right_start:right_stop].reshape(-1),
    ])
    side_green_values = np.concatenate([
        green_excess[y_start:y_stop, left_start:left_stop].reshape(-1),
        green_excess[y_start:y_stop, right_start:right_stop].reshape(-1),
    ])
    score_threshold = max(
        3.0,
        float(np.percentile(side_score_values, SIDE_SCORE_PERCENTILE)),
    )
    green_threshold = max(
        4.0,
        float(np.percentile(side_green_values, SIDE_GREEN_PERCENTILE)),
    )

    trace_mask = np.zeros((height, width), np.uint8)
    for band_start, band_stop in side_bands:
        band_score = score[y_start:y_stop, band_start:band_stop]
        band_green = green_excess[y_start:y_stop, band_start:band_stop]
        trace_mask[y_start:y_stop, band_start:band_stop] = np.where(
            (band_score >= score_threshold) & (band_green >= green_threshold),
            255,
            0,
        ).astype(np.uint8)

    trace_mask = cv2.morphologyEx(
        trace_mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
    )
    # 删除贯穿较长距离的网格线，但保留短小的侧边拐点亮斑。
    long_horizontal = cv2.morphologyEx(
        trace_mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_RECT, (max(38, width // 8), 1)),
    )
    long_vertical = cv2.morphologyEx(
        trace_mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_RECT, (1, max(34, height // 8))),
    )
    long_structures = cv2.bitwise_or(long_horizontal, long_vertical)
    # 粗亮线主体被开运算识别后，再轻微扩张以连同端点光晕一起删除。
    # 真实拐点的水平长度远小于上面的长核，不会进入 long_structures。
    long_structures = cv2.dilate(
        long_structures,
        cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (max(5, width // 80) | 1, max(3, height // 170) | 1),
        ),
    )
    trace_mask = cv2.subtract(trace_mask, long_structures)

    def side_profile(band_start: int, band_stop: int, edge_x: float) -> np.ndarray:
        band_score = score[:, band_start:band_stop]
        band_mask = trace_mask[:, band_start:band_stop] > 0
        columns = np.arange(band_start, band_stop, dtype=np.float32)
        distance = np.abs(columns - float(edge_x))
        edge_weight = np.clip(1.0 - distance / max(side_width, 1), 0.18, 1.0)
        weighted = np.where(band_mask, band_score * edge_weight[None, :], 0.0)
        # 每行只平均最亮的几个像素，粗细变化不会明显改变峰值位置。
        fraction = min(0.16, max(0.04, 8.0 / max(1, band_stop - band_start)))
        return top_fraction_mean(weighted, fraction)

    left_profile = side_profile(left_start, left_stop, detected_left_x)
    right_profile = side_profile(right_start, right_stop, detected_right_x)
    # 最坏情况 0.1 ms x 100 kHz 约有 10 个周期，即每侧约 10 个拐点。
    # 6% 的纵向间距可保留这些峰，同时继续抑制同一粗亮斑的重复峰。
    same_side_gap = max(8, int(round((y_stop - y_start) * 0.06)))
    per_side_limit = max(2, (max(1, int(maximum_points)) + 1) // 2 + 1)
    left_peaks = find_profile_peaks(
        left_profile, y_start, y_stop, same_side_gap, per_side_limit)
    right_peaks = find_profile_peaks(
        right_profile, y_start, y_stop, same_side_gap, per_side_limit)

    vertical_radius = max(6, int(round((y_stop - y_start) * 0.035)))
    turning_points: list[tuple[float, float, float]] = []
    for peak_y, _ in left_peaks:
        point = localize_turning_point(
            score, trace_mask, peak_y, left_start, left_stop,
            "left", vertical_radius)
        if point is not None:
            turning_points.append(point)
    for peak_y, _ in right_peaks:
        point = localize_turning_point(
            score, trace_mask, peak_y, right_start, right_stop,
            "right", vertical_radius)
        if point is not None:
            turning_points.append(point)

    # 大面积反光可能比 CRT 轨迹更亮，但不会贴合本帧自动估计的两条极值线。
    edge_tolerance = max(
        7.0,
        line_span * TURNING_POINT_EDGE_TOLERANCE_FRACTION,
    )
    turning_points = [
        candidate
        for candidate in turning_points
        if min(
            abs(candidate[0] - detected_left_x),
            abs(candidate[0] - detected_right_x),
        ) <= edge_tolerance
    ]

    # 同一纵向位置只允许一个拐点，抑制网格交点形成的左右重复检测。
    global_gap = max(7, int(round((y_stop - y_start) * 0.035)))
    selected: list[tuple[float, float, float]] = []
    for candidate in sorted(turning_points, key=lambda item: item[2], reverse=True):
        if all(abs(candidate[1] - previous[1]) >= global_gap for previous in selected):
            selected.append(candidate)
    point_limit = max(1, int(maximum_points))
    selected = sorted(selected[:point_limit], key=lambda item: item[1], reverse=True)
    selected = select_alternating_edge_points(
        selected,
        detected_left_x,
        detected_right_x,
    )

    points: list[WavePoint] = []

    # FPGA 的有效锯齿扫描从 -2 V 开始并逐渐升到 +2 V：
    # 屏幕下方对应扫描起点，屏幕上方对应扫描终点。
    # 每个点使用固定标定曲线在自身 X 位置处的上下边界，不从当前图像检测。
    # 这样既不依赖参考亮线，又保留 CRT 几何失真的一次性补偿。
    for x_px, y_px, strength in selected:
        x_normalized = (
            2.0 * (x_px - detected_left_x) /
            max(1.0, detected_right_x - detected_left_x) - 1.0
        )
        local_top_y = references.top_y_at(x_px)
        local_bottom_y = references.bottom_y_at(x_px)
        y_normalized = (
            1.0 - 2.0 * (y_px - local_top_y) /
            max(1.0, local_bottom_y - local_top_y)
        )
        points.append(WavePoint(
            x_px=x_px,
            y_px=y_px,
            x_normalized=float(np.clip(x_normalized, -1.25, 1.25)),
            y_normalized=float(np.clip(y_normalized, -1.1, 1.1)),
            y_volts=float(np.clip(2.0 * y_normalized, -2.2, 2.2)),
            time_normalized=float(np.clip((y_normalized + 1.0) * 0.5, 0.0, 1.0)),
            strength=float(np.clip(strength / 180.0, 0.0, 1.0)),
        ))

    # 局部曲线归一化后再按扫描时间排序，避免较强几何失真改变点的先后次序。
    points.sort(key=lambda point: point.time_normalized)

    if len(points) < min(2, point_limit):
        raise ValueError(f"只提取到 {len(points)} 个侧边拐点")
    return points, trace_mask, detected_left_x, detected_right_x


# ==================== 相位间隔计算（稳健版） ====================

def compute_same_side_period_samples(
    points: list[WavePoint],
) -> list[tuple[float, float]]:
    """计算左侧到左侧、右侧到右侧的完整周期。

    返回 ``(归一化完整周期, Y 像素周期)``。同侧做差可以自然抵消左右侧之间
    固定的纵向偏移，不再把一短一长的两种半周期混在一起。
    """

    side_points: dict[int, list[WavePoint]] = {0: [], 1: []}
    for point in points:
        side = 0 if point.x_normalized < 0.0 else 1
        side_points[side].append(point)

    samples: list[tuple[float, float]] = []
    for same_side_points in side_points.values():
        ordered = sorted(
            same_side_points,
            key=lambda point: point.time_normalized,
        )
        for first, second in zip(ordered, ordered[1:]):
            pixel_period = abs(second.y_px - first.y_px)
            if pixel_period <= 0.0:
                continue
            normalized_period = pixel_period / FREQUENCY_RAMP_HEIGHT_PX
            samples.append((normalized_period, pixel_period))
    return samples


def compute_phase_intervals(points: list[WavePoint]) -> list[float]:
    """返回同侧到同侧的归一化完整周期，保留原函数名以兼容旧调用。"""

    return [sample[0] for sample in compute_same_side_period_samples(points)]


def select_standard_period_samples(
    samples: list[tuple[float, float]],
) -> list[tuple[float, float]]:
    """保留占多数的标准周期，剔除漏点形成的二倍及以上长间距。"""

    if len(samples) < 2:
        return []

    pixel_periods = np.asarray([sample[1] for sample in samples], np.float64)
    standard_center = float(np.median(pixel_periods))
    if standard_center <= 0.0:
        return []

    long_period_limit = standard_center * LONG_PERIOD_RATIO_MIN
    lower = standard_center * (1.0 - STANDARD_PERIOD_TOLERANCE)
    upper = standard_center * (1.0 + STANDARD_PERIOD_TOLERANCE)
    return [
        sample
        for sample in samples
        if sample[1] < long_period_limit and lower <= sample[1] <= upper
    ]


def compute_robust_phase_interval(
    points: list[WavePoint],
    ramp_duration_us: float = EFFECTIVE_RAMP_DURATION_US,
) -> tuple[float, float, int, float]:
    """
    左右侧分别计算同侧到同侧的完整周期，再保留占多数的标准周期簇。
    漏检同侧点产生的二倍及以上长间距不参与频率计算。
    返回：(完整周期归一化间隔, 标准差, 有效周期数, 估计频率Hz)
    """
    samples = compute_same_side_period_samples(points)
    standard_samples = select_standard_period_samples(samples)
    if len(standard_samples) < 2:
        return 0.0, 0.0, 0, 0.0

    normalized_periods = np.asarray(
        [sample[0] for sample in standard_samples],
        np.float64,
    )
    avg_interval = float(np.median(normalized_periods))
    std_interval = float(np.std(normalized_periods))
    valid_count = len(standard_samples)

    # 归一化间隔是一个完整周期，频率公式不再乘 2。
    if ramp_duration_us <= 0.0:
        raise ValueError("锯齿持续时间必须大于 0")
    ramp_duration_sec = ramp_duration_us / 1_000_000.0
    period_sec = avg_interval * ramp_duration_sec
    freq_hz = 1.0 / period_sec if period_sec > 0.0 else 0.0

    return avg_interval, std_interval, valid_count, freq_hz


# ================================================================

def prepare_display_background(screen: np.ndarray) -> np.ndarray:
    """生成仅供人眼观察的去条纹背景，不参与任何测量。

    先根据每一行的中位亮度估计横向扫描带，再做轻微空间平滑。波形只占
    一行中的少量像素，因此中位数不会把绿色波形本身当成扫描带消除。
    """

    if screen.ndim != 3 or screen.shape[2] != 3:
        raise ValueError("结果预览要求 BGR 彩色图像")

    gray = cv2.cvtColor(screen, cv2.COLOR_BGR2GRAY)
    row_level = np.median(gray, axis=1).astype(np.float32).reshape(-1, 1)
    smooth_kernel = max(9, int(round(screen.shape[0] * 0.055)) | 1)
    smooth_row_level = cv2.GaussianBlur(
        row_level, (1, smooth_kernel), 0).reshape(-1)
    stripe_offset = row_level.reshape(-1) - smooth_row_level

    corrected = (
        screen.astype(np.float32)
        - 0.82 * stripe_offset[:, None, None]
    )
    corrected = np.clip(corrected, 0, 255).astype(np.uint8)

    # 纵向平滑进一步压低一两像素宽的扫描线，同时保留较粗的绿色轨迹。
    softened = cv2.GaussianBlur(corrected, (3, 5), 0)
    return cv2.addWeighted(corrected, 0.38, softened, 0.62, -4.0)


def draw_labeled_point(
    canvas: np.ndarray,
    center: tuple[int, int],
    label: str,
    image_width: int,
    image_top: int,
    image_bottom: int,
) -> None:
    """绘制在绿色波形和扫描线背景上仍清楚可见的拐点标记。"""

    # 黑色阴影、白色外圈和红色中心形成三层对比，亮背景和暗背景都能看清。
    cv2.circle(canvas, center, 12, (0, 0, 0), -1, cv2.LINE_AA)
    cv2.circle(canvas, center, 9, (255, 255, 255), -1, cv2.LINE_AA)
    cv2.circle(canvas, center, 5, (0, 45, 255), -1, cv2.LINE_AA)

    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.52
    thickness = 2
    (text_width, text_height), baseline = cv2.getTextSize(
        label, font, font_scale, thickness)

    # 左侧拐点的编号放在右边，右侧拐点的编号放在左边，避免贴出窗口。
    if center[0] < image_width // 2:
        text_x = center[0] + 16
    else:
        text_x = center[0] - 16 - text_width
    text_x = int(np.clip(text_x, 5, max(5, image_width - text_width - 6)))
    text_y = int(np.clip(
        center[1] + text_height // 2,
        image_top + text_height + 6,
        image_bottom - baseline - 6,
    ))

    box_left = text_x - 4
    box_top = text_y - text_height - 4
    box_right = text_x + text_width + 4
    box_bottom = text_y + baseline + 4
    cv2.rectangle(
        canvas, (box_left, box_top), (box_right, box_bottom),
        (10, 13, 16), -1, cv2.LINE_AA)
    cv2.rectangle(
        canvas, (box_left, box_top), (box_right, box_bottom),
        (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(
        canvas, label, (text_x, text_y), font, font_scale,
        (255, 255, 255), thickness, cv2.LINE_AA)


def draw_result(
    screen: np.ndarray,
    references: ReferenceLines,
    points: list[WavePoint],
    avg_interval: float,
    interval_std: float,
    valid_count: int,
    freq_hz: float,
) -> np.ndarray:
    """生成信息栏与波形分离的清晰结果面板。"""

    preview = prepare_display_background(screen)
    image_height, image_width = preview.shape[:2]
    header_height = DISPLAY_HEADER_HEIGHT
    canvas = np.full(
        (image_height + header_height, image_width, 3),
        (18, 22, 26),
        np.uint8,
    )
    canvas[header_height:, :] = preview
    cv2.line(
        canvas, (0, header_height - 1), (image_width - 1, header_height - 1),
        (80, 92, 102), 1, cv2.LINE_AA)

    # 所有状态文字都放在独立实色信息栏中，不再覆盖波形。
    wide_layout = image_width >= 520
    title_text = (
        f"DETECTED  {len(points)}  TURNING POINTS"
        if wide_layout else f"POINTS  {len(points)}"
    )
    cv2.putText(
        canvas,
        title_text,
        (14, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        (245, 248, 250),
        2,
        cv2.LINE_AA,
    )
    calibration_text = "AUTO X-EDGE"
    (cal_width, _), _ = cv2.getTextSize(
        calibration_text, cv2.FONT_HERSHEY_SIMPLEX, 0.48, 1)
    cv2.putText(
        canvas,
        calibration_text,
        (max(14, image_width - cal_width - 14), 27),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (80, 220, 255),
        1,
        cv2.LINE_AA,
    )

    if valid_count > 0:
        frequency_label = "FREQ" if valid_count >= 3 else "FREQ EST"
        frequency_text = (
            f"{frequency_label} {freq_hz / 1000.0:.3f} kHz"
            if freq_hz >= 1000.0 else f"{frequency_label} {freq_hz:.1f} Hz"
        )
        if wide_layout:
            metric_text = (
                f"PERIOD {avg_interval:.4f}    STD {interval_std:.4f}    N {valid_count}"
            )
            metric_x = 220
            metric_scale = 0.47
        else:
            metric_text = (
                f"T {avg_interval:.3f}  S {interval_std:.3f}  N {valid_count}"
            )
            metric_x = 142
            metric_scale = 0.38
        cv2.putText(
            canvas, frequency_text, (14, 61),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60 if wide_layout else 0.48,
            (0, 230, 255), 2,
            cv2.LINE_AA)
        cv2.putText(
            canvas, metric_text, (metric_x, 60),
            cv2.FONT_HERSHEY_SIMPLEX, metric_scale, (190, 205, 215), 1,
            cv2.LINE_AA)
    else:
        cv2.putText(
            canvas, "WAITING FOR ENOUGH POINTS", (14, 61),
            cv2.FONT_HERSHEY_SIMPLEX, 0.56, (80, 180, 255), 2,
            cv2.LINE_AA)

    # 点按实际锯齿扫描时间从下到上编号；P1 是最早出现的最下方拐点。
    for index, point in enumerate(points, start=1):
        center = (
            int(round(point.x_px)),
            int(round(point.y_px)) + header_height,
        )
        draw_labeled_point(
            canvas,
            center,
            f"P{index}",
            image_width,
            header_height,
            header_height + image_height,
        )

    return canvas


def process_frame(
    frame: np.ndarray,
    screen_size: tuple[int, int],
    maximum_points: int,
    manual_corners: np.ndarray | None,
    ramp_duration_us: float = EFFECTIVE_RAMP_DURATION_US,
    render_overlay: bool = True,
) -> ProcessResult:
    """按固定机位标定处理一帧，并计算稳健的相位间隔和频率。"""

    corners = manual_corners if manual_corners is not None else get_fixed_screen_corners(frame)
    rectified = rectify_screen(frame, corners, screen_size)
    references = get_fixed_reference_calibration(screen_size)
    points, trace_mask, detected_left_x, detected_right_x = (
        extract_waveform_points(rectified, references, maximum_points)
    )
    # 记录本帧实际极值线，供 CSV 归一化结果和后续调试读取。曲线标尺的
    # center/scale 保持固定，因为它们描述的是 CRT 几何而不是波形水平位置。
    references = replace(
        references,
        left_x=detected_left_x,
        right_x=detected_right_x,
    )

    # 计算稳健相位间隔和频率
    avg_interval, std_interval, valid_count, freq_hz = compute_robust_phase_interval(
        points, ramp_duration_us)

    # 实时主循环需要先完成当前帧识别，再用多帧平均背景绘制一次结果，因此可
    # 跳过这里的首次绘制，避免在树莓派上每帧重复做两遍预览去条纹。
    overlay = (
        draw_result(
            rectified,
            references,
            points,
            avg_interval,
            std_interval,
            valid_count,
            freq_hz,
        )
        if render_overlay else rectified
    )
    return ProcessResult(
        corners,
        rectified,
        trace_mask,
        overlay,
        points,
        references,
        avg_interval,
        std_interval,
        valid_count,
        freq_hz,
    )


def draw_corners(frame: np.ndarray, corners: np.ndarray) -> np.ndarray:
    """在原图上绘制检测到的屏幕区域。"""

    output = frame.copy()
    cv2.polylines(output, [corners.round().astype(np.int32)], True,
                  (0, 0, 255), 3, cv2.LINE_AA)
    return output


def save_result(output_dir: Path, frame: np.ndarray, result: ProcessResult) -> None:
    """保存图片和 CSV，并在 CSV 中添加平均间隔和频率信息。"""

    output_dir.mkdir(parents=True, exist_ok=True)
    write_image(output_dir / "screen_detection.png", draw_corners(frame, result.corners))
    write_image(output_dir / "rectified.png", result.rectified)
    write_image(output_dir / "trace_mask.png", result.trace_mask)
    write_image(output_dir / "points_overlay.png", result.overlay)

    # CSV 保存原始点数据 + 统计信息
    with (output_dir / "points.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "序号", "X像素", "Y像素", "X归一化", "Y归一化",
            "Y电压", "时间归一化", "置信度",
        ])
        for index, point in enumerate(result.points):
            writer.writerow([
                index,
                f"{point.x_px:.3f}",
                f"{point.y_px:.3f}",
                f"{point.x_normalized:.6f}",
                f"{point.y_normalized:.6f}",
                f"{point.y_volts:.6f}",
                f"{point.time_normalized:.6f}",
                f"{point.strength:.6f}",
            ])
        # 额外写入统计行
        writer.writerow([])
        writer.writerow(["稳健完整周期归一化间隔", f"{result.avg_phase_interval:.6f}"])
        writer.writerow(["标准差", f"{result.phase_interval_std:.6f}"])
        writer.writerow(["有效间隔数", f"{result.valid_interval_count}"])
        writer.writerow(["估计频率 (Hz)", f"{result.frequency_hz:.1f}"])


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="单文件 OpenCV 示波器侧边拐点提取（固定机位版）")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("opencv_output"),
        help="结果保存目录",
    )
    parser.add_argument("--max-points", type=int, default=DEFAULT_MAX_POINTS)
    parser.add_argument("--width", type=int, default=DEFAULT_SCREEN_SIZE[0])
    parser.add_argument("--height", type=int, default=DEFAULT_SCREEN_SIZE[1])
    parser.add_argument(
        "--corners",
        type=float,
        nargs=8,
        metavar=("TL_X", "TL_Y", "TR_X", "TR_Y", "BR_X", "BR_Y", "BL_X", "BL_Y"),
        help="临时覆盖文件顶部的固定屏幕四角：左上、右上、右下、左下",
    )
    parser.add_argument(
        "--ramp-us",
        type=float,
        choices=RAMP_DURATION_CHOICES_US,
        default=EFFECTIVE_RAMP_DURATION_US,
        help="FPGA 有效锯齿持续时间：100、500 或 2000 微秒",
    )
    parser.add_argument("--no-gui", action="store_true", help="不打开调试窗口")
    parser.add_argument(
        "--camera-width", type=int, default=FIXED_CALIBRATION_FRAME_SIZE[0])
    parser.add_argument(
        "--camera-height", type=int, default=FIXED_CALIBRATION_FRAME_SIZE[1])
    parser.add_argument("--camera-fps", type=int, default=30)
    parser.add_argument(
        "--save-every",
        type=int,
        default=0,
        help="每隔多少帧保存调试图，0 表示只在退出时保存",
    )
    parser.add_argument("--exposure", type=float, help="固定曝光值，具体范围由摄像头决定")
    parser.add_argument("--gain", type=float, help="固定增益值")
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    screen_size = (max(320, args.width), max(240, args.height))
    manual_corners = (
        order_corners(np.asarray(args.corners, np.float32).reshape(4, 2))
        if args.corners is not None else None
    )

    # 直接打开默认摄像头
    capture = cv2.VideoCapture(0)
    if not capture.isOpened():
        raise RuntimeError("无法打开摄像头 0")

    capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.camera_width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.camera_height)
    capture.set(cv2.CAP_PROP_FPS, args.camera_fps)
    if args.exposure is not None:
        capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        capture.set(cv2.CAP_PROP_EXPOSURE, args.exposure)
    if args.gain is not None:
        capture.set(cv2.CAP_PROP_GAIN, args.gain)

    last_result: ProcessResult | None = None
    last_frame: np.ndarray | None = None
    result_saved = False
    frame_index = 0
    display_accumulator: np.ndarray | None = None
    temporal_period_filter = TemporalPeriodFilter()

    try:
        while True:
            ok, frame = capture.read()
            if not ok or frame is None:
                raise RuntimeError("摄像头没有返回图像")

            try:
                result = process_frame(
                    frame,
                    screen_size,
                    args.max_points,
                    manual_corners,
                    args.ramp_us,
                    False,
                )
                stable_period, stable_frequency_hz = temporal_period_filter.update(
                    result.avg_phase_interval,
                    result.valid_interval_count,
                    args.ramp_us,
                )
                if stable_frequency_hz > 0.0:
                    result = replace(
                        result,
                        avg_phase_interval=stable_period,
                        frequency_hz=stable_frequency_hz,
                    )
                last_result = result
                last_frame = frame
                status_frame = frame if args.no_gui else draw_corners(frame, result.corners)

                # 识别始终使用当前原始帧；多帧平均只替换给人看的背景。
                # 这样可以压低 CRT 扫描带，又不会让历史帧影响拐点坐标和频率。
                rectified_float = result.rectified.astype(np.float32)
                if (
                    display_accumulator is None
                    or display_accumulator.shape != rectified_float.shape
                ):
                    display_accumulator = rectified_float.copy()
                else:
                    cv2.accumulateWeighted(
                        rectified_float,
                        display_accumulator,
                        DISPLAY_TEMPORAL_ALPHA,
                    )
                display_screen = cv2.convertScaleAbs(display_accumulator)
                result.overlay = draw_result(
                    display_screen,
                    result.references,
                    result.points,
                    result.avg_phase_interval,
                    result.phase_interval_std,
                    result.valid_interval_count,
                    result.frequency_hz,
                )

                # 打印信息，包含平均间隔和频率
                if frame_index % 15 == 0:
                    print(
                        f"frame={frame_index} points={len(result.points)} "
                        f"full_period={result.avg_phase_interval:.4f} "
                        f"freq={result.frequency_hz:.0f}Hz  n={result.valid_interval_count}",
                        end="\r"
                    )
            except Exception as error:
                status_frame = frame.copy()
                cv2.putText(status_frame, f"ERROR: {error}", (20, 38),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2,
                            cv2.LINE_AA)
                print(f"frame={frame_index} error={error}")
                result = None

            periodic_save = args.save_every > 0 and frame_index % max(1, args.save_every) == 0
            if result is not None and periodic_save:
                save_result(args.output_dir, frame, result)
                result_saved = True

            if not args.no_gui:
                cv2.imshow("1-original-screen", status_frame)
                if result is not None:
                    cv2.imshow("2-rectified", result.rectified)
                    cv2.imshow("3-trace-mask", result.trace_mask)
                    cv2.imshow("4-turning-points", result.overlay)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key == ord("s") and last_result is not None:
                    save_result(args.output_dir, frame, last_result)
                    result_saved = True

            frame_index += 1
    except KeyboardInterrupt:
        pass
    finally:
        capture.release()
        cv2.destroyAllWindows()

    if last_result is None:
        return 1
    if args.no_gui and last_frame is not None:
        save_result(args.output_dir, last_frame, last_result)
        result_saved = True
    if result_saved:
        print(f"\n结果已保存到：{args.output_dir.resolve()}")
    else:
        print("\n运行结束；按 S 可在调试界面中保存结果")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())



# ========== 控制器模块 ==========


from dataclasses import dataclass
from typing import Any


    CMD_ACK,
    CMD_DISABLE,
    CMD_NACK,
    CMD_PROBE_DUAL,
    CMD_PROBE_SINGLE,
    CMD_TARGET,
    ERROR_CAMERA,
    ERROR_CANCELLED,
    ERROR_COARSE_FAILED,
    ERROR_PHASE_UNSTABLE,
    ERROR_TIMEOUT,
    ERROR_VISUAL_RANGE,
    STATE_COARSE,
    STATE_ERROR,
    STATE_FINE_PHASE,
    STATE_LOCKED,
    STATE_TRACK,
    STATUS_ERROR,
    STATUS_LOCKED,
    STATUS_PROGRESS,
    FLAG_ACK_REQUEST,
    Frame,
    RESULT_ACCEPTED,
    RESULT_DUPLICATE,
    SerialLink,
    locked_payload,
    progress_payload,
)
    CoarseFrameObservation,
    CoarseMeasurement,
    FrequencyEstimator,
    TargetAnalyzer,
    TraceExtractor,
    aggregate_masks,
    circular_mean_cycles,
    coarse_observation_from_points,
    resolve_dual_interval_frequency,
    summarize_coarse_observations,
    wrap_cycles,
)



@dataclass(frozen=True)
class PhaseBlock:
    phase_cycles: float
    confidence: float


@dataclass
class PendingCommand:
    frame: Frame
    purpose: str
    retries: int
    deadline: float


class AutoLissajousController:
    """Non-blocking Task5 state machine driven by :meth:`poll`."""

    COARSE_WIDTH_CODES = (0, 1, 2)

    def __init__(self, config: dict[str, Any], link: SerialLink,
                 camera: ScopeCamera) -> None:
        self.config = config
        self.link = link
        self.camera = camera
        configured_widths = config.get("coarse", {}).get(
            "widths", self.COARSE_WIDTH_CODES)
        widths = tuple(int(value) for value in configured_widths)
        self._coarse_width_codes = (
            widths if widths and all(0 <= value <= 3 for value in widths)
            else self.COARSE_WIDTH_CODES)
        self.extractor = TraceExtractor(config)
        self.frequency = FrequencyEstimator()
        self.target_analyzer = TargetAnalyzer(config)
        self.circle_detector = CircleDetector(config)
        self.frequency_scanner = FrequencyScanner(config)
        self.probe_count = 0
        self._preview = bool(config.get("runtime", {}).get("preview", False))
        self._mode = "IDLE"
        self._target = 0
        self._run_started = 0.0
        self._deadline = 0.0
        self._coarse_index = 0
        self._coarse_width_code = 0
        self._coarse_frequency_hz = 0.0
        self._coarse_quality = 0
        self._coarse_points = 0
        self._coarse_observations: list[CoarseFrameObservation] = []
        self._fine_width_code = 0
        self._fine_interval_index = 0
        self._fine_frame_phases: list[tuple[float, float]] = []
        self._fine_blocks: dict[int, list[PhaseBlock]] = {0: [], 1: []}
        self._fine_frame_attempts = 0
        self._fine_round = 0
        self._final_frequency_hz = 0.0
        self._frequency_correction_hz = 0.0
        self._tuning_word = 0
        self._phase = 0
        self._amplitude = 255
        self._track_masks: list[np.ndarray] = []
        self._track_attempt = 0
        self._stable_since = 0.0
        self._last_status = 0.0
        self._pending_command: PendingCommand | None = None
        self._fallback_sequence = 0

        # 频率扫描相关状态
        self._scan_points: list[float] = []
        self._scan_index = 0
        self._scan_results: list[tuple[float, int, bool]] = []
        self._best_circle_freq = 0.0
        self._best_circle_quality = 0
        self._circle_scan_attempts = 0

    @property
    def active(self) -> bool:
        return self._mode not in ("IDLE", "ERROR", "LOCKED_HOLD")

    @property
    def mode(self) -> str:
        return self._mode

    def handle_frame(self, frame: Frame, now: float | None = None) -> bool:
        """Consume the STM32's acknowledgement of a requested FPGA change."""
        if frame.command not in (CMD_ACK, CMD_NACK):
            return False
        pending = self._pending_command
        if pending is None:
            return False
        if (frame.payload[0] != pending.frame.sequence or
                frame.payload[1] != pending.frame.command):
            return False

        timestamp = time.monotonic() if now is None else float(now)
        result = frame.payload[2]
        self._pending_command = None
        if frame.command != CMD_ACK or result not in (
                RESULT_ACCEPTED, RESULT_DUPLICATE):
            self._fail(
                ERROR_TIMEOUT,
                f"FPGA rejected {pending.purpose} result={result}",
            )
            return True

        if pending.purpose == "coarse probe":
            self._mode = "COARSE_SETTLE"
            self._deadline = timestamp + float(
                self.config.get("coarse", {}).get("settle_s", 0.18))
        elif pending.purpose.startswith("fine "):
            self._mode = "FINE_SETTLE"
            self._deadline = timestamp + float(
                self.config.get("fine_phase", {}).get("settle_s", 0.18))
        elif pending.purpose in ("initial target", "target correction", "best circle frequency"):
            self._mode = "TRACK_SETTLE"
            self._deadline = timestamp + float(
                self.config.get("runtime", {}).get("settle_s", 0.18))
            self._track_masks = []
        elif pending.purpose.startswith("circle scan"):
            self._mode = "CIRCLE_SCAN_SETTLE"
            self._deadline = timestamp + float(
                self.config.get("runtime", {}).get("settle_s", 0.18))
            self._track_masks = []
        else:
            self._fail(ERROR_TIMEOUT, f"unknown acknowledged action {pending.purpose}")
        return True

    def _send_progress(self, state: int, stage: int, quality: int = 0,
                       point_count: int = 0,
                       frequency_hz: float = 0.0) -> None:
        frequency_millihz = max(0, int(round(frequency_hz * 1000.0)))
        self.link.send(STATUS_PROGRESS, progress_payload(
            state, stage, quality, point_count, frequency_millihz))

    def _send_acknowledged(self, command: int, payload: bytes) -> Frame | None:
        """Issue a command that must be applied by FPGA before vision moves on."""
        send_frame = getattr(self.link, "send_frame", None)
        if callable(send_frame):
            return send_frame(command, payload, flags=FLAG_ACK_REQUEST)

        # Test and offline helper links from earlier revisions expose only
        # send().  Retain that narrow compatibility path without weakening the
        # real serial path, which always uses a specific on-wire sequence.
        try:
            sent = self.link.send(command, payload)
        except (OSError, RuntimeError):
            return None
        if sent is False:
            return None
        frame = Frame(self._fallback_sequence, command, payload,
                      FLAG_ACK_REQUEST)
        self._fallback_sequence = (self._fallback_sequence + 1) & 0xFF
        return frame

    def _start_ack_wait(self, frame: Frame | None, purpose: str,
                        now: float) -> bool:
        if frame is None:
            self._fail(ERROR_TIMEOUT, f"serial send failed for {purpose}")
            return False
        protocol = self.config.get("protocol", {})
        timeout = max(0.05, float(protocol.get("ack_timeout_s", 0.60)))
        self._pending_command = PendingCommand(frame, purpose, 0, now + timeout)
        return True

    def _resend_pending(self, pending: PendingCommand) -> bool:
        resend = getattr(self.link, "resend", None)
        if callable(resend):
            return resend(pending.frame) is not None
        try:
            result = self.link.send(pending.frame.command, pending.frame.payload)
        except (OSError, RuntimeError):
            return False
        return result is not False

    def _poll_pending_command(self, now: float) -> None:
        pending = self._pending_command
        if pending is None or now < pending.deadline:
            return
        protocol = self.config.get("protocol", {})
        maximum_retries = max(0, int(protocol.get("ack_retries", 3)))
        if pending.retries < maximum_retries and self._resend_pending(pending):
            pending.retries += 1
            pending.deadline = now + max(
                0.05, float(protocol.get("ack_timeout_s", 0.60)))
            LOGGER.warning("retrying %s seq=%d attempt=%d", pending.purpose,
                           pending.frame.sequence, pending.retries)
            return
        self._pending_command = None
        self._fail(ERROR_TIMEOUT,
                   f"no FPGA ACK for {pending.purpose} seq={pending.frame.sequence}")

    def _send_probe(self, command: int, width_code: int,
                    interval_index: int = 0) -> Frame | None:
        payload = bytearray(8)
        payload[0] = width_code & 0xFF
        payload[1] = interval_index & 0x01
        frame = self._send_acknowledged(command, bytes(payload))
        self.probe_count += 1
        return frame

    def _send_target(self, target: int, amplitude: int, phase: int,
                     tuning_word: int) -> Frame | None:
        payload = bytearray(8)
        payload[0] = target
        payload[1] = max(1, min(255, amplitude))
        payload[2] = phase & 0xFF
        payload[3] = 0
        payload[4:8] = int(tuning_word).to_bytes(4, "little", signed=False)
        return self._send_acknowledged(CMD_TARGET, bytes(payload))

    @staticmethod
    def _phase_delta(estimated: int, target: int) -> int:
        desired = (64, 192) if target == 2 else (0, 128)
        deltas = [((phase - estimated + 128) & 0xFF) - 128
                  for phase in desired]
        return min(deltas, key=abs)

    def start(self, target: int, now: float | None = None) -> bool:
        if target not in (1, 2, 3):
            return False
        timestamp = time.monotonic() if now is None else float(now)
        self._target = target
        self._run_started = timestamp
        self._pending_command = None
        self.probe_count = 0
        self._coarse_index = 0
        self._coarse_frequency_hz = 0.0
        self._coarse_quality = 0
        self._coarse_points = 0
        self._fine_round = 0
        self._fine_blocks = {0: [], 1: []}
        self._track_attempt = 0
        self._begin_coarse(timestamp)
        return self._mode != "ERROR"

    def cancel(self, target: int = 0) -> None:
        if self._mode not in ("IDLE", "ERROR"):
            payload = bytes((ERROR_CANCELLED, self._target & 0xFF, 0, 0,
                             0, 0, 0, 0))
            self.link.send(STATUS_ERROR, payload)
        self._mode = "IDLE"
        self._target = 0
        self._pending_command = None

    def _begin_coarse(self, now: float) -> None:
        coarse = self.config.get("coarse", {})
        self._coarse_width_code = self._coarse_width_codes[self._coarse_index]
        self._coarse_observations = []
        frame = self._send_probe(CMD_PROBE_SINGLE, self._coarse_width_code)
        self._send_progress(STATE_COARSE, self._coarse_width_code)
        if self._start_ack_wait(frame, "coarse probe", now):
            self._mode = "COARSE_WAIT_ACK"

    def _start_coarse_capture(self, now: float) -> None:
        duration = float(self.config.get("coarse", {}).get(
            "capture_seconds", 1.0))
        self._coarse_observations = []
        self._deadline = now + duration
        self._mode = "COARSE_CAPTURE"

    def _capture_coarse_frame(self) -> None:
        coarse = self.config.get("coarse", {})
        width_us = self.frequency.WIDTHS_US[self._coarse_width_code]
        maximum_points = int(coarse.get("extract_maximum_points", 32))
        size_values = coarse.get("screen_size", list(DEFAULT_SCREEN_SIZE))
        screen_size = (int(size_values[0]), int(size_values[1]))
        try:
            frame = (self.camera.read_raw() if hasattr(self.camera, "read_raw")
                     else self.camera.read())
            result = process_frame(
                frame, screen_size, maximum_points, None, width_us, False)
            observation = coarse_observation_from_points(result.points)
            if self._preview:
                cv2.imshow("scope", result.rectified)
                cv2.imshow("turning-points", result.trace_mask)
                cv2.waitKey(1)
        except (ValueError, RuntimeError) as exc:
            LOGGER.debug("coarse frame rejected: %s", exc)
            observation = CoarseFrameObservation(0, (), (), 0.0)
        self._coarse_observations.append(observation)

    def _coarse_summary(self) -> CoarseMeasurement:
        coarse = self.config.get("coarse", {})
        width_us = self.frequency.WIDTHS_US[self._coarse_width_code]
        return summarize_coarse_observations(
            self._coarse_observations,
            width_us,
            minimum_points=int(coarse.get("minimum_points", 5)),
            minimum_periods=int(coarse.get("minimum_complete_periods", 3)),
            minimum_valid_ratio=float(coarse.get("minimum_valid_ratio", 0.70)),
            maximum_cv=float(coarse.get("maximum_cv", 0.08)),
            minimum_confidence=float(coarse.get("minimum_confidence", 0.35)),
            maximum_points=int(coarse.get("maximum_points", 22)),
        )

    def _finish_coarse(self, now: float) -> None:
        summary = self._coarse_summary()
        LOGGER.info(
            "coarse %dus: accepted=%s f=%.3fHz points=%d valid=%.1f%% "
            "cv=%.4f q=%.3f reason=%s",
            int(self.frequency.WIDTHS_US[self._coarse_width_code]),
            summary.accepted, summary.frequency_hz,
            summary.median_point_count, summary.valid_frame_ratio * 100.0,
            summary.period_cv, summary.confidence, summary.reason)
        quality = int(round(summary.confidence * 100.0))
        self._send_progress(
            STATE_COARSE, self._coarse_width_code, quality,
            summary.median_point_count,
            summary.frequency_hz if summary.accepted else 0.0)
        if (summary.reason == "VISUAL_RANGE_HIGH" and
                self._coarse_width_code == 0):
            self._fail(ERROR_VISUAL_RANGE, "100 us trace is over-dense")
            return
        frequency_valid = 900.0 <= summary.frequency_hz <= 101_000.0
        if not summary.accepted or not frequency_valid:
            self._coarse_index += 1
            if self._coarse_index >= len(self._coarse_width_codes):
                self._fail(ERROR_COARSE_FAILED,
                           f"2 ms coarse measurement failed: {summary.reason}")
            else:
                self._begin_coarse(now)
            return

        self._coarse_frequency_hz = summary.frequency_hz
        self._coarse_quality = quality
        self._coarse_points = summary.median_point_count
        self._fine_width_code = 0 if summary.frequency_hz > 5000.0 else 1
        self._fine_blocks = {0: [], 1: []}
        self._fine_round = 0
        self._begin_fine_interval(0, now)

    def _begin_fine_interval(self, interval_index: int, now: float) -> None:
        fine = self.config.get("fine_phase", {})
        self._fine_interval_index = interval_index
        self._fine_frame_phases = []
        self._fine_frame_attempts = 0
        frame = self._send_probe(
            CMD_PROBE_DUAL, self._fine_width_code, interval_index)
        self._send_progress(
            STATE_FINE_PHASE, interval_index, self._coarse_quality,
            self._coarse_points, self._coarse_frequency_hz)
        if self._start_ack_wait(frame, f"fine {3 if interval_index == 0 else 7}ms probe", now):
            self._mode = "FINE_WAIT_ACK"

    def _capture_fine_frame(self, now: float) -> None:
        fine = self.config.get("fine_phase", {})
        self._fine_frame_attempts += 1
        try:
            frame = self.camera.read()
            mask = self.extractor.extract(frame)
            fit = self.frequency.estimate_dual_phase(
                mask, self._fine_width_code, self._coarse_frequency_hz)
            minimum_fit = float(fine.get("minimum_fit_confidence", 0.35))
            if fit.confidence >= minimum_fit:
                self._fine_frame_phases.append(
                    (fit.phase_difference_cycles, fit.confidence))
            if self._preview:
                cv2.imshow("scope", frame)
                cv2.imshow("trace", mask)
                cv2.waitKey(1)
        except (ValueError, RuntimeError) as exc:
            LOGGER.debug("fine frame rejected: %s", exc)

        frames_per_block = int(fine.get("frames_per_block", 3))
        required_blocks = int(fine.get("required_blocks", 3))
        maximum_attempts = int(fine.get(
            "maximum_frame_attempts", frames_per_block * required_blocks * 4))
        if len(self._fine_frame_phases) >= frames_per_block:
            samples = self._fine_frame_phases[:frames_per_block]
            phase = circular_mean_cycles([item[0] for item in samples])
            confidence = float(np.median([item[1] for item in samples]))
            block = PhaseBlock(phase, confidence)
            existing = self._fine_blocks[self._fine_interval_index]
            if existing:
                distance = abs(wrap_cycles(phase - existing[-1].phase_cycles))
                if distance > float(fine.get("block_phase_tolerance_cycles", 0.08)):
                    existing.clear()
            existing.append(block)
            self._fine_frame_phases = []
            if len(existing) >= required_blocks:
                if self._fine_interval_index == 0:
                    self._begin_fine_interval(1, now)
                else:
                    self._finish_fine(now)
                return
        if self._fine_frame_attempts >= maximum_attempts:
            self._retry_fine(now, "not enough stable phase frames")

    def _retry_fine(self, now: float, reason: str) -> None:
        maximum_rounds = int(self.config.get("fine_phase", {}).get(
            "maximum_rounds", 3))
        self._fine_round += 1
        LOGGER.warning("fine phase retry %d: %s", self._fine_round, reason)
        if self._fine_round >= maximum_rounds:
            self._fail(ERROR_PHASE_UNSTABLE, reason)
            return
        self._fine_blocks = {0: [], 1: []}
        self._begin_fine_interval(0, now)

    def _finish_fine(self, now: float) -> None:
        fine = self.config.get("fine_phase", {})
        blocks_3 = self._fine_blocks[0]
        blocks_7 = self._fine_blocks[1]
        required = int(fine.get("required_blocks", 3))
        if len(blocks_3) < required or len(blocks_7) < required:
            self._retry_fine(now, "fewer than three consistent blocks")
            return
        uncertainty = float(fine.get("coarse_uncertainty_hz", 450.0))
        pair_frequencies: list[float] = []
        try:
            for block_3, block_7 in zip(blocks_3[-required:],
                                        blocks_7[-required:]):
                pair = resolve_dual_interval_frequency(
                    self._coarse_frequency_hz,
                    block_3.phase_cycles,
                    block_7.phase_cycles,
                    uncertainty,
                    block_3.confidence,
                    block_7.confidence,
                    float(fine.get("maximum_residual_cycles", 0.08)))
                pair_frequencies.append(pair.frequency_hz)
            if (max(pair_frequencies) - min(pair_frequencies) >
                    float(fine.get("block_frequency_tolerance_hz", 20.0))):
                raise ValueError("three phase blocks select different frequencies")
            mean_3 = circular_mean_cycles(
                [item.phase_cycles for item in blocks_3[-required:]])
            mean_7 = circular_mean_cycles(
                [item.phase_cycles for item in blocks_7[-required:]])
            confidence_3 = float(np.median(
                [item.confidence for item in blocks_3[-required:]]))
            confidence_7 = float(np.median(
                [item.confidence for item in blocks_7[-required:]]))
            final = resolve_dual_interval_frequency(
                self._coarse_frequency_hz, mean_3, mean_7, uncertainty,
                confidence_3, confidence_7,
                float(fine.get("maximum_residual_cycles", 0.08)))
        except ValueError as exc:
            self._retry_fine(now, str(exc))
            return

        self._final_frequency_hz = final.frequency_hz
        self._frequency_correction_hz = final.correction_hz

        # 对于目标2（圆形），启动频率扫描以寻找最佳圆形
        if self._target == 2:
            LOGGER.info("Target 2 (circle): starting frequency scan")
            self._begin_circle_scan(now)
        else:
            # 对于其他目标，直接设置频率
            self._tuning_word = int(round(
                self._final_frequency_hz * (2**32) / 50_000_000.0))
            if not 1 <= self._tuning_word <= 0xFFFFFFFF:
                self._fail(ERROR_PHASE_UNSTABLE, "final DDS tuning word is invalid")
                return
            target_config = self.config.get("target", {})
            amplitude_map = target_config.get("initial_amplitude", {})
            self._amplitude = int(amplitude_map.get(str(self._target), 255))
            self._phase = int(target_config.get("initial_phase", 0)) & 0xFF
            frame = self._send_target(
                self._target, self._amplitude, self._phase, self._tuning_word)
            self._send_progress(
                STATE_TRACK, 0, int(round(final.confidence * 100.0)),
                self._coarse_points, self._final_frequency_hz)
            if self._start_ack_wait(frame, "initial target", now):
                self._mode = "TRACK_WAIT_ACK"

    def _capture_track_frame(self, now: float) -> None:
        runtime = self.config.get("runtime", {})
        required = int(runtime.get("aggregate_frames", 3))
        frame = self.camera.read()
        mask = self.extractor.extract(frame)
        self._track_masks.append(mask)
        if self._preview:
            cv2.imshow("scope", frame)
            cv2.imshow("trace", mask)
            cv2.waitKey(1)
        if len(self._track_masks) < required:
            return
        analysis = self.target_analyzer.analyze(
            aggregate_masks(self._track_masks), self._target)
        self._track_masks = []
        target_config = self.config.get("target", {})
        threshold = int(target_config.get("lock_quality", 65))
        if analysis.quality >= threshold and abs(analysis.span_y_div - 8.0) < 1.0:
            self._enter_locked(now, analysis.quality)
            return
        self._track_attempt += 1
        if self._track_attempt >= int(target_config.get("maximum_corrections", 8)):
            self._fail(ERROR_CAMERA, "target shape did not converge")
            return
        amplitude_min = int(target_config.get("amplitude_min", 96))
        if analysis.span_y_div > 0.5:
            self._amplitude = int(round(
                self._amplitude * 8.0 / analysis.span_y_div))
            self._amplitude = max(amplitude_min, min(255, self._amplitude))
        delta = self._phase_delta(analysis.estimated_phase, self._target)
        direction = 1 if self._track_attempt & 1 else -1
        self._phase = (self._phase + direction * delta) & 0xFF
        frame = self._send_target(
            self._target, self._amplitude, self._phase, self._tuning_word)
        self._send_progress(
            STATE_TRACK, 0, analysis.quality, self._coarse_points,
            self._final_frequency_hz)
        if self._start_ack_wait(frame, "target correction", now):
            self._mode = "TRACK_WAIT_ACK"

    def _enter_locked(self, now: float, quality: int) -> None:
        self.link.send(STATUS_LOCKED, locked_payload(
            self._target, quality, self._coarse_width_code,
            int(round(self._final_frequency_hz * 1000.0))))
        self._send_progress(
            STATE_LOCKED, self._coarse_width_code, quality,
            self._coarse_points, self._final_frequency_hz)
        self._mode = "STABLE"
        self._stable_since = now
        self._last_status = now

    def _poll_stable(self, now: float) -> None:
        target_config = self.config.get("target", {})
        if now - self._last_status >= 0.5:
            self.link.send(STATUS_LOCKED, locked_payload(
                self._target, self._coarse_quality,
                self._coarse_width_code,
                int(round(self._final_frequency_hz * 1000.0))))
            self._last_status = now
        if now - self._stable_since >= float(
                target_config.get("stability_seconds", 5.2)):
            self._mode = "LOCKED_HOLD"

    def _fail(self, code: int, reason: str) -> None:
        LOGGER.error("Task5 error %d: %s", code, reason)
        payload = bytes((code & 0xFF, self._target & 0xFF, 0, 0,
                         0, 0, 0, 0))
        self.link.send(STATUS_ERROR, payload)
        self._send_progress(
            STATE_ERROR, self._coarse_width_code, 0,
            self._coarse_points, 0.0)
        # Do not send TARGET or reuse a stale frequency. The FPGA remains on
        # the most recently requested probe until STM32 starts a new session.
        self._mode = "ERROR"

    def poll(self, now: float | None = None) -> None:
        timestamp = time.monotonic() if now is None else float(now)
        self._poll_pending_command(timestamp)
        if self._mode == "ERROR":
            return
        if self.active:
            timeout = float(self.config.get("target", {}).get(
                "control_timeout_s", 19.5))
            if timestamp - self._run_started > timeout:
                self._fail(ERROR_TIMEOUT, "Task5 control timeout")
                return
        try:
            if self._mode == "COARSE_SETTLE" and timestamp >= self._deadline:
                self._start_coarse_capture(timestamp)
            elif self._mode == "COARSE_CAPTURE":
                if timestamp < self._deadline:
                    self._capture_coarse_frame()
                else:
                    self._finish_coarse(timestamp)
            elif self._mode == "FINE_SETTLE" and timestamp >= self._deadline:
                self._mode = "FINE_CAPTURE"
            elif self._mode == "FINE_CAPTURE":
                self._capture_fine_frame(timestamp)
            elif self._mode == "TRACK_SETTLE" and timestamp >= self._deadline:
                self._mode = "TRACK_CAPTURE"
                self._track_masks = []
            elif self._mode == "TRACK_CAPTURE":
                self._capture_track_frame(timestamp)
            elif self._mode == "CIRCLE_SCAN_SETTLE" and timestamp >= self._deadline:
                self._mode = "CIRCLE_SCAN_CAPTURE"
            elif self._mode == "CIRCLE_SCAN_CAPTURE":
                self._capture_circle_scan_frame(timestamp)
            elif self._mode == "STABLE":
                self._poll_stable(timestamp)
        except Exception as exc:
            LOGGER.exception("Task5 state %s failed", self._mode)
            self._fail(ERROR_CAMERA, str(exc))

    # Compatibility helper retained for the existing static target-correction
    # regression test. Runtime control uses the non-blocking TRACK states above.
    def _capture_mask(self, count: int | None = None):
        runtime = self.config.get("runtime", {})
        frame_count = int(runtime.get("aggregate_frames", 3)
                          if count is None else count)
        interval = float(runtime.get("frame_interval_s", 0.025))
        masks = []
        for _ in range(max(1, frame_count)):
            frame = self.camera.read()
            masks.append(self.extractor.extract(frame))
            if interval > 0:
                time.sleep(interval)
        return aggregate_masks(masks)

    def _settle(self) -> None:
        time.sleep(float(self.config.get("runtime", {}).get("settle_s", 0.18)))

    def _adjust_target(self, target: int, tuning_word: int,
                       phase: int, amplitude: int):
        target_config = self.config.get("target", {})
        amplitude_min = int(target_config.get("amplitude_min", 96))
        corrections = int(target_config.get("initial_corrections", 2))
        observations = []

        def observe(candidate_phase: int, candidate_amplitude: int):
            self._send_target(target, candidate_amplitude, candidate_phase,
                              tuning_word)
            self._settle()
            analysis = self.target_analyzer.analyze(self._capture_mask(), target)
            observations.append((analysis, candidate_phase, candidate_amplitude))
            return analysis

        initial = observe(phase, amplitude)
        corrected_amplitude = amplitude
        if initial.span_y_div > 0.5:
            corrected_amplitude = int(round(amplitude * 8.0 / initial.span_y_div))
            corrected_amplitude = max(amplitude_min, min(255, corrected_amplitude))
        delta = self._phase_delta(initial.estimated_phase, target)
        candidates = []
        if abs(delta) > 2:
            candidates = [(phase + delta) & 0xFF, (phase - delta) & 0xFF]
        elif corrected_amplitude != amplitude:
            candidates = [phase]
        for candidate_phase in candidates[:max(0, corrections)]:
            observe(candidate_phase, corrected_amplitude)
        best, best_phase, best_amplitude = min(
            observations,
            key=lambda item: item[0].desired_score +
            0.004 * abs(item[0].span_y_div - 8.0))
        if (best_phase != observations[-1][1] or
                best_amplitude != observations[-1][2]):
            self._send_target(target, best_amplitude, best_phase, tuning_word)
            self._settle()
        return best, best_phase, best_amplitude

    def run_target(self, target: int) -> None:
        """Legacy entry point: start the non-blocking controller."""
        self.start(target)

    # ==================== 圆形扫描方法 ====================

    def _begin_circle_scan(self, now: float) -> None:
        """开始圆形频率扫描"""
        scan_cfg = self.config.get("frequency_scan", {})

        # 规划扫描点
        uncertainty = float(self.config.get("fine_phase", {}).get(
            "coarse_uncertainty_hz", 450.0))
        self._scan_points = self.frequency_scanner.plan_scan_points(
            self._final_frequency_hz, uncertainty
        )
        self._scan_index = 0
        self._scan_results = []
        self._best_circle_freq = 0.0
        self._best_circle_quality = 0
        self._circle_scan_attempts = 0

        LOGGER.info(
            f"Circle scan: planned {len(self._scan_points)} points "
            f"around {self._final_frequency_hz:.1f} Hz"
        )

        if not self._scan_points:
            self._fail(ERROR_PHASE_UNSTABLE, "no scan points generated")
            return

        # 开始第一个扫描点
        self._start_circle_scan_point(now)

    def _start_circle_scan_point(self, now: float) -> None:
        """启动一个扫描点的测试"""
        if self._scan_index >= len(self._scan_points):
            # 所有点都扫描完毕
            self._finish_circle_scan(now)
            return

        scan_freq = self._scan_points[self._scan_index]

        # 计算DDS调谐字
        tuning_word = self.frequency_scanner.frequency_to_tuning_word(scan_freq)

        if not 1 <= tuning_word <= 0xFFFFFFFF:
            LOGGER.warning(f"Invalid tuning word for {scan_freq:.1f} Hz, skipping")
            self._scan_index += 1
            self._start_circle_scan_point(now)
            return

        self._tuning_word = tuning_word

        # 设置初始幅度和相位
        target_config = self.config.get("target", {})
        amplitude_map = target_config.get("initial_amplitude", {})
        self._amplitude = int(amplitude_map.get(str(self._target), 255))

        # 圆形需要90度相位差，初始设置为64 (90度)
        self._phase = 64

        LOGGER.info(
            f"Circle scan point {self._scan_index + 1}/{len(self._scan_points)}: "
            f"testing {scan_freq:.1f} Hz (TW={tuning_word})"
        )

        # 发送目标命令
        frame = self._send_target(
            self._target, self._amplitude, self._phase, self._tuning_word
        )

        self._send_progress(
            STATE_TRACK, self._scan_index,
            self._best_circle_quality,
            self._coarse_points, scan_freq
        )

        if self._start_ack_wait(frame, f"circle scan {scan_freq:.1f}Hz", now):
            self._mode = "CIRCLE_SCAN_WAIT_ACK"

    def _capture_circle_scan_frame(self, now: float) -> None:
        """捕获并评估当前扫描点的圆形质量"""
        runtime = self.config.get("runtime", {})
        required = int(runtime.get("aggregate_frames", 3))

        try:
            frame = self.camera.read()
            mask = self.extractor.extract(frame)
            self._track_masks.append(mask)

            if self._preview:
                cv2.imshow("scope", frame)
                cv2.imshow("trace", mask)
                cv2.waitKey(1)

            if len(self._track_masks) < required:
                return

            # 聚合掩膜并检测圆形
            aggregated_mask = aggregate_masks(self._track_masks)
            circle_quality = self.circle_detector.detect_circle(aggregated_mask)

            current_freq = self._scan_points[self._scan_index]

            LOGGER.info(
                f"Circle scan {current_freq:.1f} Hz: "
                f"is_circle={circle_quality.is_circle}, "
                f"quality={circle_quality.quality_score}, "
                f"circularity={circle_quality.circularity:.3f}, "
                f"axis_ratio={circle_quality.radius_ratio:.3f}"
            )

            # 记录结果
            self._scan_results.append((
                current_freq,
                circle_quality.quality_score,
                circle_quality.is_circle
            ))

            # 更新最佳结果
            if circle_quality.is_circle and circle_quality.quality_score > self._best_circle_quality:
                self._best_circle_freq = current_freq
                self._best_circle_quality = circle_quality.quality_score
                LOGGER.info(f"New best circle: {current_freq:.1f} Hz, quality={circle_quality.quality_score}")

            # 检查是否已找到足够好的圆形
            scan_cfg = self.config.get("frequency_scan", {})
            early_stop_quality = int(scan_cfg.get("early_stop_quality", 90))

            if circle_quality.is_circle and circle_quality.quality_score >= early_stop_quality:
                LOGGER.info(f"Found excellent circle at {current_freq:.1f} Hz, stopping scan")
                self._finish_circle_scan(now)
                return

            # 继续下一个扫描点
            self._track_masks = []
            self._scan_index += 1
            self._start_circle_scan_point(now)

        except Exception as exc:
            LOGGER.warning(f"Circle scan frame failed: {exc}")
            self._track_masks = []
            self._scan_index += 1
            self._start_circle_scan_point(now)

    def _finish_circle_scan(self, now: float) -> None:
        """完成圆形扫描，选择最佳频率"""
        if not self._scan_results:
            self._fail(ERROR_CAMERA, "circle scan produced no results")
            return

        # 选择最佳频率
        scan_result = self.frequency_scanner.select_best_frequency(self._scan_results)

        LOGGER.info(
            f"Circle scan complete: scanned {scan_result.scan_count} points, "
            f"best freq={scan_result.frequency_hz:.1f} Hz, "
            f"quality={scan_result.quality_score}, "
            f"found_circle={scan_result.found_circle}"
        )

        if not scan_result.found_circle:
            # 检查是否需要扩展扫描
            if self.frequency_scanner.should_extend_scan(
                self._scan_results, len(self._scan_points)
            ):
                LOGGER.info("Extending circle scan range")
                extended_points = self.frequency_scanner.generate_extended_points(
                    self._final_frequency_hz, self._scan_points
                )
                self._scan_points.extend(extended_points)
                self._start_circle_scan_point(now)
                return
            else:
                self._fail(ERROR_CAMERA, "no valid circle found in scan range")
                return

        # 使用最佳频率
        self._final_frequency_hz = scan_result.frequency_hz
        self._tuning_word = scan_result.tuning_word

        # 设置最佳相位（圆形通常是90度）
        self._phase = 64

        # 进入跟踪模式以微调
        frame = self._send_target(
            self._target, self._amplitude, self._phase, self._tuning_word
        )

        self._send_progress(
            STATE_TRACK, 0, scan_result.quality_score,
            self._coarse_points, self._final_frequency_hz
        )

        if self._start_ack_wait(frame, "best circle frequency", now):
            self._mode = "TRACK_WAIT_ACK"



# ========== 主程序 ==========




    CMD_ACK,
    CMD_NACK,
    EVENT_CANCEL,
    EVENT_START,
    RESULT_ACCEPTED,
    RESULT_BAD_ARGUMENT,
    RESULT_BUSY,
    RESULT_DUPLICATE,
    RESULT_UNSUPPORTED,
    STATUS_HEARTBEAT,
    Frame,
    SerialLink,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Task5 Lissajous vision controller")
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--port", help="override STM32 serial port")
    parser.add_argument("--source", help="camera index or offline video path")
    parser.add_argument("--preview", action="store_true")
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    config_path = Path(args.config)
    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if args.preview:
        config.setdefault("runtime", {})["preview"] = True

    serial_config = config.get("serial", {})
    port = args.port or serial_config.get("port", "/dev/serial0")
    source = args.source
    if source is not None and source.isdigit():
        source = int(source)

    link = SerialLink(port, int(serial_config.get("baudrate", 115200)))
    camera = ScopeCamera(config, source)
    controller = AutoLissajousController(config, link, camera)
    last_heartbeat = 0.0
    # A retry repeats the original sequence.  Cache the outcome so an EVENT
    # retransmission gets the same reply without restarting the camera flow.
    event_replies: dict[tuple[int, int], tuple[bool, int]] = {}
    event_reply_order: list[tuple[int, int]] = []

    def reply_to_event(frame: Frame, accepted: bool, result: int) -> None:
        if not frame.requests_ack:
            return
        link.reply(frame, accepted=accepted, result=result)

    def remember_event(frame: Frame, accepted: bool, result: int) -> None:
        key = (frame.sequence, frame.command)
        event_replies[key] = (accepted, result)
        event_reply_order.append(key)
        if len(event_reply_order) > 32:
            oldest = event_reply_order.pop(0)
            event_replies.pop(oldest, None)

    logging.info("ready: serial=%s source=%s", port,
                 source if source is not None else config.get("camera", {}).get("device", 0))
    try:
        while True:
            now = time.monotonic()
            if now - last_heartbeat >= 0.5:
                link.send(STATUS_HEARTBEAT, bytes(8))
                last_heartbeat = now
            for frame in link.poll():
                if frame.command in (CMD_ACK, CMD_NACK):
                    controller.handle_frame(frame, now)
                    continue

                if frame.command not in (EVENT_START, EVENT_CANCEL):
                    reply_to_event(frame, False, RESULT_UNSUPPORTED)
                    continue

                key = (frame.sequence, frame.command)
                cached = event_replies.get(key)
                if cached is not None:
                    reply_to_event(frame, cached[0],
                                   RESULT_DUPLICATE if cached[0] else cached[1])
                    continue

                if frame.command == EVENT_START:
                    target = frame.payload[0]
                    if not 1 <= target <= 3:
                        accepted, result = False, RESULT_BAD_ARGUMENT
                    elif controller.active:
                        accepted, result = False, RESULT_BUSY
                    elif controller.start(target, now):
                        accepted, result = True, RESULT_ACCEPTED
                    else:
                        accepted, result = False, RESULT_BUSY
                    remember_event(frame, accepted, result)
                    reply_to_event(frame, accepted, result)
                elif frame.command == EVENT_CANCEL:
                    controller.cancel(frame.payload[0])
                    remember_event(frame, True, RESULT_ACCEPTED)
                    reply_to_event(frame, True, RESULT_ACCEPTED)
            controller.poll(now)
            time.sleep(0.005)
    except KeyboardInterrupt:
        return 0
    finally:
        camera.close()
        link.close()


if __name__ == "__main__":
    raise SystemExit(main())
