# -*- coding: utf-8 -*-
"""Task5 Raspberry Pi OpenCV Controller with Auto Lock Circle
完整单文件版本，包含自动锁圆功能

This file integrates:
- Protocol handling (UART communication with STM32)
- Camera interface (with thread-safe frame buffering)
- Fixed-camera OpenCV processing (point extraction)
- Vision algorithms (frequency estimation, phase analysis)
- Circle detection and quality assessment
- Frequency scanning with 100Hz steps
- Main controller with auto lock circle state machine

Version: 1.0-AutoLockCircle
Build Date: 2026-08-01
Compatible with: FPGA Task5 v2 protocol, STM32 Apollo LCD
"""

from __future__ import annotations

import argparse
import logging
import math
import threading
import time
from collections import deque
from dataclasses import dataclass, replace
from math import ceil, floor, pi
from pathlib import Path
from typing import Any, Iterable

import cv2
import numpy as np

# Build identification
TASK5_CV_BUILD = "2026-08-01-auto-lock-circle-v1.0"
TASK5_CV_BUILD_TAG = "AUTO-CIRCLE-V1"

LOGGER = logging.getLogger(__name__)

# ============================================================================
# Protocol Constants and Types
# ============================================================================

SYNC = bytes((0xA5, 0x5A))
PROTOCOL_MARKER = 0x51
FRAME_SIZE = 16

# Frame flags
FLAG_ACK_REQUEST = 0x01
FLAG_RETRY = 0x02
FLAG_NACK = 0x40
FLAG_ACK = 0x80

# Commands (Pi -> STM32 -> FPGA)
CMD_DISABLE = 0x00
CMD_STANDBY = 0x01
CMD_PROBE_SINGLE = 0x10
CMD_PROBE_DUAL = 0x11
CMD_TARGET = 0x20

# Events (STM32 -> Pi)
EVENT_START = 0x40
EVENT_CANCEL = 0x41

# Status (Pi -> STM32)
STATUS_PROGRESS = 0x80
STATUS_LOCKED = 0x81
STATUS_ERROR = 0x82
STATUS_HEARTBEAT = 0x83

# Responses
CMD_ACK = 0x70
CMD_NACK = 0x71

# State codes
STATE_IDLE = 0
STATE_WAIT_PI = 1
STATE_COARSE = 2
STATE_FINE_PHASE = 3
STATE_TRACK = 4
STATE_LOCKED = 5
STATE_ERROR = 6

# Target types
TARGET_DIAGONAL = 1
TARGET_CIRCLE = 2
TARGET_EIGHT = 3

# Error codes
ERROR_VISUAL_RANGE = 1
ERROR_COARSE_FAILED = 2
ERROR_PHASE_UNSTABLE = 3
ERROR_TIMEOUT = 4
ERROR_CANCELLED = 5
ERROR_CAMERA = 6

# Result codes
RESULT_ACCEPTED = 0
RESULT_DUPLICATE = 1
RESULT_BAD_ARGUMENT = 2
RESULT_UNSUPPORTED = 3
RESULT_BUSY = 4


def crc16_ccitt_false(data: bytes | bytearray | Iterable[int]) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)."""
    crc = 0xFFFF
    for value in data:
        crc ^= (int(value) & 0xFF) << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass(frozen=True)
class Frame:
    """Task5 v2 protocol frame."""
    sequence: int
    command: int
    payload: bytes
    flags: int = 0

    def __post_init__(self) -> None:
        if len(self.payload) != 8:
            raise ValueError("payload must be exactly 8 bytes")
        if not 0 <= self.sequence <= 0xFF:
            raise ValueError("sequence out of range")
        if not 0 <= self.command <= 0xFF:
            raise ValueError("command out of range")
        if not 0 <= self.flags <= 0xFF:
            raise ValueError("flags out of range")

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
    """Parse incoming Task5 v2 frames from byte stream."""

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


class SerialLink:
    """UART link to STM32 with auto-reconnect."""

    def __init__(self, port: str, baudrate: int = 115200,
                 reconnect_interval_s: float = 1.0) -> None:
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("pyserial required") from exc
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
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None

    def _warn(self, message: str) -> None:
        now = time.monotonic()
        if message != self._last_error or now - self._last_warning_at >= 5.0:
            LOGGER.warning("serial %s: %s", self._port, message)
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
        except Exception as exc:
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
        except Exception as exc:
            self._disconnect(exc)
            return []
        return self._parser.feed(data)

    def _write_frame(self, frame: Frame) -> bool:
        if not self._open_if_due():
            return False
        try:
            self._serial.write(frame.encode())
            return True
        except Exception as exc:
            self._disconnect(exc)
            return False

    def send_frame(self, command: int, payload: bytes = bytes(8), *,
                   flags: int = 0) -> Frame | None:
        frame = Frame(self._sequence, command, payload, flags)
        if not self._write_frame(frame):
            return None
        self._sequence = (self._sequence + 1) & 0xFF
        return frame

    def resend(self, frame: Frame) -> Frame | None:
        retry = Frame(frame.sequence, frame.command, frame.payload,
                      frame.flags | FLAG_RETRY)
        return retry if self._write_frame(retry) else None

    def send(self, command: int, payload: bytes = bytes(8)) -> bool:
        return self.send_frame(command, payload) is not None

    def reply(self, request: Frame, *, accepted: bool, result: int = RESULT_ACCEPTED) -> bool:
        payload = bytes((request.sequence & 0xFF, request.command & 0xFF,
                         result & 0xFF, 0, 0, 0, 0, 0))
        return self.send(CMD_ACK if accepted else CMD_NACK, payload)


def u32le(value: int) -> bytes:
    """Encode uint32 as little-endian bytes."""
    return int(value).to_bytes(4, "little", signed=False)


def progress_payload(state: int, stage: int, quality: int,
                    point_count: int, frequency_millihz: int) -> bytes:
    return bytes((state & 0xFF, stage & 0xFF,
                  max(0, min(100, quality)),
                  max(0, min(255, point_count)))) + u32le(frequency_millihz)


def locked_payload(target: int, quality: int, coarse_width_code: int,
                   frequency_millihz: int) -> bytes:
    return bytes((target & 0xFF, max(0, min(100, quality)),
                  coarse_width_code & 0xFF, 0)) + u32le(frequency_millihz)


# ============================================================================
# Camera Interface
# ============================================================================

class ScopeCamera:
    """Camera with thread-safe latest-frame buffering."""

    def __init__(self, config: dict[str, Any], source: str | int | None = None) -> None:
        camera_config = config.get("camera", {})
        selected = camera_config.get("device", 0) if source is None else source
        if isinstance(selected, str) and selected.isdigit():
            selected = int(selected)
        self._is_file = isinstance(selected, str) and Path(selected).exists()
        self._capture = cv2.VideoCapture(selected)
        if not self._capture.isOpened():
            raise RuntimeError(f"cannot open camera: {selected}")

        if not self._is_file:
            self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, camera_config.get("width", 1280))
            self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_config.get("height", 720))
            self._capture.set(cv2.CAP_PROP_FPS, camera_config.get("fps", 30))
            self._capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self._capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, camera_config.get("auto_exposure", 1))

        self._config = camera_config
        output = config.get("vision", {}).get("canonical_size", [640, 480])
        self._output_size = (int(output[0]), int(output[1]))

        # Thread-safe frame buffer
        self._frame_lock = threading.Lock()
        self._latest_frame: np.ndarray | None = None
        self._frame_sequence = 0
        self._stop_reader = threading.Event()
        self._reader_thread: threading.Thread | None = None

        if not self._is_file:
            self._reader_thread = threading.Thread(
                target=self._read_live_frames, daemon=True)
            self._reader_thread.start()

    def _read_live_frames(self) -> None:
        while not self._stop_reader.is_set():
            ok, frame = self._capture.read()
            if ok and frame is not None:
                with self._frame_lock:
                    self._latest_frame = frame
                    self._frame_sequence += 1
            else:
                time.sleep(0.02)

    def close(self) -> None:
        self._stop_reader.set()
        if self._reader_thread:
            self._reader_thread.join(timeout=1.0)
        self._capture.release()

    def read_raw(self) -> np.ndarray:
        if self._is_file:
            ok, frame = self._capture.read()
            if not ok:
                self._capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ok, frame = self._capture.read()
            if not ok or frame is None:
                raise RuntimeError("camera returned no frame")
            return frame

        with self._frame_lock:
            if self._latest_frame is None:
                raise RuntimeError("no frame available")
            return self._latest_frame.copy()

    def read(self) -> np.ndarray:
        return self._rectify(self.read_raw())

    def _rectify(self, frame: np.ndarray) -> np.ndarray:
        points = self._config.get("perspective_points")
        if points and len(points) == 4:
            source = np.asarray(points, dtype=np.float32)
            width, height = self._output_size
            destination = np.asarray(
                [[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]],
                dtype=np.float32)
            transform = cv2.getPerspectiveTransform(source, destination)
            return cv2.warpPerspective(frame, transform, self._output_size)

        roi = self._config.get("roi", [0.0, 0.0, 1.0, 1.0])
        height, width = frame.shape[:2]
        x, y, roi_width, roi_height = [float(v) for v in roi]
        if max(x, y, roi_width, roi_height) <= 1.0:
            x, roi_width = x * width, roi_width * width
            y, roi_height = y * height, roi_height * height
        x0 = max(0, min(width - 1, int(round(x))))
        y0 = max(0, min(height - 1, int(round(y))))
        x1 = max(x0 + 1, min(width, int(round(x + roi_width))))
        y1 = max(y0 + 1, min(height, int(round(y + roi_height))))
        return cv2.resize(frame[y0:y1, x0:x1], self._output_size)


# PLACEHOLDER_FOR_PART2
