from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Iterable


LOGGER = logging.getLogger(__name__)

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
            import serial
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
