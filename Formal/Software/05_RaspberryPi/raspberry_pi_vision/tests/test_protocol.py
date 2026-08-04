import sys
from types import SimpleNamespace

from protocol import (
    CMD_ACK,
    CMD_PROBE_SINGLE,
    CMD_TARGET,
    FLAG_ACK,
    FLAG_ACK_REQUEST,
    FLAG_RETRY,
    FRAME_SIZE,
    Frame,
    FrameParser,
    PROTOCOL_MARKER,
    RESULT_ACCEPTED,
    SequenceTracker,
    STATUS_HEARTBEAT,
    SerialLink,
    crc16_ccitt_false,
    progress_payload,
)


def test_crc16_ccitt_false_reference_vector() -> None:
    assert crc16_ccitt_false(b"123456789") == 0x29B1


def test_frame_round_trip_with_fragmentation() -> None:
    frame = Frame(17, CMD_TARGET, bytes(range(8)), FLAG_ACK_REQUEST)
    encoded = frame.encode()
    assert len(encoded) == FRAME_SIZE == 16
    assert encoded[:3] == bytes((0xA5, 0x5A, PROTOCOL_MARKER))
    assert encoded[3] == FLAG_ACK_REQUEST
    assert int.from_bytes(encoded[-2:], "little") == crc16_ccitt_false(encoded[:14])

    parser = FrameParser()
    assert parser.feed(b"noise" + encoded[:7]) == []
    decoded = parser.feed(encoded[7:])
    assert decoded == [frame]


def test_probe_wire_golden_vector() -> None:
    frame = Frame(1, CMD_PROBE_SINGLE, bytes(8), FLAG_ACK_REQUEST)
    assert frame.encode().hex(" ").upper() == (
        "A5 5A 51 01 01 10 00 00 00 00 00 00 00 00 70 25"
    )


def test_parser_recovers_after_bad_crc_and_bad_marker() -> None:
    good = Frame(3, CMD_PROBE_SINGLE, bytes(8)).encode()
    bad_crc = bytearray(good)
    bad_crc[7] ^= 0x20
    bad_marker = bytearray(good)
    bad_marker[2] = 0x99
    parser = FrameParser()
    assert parser.feed(bytes(bad_crc) + bytes(bad_marker) + good) == [
        Frame(3, CMD_PROBE_SINGLE, bytes(8))
    ]


def test_sequence_tracker_gates_duplicates_on_retry_flag() -> None:
    tracker = SequenceTracker()
    frame = Frame(255, CMD_PROBE_SINGLE, bytes((3, 0, 0, 0, 0, 0, 0, 0)))
    assert tracker.accept(frame)
    assert not tracker.accept(Frame(255, CMD_PROBE_SINGLE, frame.payload, FLAG_RETRY))
    assert tracker.accept(Frame(255, CMD_TARGET, bytes(8)))
    assert not tracker.accept(Frame(255, CMD_PROBE_SINGLE, frame.payload, FLAG_RETRY))
    assert tracker.accept(Frame(0, CMD_PROBE_SINGLE, frame.payload))


def test_ack_payload_identifies_original_request() -> None:
    request = Frame(42, CMD_PROBE_SINGLE, bytes((1, 0, 0, 0, 0, 0, 0, 0)),
                    FLAG_ACK_REQUEST)
    response = Frame(5, CMD_ACK,
                     bytes((request.sequence, request.command, RESULT_ACCEPTED,
                            2, 1, 0, 0, 0)), FLAG_ACK)
    parsed = FrameParser().feed(response.encode())
    assert parsed == [response]
    assert parsed[0].payload[:3] == bytes((42, CMD_PROBE_SINGLE, 0))


def test_progress_payload_uses_millihz_layout() -> None:
    payload = progress_payload(2, 3, 87, 12, 100_000_032)
    assert payload[:4] == bytes((2, 3, 87, 12))
    assert int.from_bytes(payload[4:8], "little") == 100_000_032


def test_serial_link_resends_same_sequence_with_retry_flag(monkeypatch) -> None:
    ports = []

    class FakeSerial:
        def __init__(self, port, baudrate, timeout):
            self.is_open = True
            self.writes = []
            ports.append(self)

        @property
        def in_waiting(self):
            return 0

        def read(self, size):
            return b""

        def write(self, data):
            self.writes.append(data)

        def close(self):
            self.is_open = False

    fake_module = SimpleNamespace(Serial=FakeSerial, SerialException=OSError)
    monkeypatch.setitem(sys.modules, "serial", fake_module)

    link = SerialLink("/dev/test", reconnect_interval_s=0.0)
    request = link.send_frame(CMD_PROBE_SINGLE, bytes((1, 0, 0, 0, 0, 0, 0, 0)),
                              flags=FLAG_ACK_REQUEST)
    assert request is not None
    retry = link.resend(request)
    assert retry is not None
    first, second = [FrameParser().feed(raw)[0] for raw in ports[0].writes]
    assert first.sequence == second.sequence == request.sequence
    assert first.payload == second.payload == request.payload
    assert first.flags == FLAG_ACK_REQUEST
    assert second.flags == FLAG_ACK_REQUEST | FLAG_RETRY
    link.close()


def test_serial_link_recovers_after_runtime_eio(monkeypatch) -> None:
    ports = []

    class FakeSerial:
        def __init__(self, port, baudrate, timeout):
            self.is_open = True
            self.fail_read = len(ports) == 0
            self.writes = []
            ports.append(self)

        @property
        def in_waiting(self):
            if self.fail_read:
                raise OSError(5, "Input/output error")
            return 0

        def read(self, size):
            return b""

        def write(self, data):
            self.writes.append(data)

        def close(self):
            self.is_open = False

    fake_module = SimpleNamespace(Serial=FakeSerial, SerialException=OSError)
    monkeypatch.setitem(sys.modules, "serial", fake_module)

    link = SerialLink("/dev/test", reconnect_interval_s=0.0)
    assert link.poll() == []
    assert link.send(STATUS_HEARTBEAT)
    assert len(ports) == 2
    assert FrameParser().feed(ports[1].writes[0])[0].command == STATUS_HEARTBEAT
    link.close()
