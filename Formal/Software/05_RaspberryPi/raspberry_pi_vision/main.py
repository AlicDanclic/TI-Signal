from __future__ import annotations

import argparse
import logging
import time
from pathlib import Path

import yaml

from camera import ScopeCamera
from controller import AutoLissajousController
from protocol import (
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
