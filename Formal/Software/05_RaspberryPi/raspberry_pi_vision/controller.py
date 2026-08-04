from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np

from camera import ScopeCamera
from circle_detector import CircleDetector
from frequency_scanner import FrequencyScanner
from opencv_main import DEFAULT_SCREEN_SIZE, process_frame
from protocol import (
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
from vision import (
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

LOGGER = logging.getLogger(__name__)


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
