from __future__ import annotations

import json
import socket
import threading
import time
from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np

from .sample_buffer import LatestValueMailbox


DEFAULT_CAD_JOINT_NAMES = (
    "link1_link",
    "link2_link",
    "link3_link",
    "link4_link",
    "link5_link",
)


class CadInputError(ValueError):
    pass


class CadSafetyError(RuntimeError):
    pass


def _finite_vector(value: Any, size: int, name: str) -> np.ndarray:
    try:
        vector = np.asarray(value, dtype=float).reshape(size)
    except (TypeError, ValueError) as exc:
        raise CadInputError(f"{name} must contain exactly {size} numeric values") from exc
    if not np.all(np.isfinite(vector)):
        raise CadInputError(f"{name} must contain only finite values")
    return vector


def wrapped_angle_delta(current: np.ndarray, anchor: np.ndarray) -> np.ndarray:
    """Return the shortest signed angular displacement for each joint."""

    delta = np.asarray(current, dtype=float) - np.asarray(anchor, dtype=float)
    return np.arctan2(np.sin(delta), np.cos(delta))


@dataclass(frozen=True)
class CadJointSample:
    sequence: int
    source_time_ns: int
    arrival_monotonic_ns: int
    arrival_epoch_ns: int
    q: np.ndarray

    def __post_init__(self) -> None:
        if isinstance(self.sequence, bool) or int(self.sequence) < 0:
            raise CadInputError("CAD sequence must be a nonnegative integer")
        if isinstance(self.source_time_ns, bool) or int(self.source_time_ns) <= 0:
            raise CadInputError("CAD source timestamp must be a positive integer")
        if int(self.arrival_monotonic_ns) <= 0 or int(self.arrival_epoch_ns) <= 0:
            raise CadInputError("CAD arrival timestamps must be positive")
        object.__setattr__(self, "sequence", int(self.sequence))
        object.__setattr__(self, "source_time_ns", int(self.source_time_ns))
        object.__setattr__(self, "arrival_monotonic_ns", int(self.arrival_monotonic_ns))
        object.__setattr__(self, "arrival_epoch_ns", int(self.arrival_epoch_ns))
        object.__setattr__(self, "q", _finite_vector(self.q, 5, "CAD q").copy())


def parse_cad_joint_packet(
    raw: str | bytes | dict[str, Any],
    *,
    expected_names: Sequence[str] = DEFAULT_CAD_JOINT_NAMES,
    arrival_monotonic_ns: int | None = None,
    arrival_epoch_ns: int | None = None,
    max_packet_age_s: float = 0.100,
    max_future_skew_s: float = 0.050,
) -> CadJointSample:
    """Parse one M3T joint packet and reject incomplete or delayed state."""

    try:
        if isinstance(raw, bytes):
            payload = json.loads(raw.decode("utf-8"))
        elif isinstance(raw, str):
            payload = json.loads(raw)
        else:
            payload = raw
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise CadInputError("CAD packet is not valid UTF-8 JSON") from exc
    if not isinstance(payload, dict):
        raise CadInputError("CAD packet must be a JSON object")

    expected = tuple(str(name) for name in expected_names)
    names_raw = payload.get("names")
    if not isinstance(names_raw, list) or not all(isinstance(name, str) for name in names_raw):
        raise CadInputError("CAD packet names must be a list of strings")
    names = tuple(names_raw)
    if len(names) != len(set(names)):
        raise CadInputError("CAD packet contains duplicate joint names")
    if len(names) != len(expected) or set(names) != set(expected):
        missing = sorted(set(expected) - set(names))
        unexpected = sorted(set(names) - set(expected))
        raise CadInputError(
            f"CAD packet joint set mismatch: missing={missing}, unexpected={unexpected}"
        )

    values = _finite_vector(payload.get("q"), len(expected), "CAD q")
    by_name = dict(zip(names, values, strict=True))
    ordered = np.array([by_name[name] for name in expected], dtype=float)

    sequence = payload.get("seq")
    source_time = payload.get("time_ns")
    if isinstance(sequence, bool) or not isinstance(sequence, (int, np.integer)):
        raise CadInputError("CAD sequence must be an integer")
    if isinstance(source_time, bool) or not isinstance(source_time, (int, np.integer)):
        raise CadInputError("CAD source timestamp must be an integer")

    monotonic_ns = time.perf_counter_ns() if arrival_monotonic_ns is None else int(arrival_monotonic_ns)
    epoch_ns = time.time_ns() if arrival_epoch_ns is None else int(arrival_epoch_ns)
    max_age_ns = int(float(max_packet_age_s) * 1e9)
    max_future_ns = int(float(max_future_skew_s) * 1e9)
    if max_age_ns <= 0 or max_future_ns < 0:
        raise ValueError("CAD packet age limits must be positive/nonnegative")
    source_age_ns = epoch_ns - int(source_time)
    if source_age_ns > max_age_ns:
        raise CadInputError(
            f"CAD packet is stale at receipt ({source_age_ns / 1e6:.1f} ms)"
        )
    if source_age_ns < -max_future_ns:
        raise CadInputError(
            f"CAD packet timestamp is {-source_age_ns / 1e6:.1f} ms in the future"
        )

    return CadJointSample(
        sequence=int(sequence),
        source_time_ns=int(source_time),
        arrival_monotonic_ns=monotonic_ns,
        arrival_epoch_ns=epoch_ns,
        q=ordered,
    )


@dataclass(frozen=True)
class CadFreshnessStatus:
    fresh: bool
    age_s: float
    recovery_streak: int
    unique_samples: int
    repeated_samples: int
    discontinuity_generation: int
    discontinuities: int
    last_discontinuity: str


class CadFreshnessWatchdog:
    """Require continuous, plausible M3T packets before enabling motion."""

    def __init__(
        self,
        timeout_s: float,
        fresh_samples_to_recover: int,
        max_source_step_rad: np.ndarray,
    ) -> None:
        timeout = float(timeout_s)
        if not np.isfinite(timeout) or timeout <= 0.0:
            raise ValueError("CAD stale timeout must be finite and positive")
        self.timeout_ns = int(timeout * 1e9)
        self.fresh_samples_to_recover = int(fresh_samples_to_recover)
        if self.fresh_samples_to_recover < 1:
            raise ValueError("CAD recovery count must be at least one")
        self.max_source_step_rad = _finite_vector(
            max_source_step_rad, 5, "CAD source step limits"
        )
        if np.any(self.max_source_step_rad <= 0.0):
            raise ValueError("CAD source step limits must be positive")

        self.last_sequence: int | None = None
        self.last_q: np.ndarray | None = None
        self.last_unique_arrival_ns: int | None = None
        self.unique_samples = 0
        self.repeated_samples = 0
        self.discontinuity_generation = 0
        self.discontinuities = 0
        self.last_discontinuity = ""
        self._recovery_streak = 0
        self._fresh = False
        self._timed_out = False

    def _mark_discontinuity(self, reason: str) -> None:
        self._fresh = False
        self._recovery_streak = 0
        self.discontinuity_generation += 1
        self.discontinuities += 1
        self.last_discontinuity = reason

    def observe(
        self,
        sample: CadJointSample,
        now_ns: int | None = None,
    ) -> CadFreshnessStatus:
        if self.last_sequence is not None and sample.sequence == self.last_sequence:
            self.repeated_samples += 1
            return self.poll(now_ns)

        restarted = self.last_sequence is not None and sample.sequence < self.last_sequence
        jump_joint: int | None = None
        if not restarted and self.last_q is not None:
            step = np.abs(wrapped_angle_delta(sample.q, self.last_q))
            outside = np.flatnonzero(step > self.max_source_step_rad + 1e-12)
            if outside.size:
                jump_joint = int(outside[0])

        if restarted:
            self._mark_discontinuity("sequence_restart")
        elif jump_joint is not None:
            self._mark_discontinuity(f"source_jump_joint_{jump_joint}")

        self.last_sequence = sample.sequence
        self.last_q = sample.q.copy()
        self.last_unique_arrival_ns = sample.arrival_monotonic_ns
        self.unique_samples += 1
        self._timed_out = False
        if not restarted and jump_joint is None:
            self._recovery_streak += 1
            if self._recovery_streak >= self.fresh_samples_to_recover:
                self._fresh = True
        return self.poll(now_ns)

    def poll(self, now_ns: int | None = None) -> CadFreshnessStatus:
        now = time.perf_counter_ns() if now_ns is None else int(now_ns)
        age_ns = (
            self.timeout_ns + 1
            if self.last_unique_arrival_ns is None
            else now - self.last_unique_arrival_ns
        )
        if age_ns > self.timeout_ns:
            if not self._timed_out and self.last_unique_arrival_ns is not None:
                self._mark_discontinuity("timeout")
            self._timed_out = True
            self._fresh = False
            self._recovery_streak = 0
        return CadFreshnessStatus(
            fresh=self._fresh,
            age_s=max(0.0, age_ns / 1e9),
            recovery_streak=self._recovery_streak,
            unique_samples=self.unique_samples,
            repeated_samples=self.repeated_samples,
            discontinuity_generation=self.discontinuity_generation,
            discontinuities=self.discontinuities,
            last_discontinuity=self.last_discontinuity,
        )


def rest_command_limits(
    model_joint_limits: np.ndarray,
    rest_q: np.ndarray,
    minimum_delta_rad: np.ndarray,
    maximum_delta_rad: np.ndarray,
) -> np.ndarray:
    """Intersect the commissioning travel envelope with official model limits."""

    model_limits = np.asarray(model_joint_limits, dtype=float).reshape(6, 2)
    rest = _finite_vector(rest_q, 6, "rest position")
    minimum = _finite_vector(minimum_delta_rad, 6, "minimum rest-relative travel")
    maximum = _finite_vector(maximum_delta_rad, 6, "maximum rest-relative travel")
    if np.any(minimum > 0.0) or np.any(maximum < 0.0) or np.any(minimum >= maximum):
        raise CadSafetyError("rest-relative travel must straddle zero with ordered bounds")
    lower = np.maximum(model_limits[:, 0], rest + minimum)
    upper = np.minimum(model_limits[:, 1], rest + maximum)
    if np.any(lower >= upper):
        raise CadSafetyError("commissioning envelope leaves an empty joint range")
    if np.any(rest < lower) or np.any(rest > upper):
        raise CadSafetyError("rest position is outside the commissioning envelope")
    return np.column_stack([lower, upper])


class CadRestMapper:
    """Map five tracked leader joints relative to a rest-anchored robot pose."""

    def __init__(
        self,
        *,
        rest_q: np.ndarray,
        signs: np.ndarray,
        scales: np.ndarray,
        source_deadband_rad: np.ndarray | None = None,
        command_limits: np.ndarray,
    ) -> None:
        self.rest_q = _finite_vector(rest_q, 6, "rest position")
        self.signs = _finite_vector(signs, 5, "CAD joint signs")
        self.scales = _finite_vector(scales, 5, "CAD joint scales")
        if np.any(self.signs == 0.0) or np.any(self.scales <= 0.0):
            raise CadSafetyError("CAD signs must be nonzero and scales must be positive")
        self.source_deadband_rad = (
            np.zeros(5, dtype=float)
            if source_deadband_rad is None
            else _finite_vector(source_deadband_rad, 5, "CAD source deadband")
        )
        if np.any(self.source_deadband_rad < 0.0):
            raise CadSafetyError("CAD source deadband must be nonnegative")
        self.command_limits = np.asarray(command_limits, dtype=float).reshape(6, 2)
        if not np.all(np.isfinite(self.command_limits)) or np.any(
            self.command_limits[:, 0] >= self.command_limits[:, 1]
        ):
            raise CadSafetyError("CAD command limits must be finite and ordered")
        self.engaged = False
        self.reanchor_generation = 0
        self._source_anchor: np.ndarray | None = None
        self._robot_anchor: np.ndarray | None = None

    def release(self) -> None:
        self.engaged = False
        self._source_anchor = None
        self._robot_anchor = None

    def engage(self, source_q: np.ndarray, robot_q: np.ndarray) -> np.ndarray:
        source = _finite_vector(source_q, 5, "CAD source anchor")
        robot = _finite_vector(robot_q, 6, "robot anchor")
        if np.any(robot < self.command_limits[:, 0] - 1e-12) or np.any(
            robot > self.command_limits[:, 1] + 1e-12
        ):
            raise CadSafetyError("robot anchor is outside the commissioning envelope")
        self._source_anchor = source.copy()
        self._robot_anchor = robot.copy()
        self.engaged = True
        self.reanchor_generation += 1
        return robot.copy()

    def map(self, source_q: np.ndarray) -> np.ndarray:
        if not self.engaged or self._source_anchor is None or self._robot_anchor is None:
            raise CadSafetyError("CAD mapper is not anchored")
        source = _finite_vector(source_q, 5, "CAD source position")
        source_delta = wrapped_angle_delta(source, self._source_anchor)
        source_delta = np.sign(source_delta) * np.maximum(
            np.abs(source_delta) - self.source_deadband_rad,
            0.0,
        )
        target = self._robot_anchor.copy()
        target[:5] += self.signs * self.scales * source_delta
        # The leader currently publishes five joints. Keep J5 at rest rather
        # than allowing repeated re-anchors to accumulate an unobserved offset.
        target[5] = self.rest_q[5]
        outside = np.flatnonzero(
            (target < self.command_limits[:, 0] - 1e-12)
            | (target > self.command_limits[:, 1] + 1e-12)
        )
        if outside.size:
            joint = int(outside[0])
            raise CadSafetyError(
                f"CAD target for joint {joint} left the commissioning envelope"
            )
        return target


class CadDeadmanController:
    """Require a release/repress after every stale or discontinuous stream."""

    def __init__(self, mapper: CadRestMapper) -> None:
        self.mapper = mapper
        self._was_pressed = False
        self._needs_release = False

    @property
    def engaged(self) -> bool:
        return self.mapper.engaged

    @property
    def needs_release(self) -> bool:
        return self._needs_release

    def fault(self, deadman_pressed: bool) -> None:
        self.mapper.release()
        self._needs_release = self._needs_release or bool(deadman_pressed)

    def update(
        self,
        *,
        deadman_pressed: bool,
        stream_fresh: bool,
        sample: CadJointSample | None,
        robot_q: np.ndarray,
    ) -> np.ndarray | None:
        pressed = bool(deadman_pressed)
        if not stream_fresh or sample is None:
            self.fault(pressed)
            if not pressed:
                self._needs_release = False
                self._was_pressed = False
            else:
                self._was_pressed = True
            return None
        if not pressed:
            self.mapper.release()
            self._needs_release = False
            self._was_pressed = False
            return None
        if self._needs_release:
            self._was_pressed = True
            return None
        try:
            if not self._was_pressed or not self.mapper.engaged:
                self.mapper.engage(sample.q, robot_q)
            target = self.mapper.map(sample.q)
        except CadSafetyError:
            self.fault(pressed)
            raise
        self._was_pressed = True
        return target


class CadUdpReceiver:
    """Receive validated M3T packets into a capacity-one latest-state mailbox."""

    def __init__(
        self,
        host: str,
        port: int,
        *,
        expected_names: Sequence[str] = DEFAULT_CAD_JOINT_NAMES,
        max_packet_age_s: float = 0.100,
        max_future_skew_s: float = 0.050,
    ) -> None:
        self.host = str(host)
        self.port = int(port)
        self.expected_names = tuple(expected_names)
        self.max_packet_age_s = float(max_packet_age_s)
        self.max_future_skew_s = float(max_future_skew_s)
        self.mailbox: LatestValueMailbox[CadJointSample] = LatestValueMailbox()
        self.received_packets = 0
        self.valid_packets = 0
        self.invalid_packets = 0
        self.last_error: str | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._socket: socket.socket | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self.host, self.port))
        sock.settimeout(0.1)
        self._socket = sock
        self.port = int(sock.getsockname()[1])
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="cad-joint-udp", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        sock = self._socket
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._socket = None

    def _run(self) -> None:
        sock = self._socket
        if sock is None:
            return
        while not self._stop.is_set():
            try:
                payload, _address = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError as exc:
                if not self._stop.is_set():
                    self.last_error = f"{type(exc).__name__}: {exc}"
                return
            self.received_packets += 1
            arrival_monotonic_ns = time.perf_counter_ns()
            arrival_epoch_ns = time.time_ns()
            try:
                sample = parse_cad_joint_packet(
                    payload,
                    expected_names=self.expected_names,
                    arrival_monotonic_ns=arrival_monotonic_ns,
                    arrival_epoch_ns=arrival_epoch_ns,
                    max_packet_age_s=self.max_packet_age_s,
                    max_future_skew_s=self.max_future_skew_s,
                )
            except (CadInputError, ValueError) as exc:
                self.invalid_packets += 1
                self.last_error = str(exc)
                continue
            self.valid_packets += 1
            self.last_error = None
            self.mailbox.publish(sample)
