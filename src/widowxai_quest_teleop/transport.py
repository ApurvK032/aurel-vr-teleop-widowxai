from __future__ import annotations

import json
import threading
import time
from typing import Any

import numpy as np
from websockets.sync.client import connect

from .sample_buffer import LatestValueMailbox
from .types import Pose, QuestSample


QUEST_HANDS = ("left", "right")
QUEST_MAPPING_MODES = ("real", "mirror")


def _validated_selection(
    value: str | None,
    choices: tuple[str, ...],
    label: str,
) -> str | None:
    if value is None:
        return None
    selected = str(value).lower()
    if selected not in choices:
        raise ValueError(f"unsupported Quest {label}: {value}")
    return selected


def _reported_hand(payload: dict[str, Any], expected_hand: str | None) -> str:
    reported = payload.get("selected_hand")
    if reported is not None:
        selected = _validated_selection(str(reported), QUEST_HANDS, "hand")
        assert selected is not None
        return selected
    if expected_hand is not None:
        return expected_hand
    available = [hand for hand in QUEST_HANDS if isinstance(payload.get(hand), dict)]
    if len(available) != 1:
        raise ValueError("pose message must identify exactly one selected controller")
    return available[0]


def _mirror_webxr_pose(
    position: np.ndarray,
    orientation_xyzw: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Reflect a pose across WebXR's operator sagittal plane."""

    mirrored_position = position.copy()
    mirrored_position[0] *= -1.0
    mirrored_orientation = orientation_xyzw.copy()
    mirrored_orientation[1:3] *= -1.0
    return mirrored_position, mirrored_orientation


def parse_pose_message(
    raw: str | dict[str, Any],
    hand: str | None = "left",
    mapping_mode: str | None = "real",
) -> QuestSample:
    payload = json.loads(raw) if isinstance(raw, str) else raw
    if payload.get("type") != "pose":
        raise ValueError("not a pose message")
    expected_hand = _validated_selection(hand, QUEST_HANDS, "hand")
    expected_mapping_mode = _validated_selection(
        mapping_mode,
        QUEST_MAPPING_MODES,
        "mapping mode",
    )
    selected_hand = _reported_hand(payload, expected_hand)
    if expected_hand is not None and selected_hand != expected_hand:
        raise ValueError(
            f"Quest page selected {selected_hand}, expected {expected_hand}"
        )
    selected_mapping_mode = _validated_selection(
        str(payload.get("mapping_mode", "real")),
        QUEST_MAPPING_MODES,
        "mapping mode",
    )
    assert selected_mapping_mode is not None
    if (
        expected_mapping_mode is not None
        and selected_mapping_mode != expected_mapping_mode
    ):
        raise ValueError(
            "Quest page selected mapping mode "
            f"{selected_mapping_mode}, expected {expected_mapping_mode}"
        )
    controller_payload = payload.get(selected_hand)
    if not isinstance(controller_payload, dict):
        raise ValueError(f"pose message has no {selected_hand} controller")
    xyzw = np.asarray(controller_payload["orientation_xyzw"], dtype=float).reshape(4)
    position = np.asarray(controller_payload["position"], dtype=float).reshape(3)
    if selected_mapping_mode == "mirror":
        position, xyzw = _mirror_webxr_pose(position, xyzw)
    controller = Pose(
        position,
        np.array([xyzw[3], xyzw[0], xyzw[1], xyzw[2]], dtype=float),
    )
    head_wxyz = None
    # Prefer the session-locked operator frame from new clients. Older clients
    # and recordings still work through the live-head fallback.
    head = payload.get("operator_head")
    if not isinstance(head, dict) or "orientation_xyzw" not in head:
        head = payload.get("head")
    if isinstance(head, dict) and "orientation_xyzw" in head:
        head_xyzw = np.asarray(head["orientation_xyzw"], dtype=float).reshape(4)
        head_wxyz = np.array([head_xyzw[3], head_xyzw[0], head_xyzw[1], head_xyzw[2]])
    return QuestSample(
        sequence=int(payload["sequence"]),
        capture_monotonic_ms=float(payload["capture_monotonic_ms"]),
        capture_epoch_ms=float(payload["capture_epoch_ms"]),
        send_monotonic_ms=float(payload["send_monotonic_ms"]),
        pc_arrival_monotonic_ns=time.perf_counter_ns(),
        pc_arrival_epoch_ns=time.time_ns(),
        reconnect_generation=int(payload.get("reconnect_generation", 0)),
        controller_pose=controller,
        grip=float(controller_payload.get("grip", 0.0)),
        trigger=float(controller_payload.get("trigger", 0.0)),
        hand=selected_hand,
        mapping_mode=selected_mapping_mode,
        head_quaternion_wxyz=head_wxyz,
    )


class QuestReceiver:
    """Reconnecting WebSocket subscriber backed by a capacity-one mailbox."""

    def __init__(
        self,
        websocket_url: str,
        *,
        hand: str | None = "left",
        mapping_mode: str | None = "real",
    ) -> None:
        selected_hand = _validated_selection(hand, QUEST_HANDS, "hand")
        selected_mapping_mode = _validated_selection(
            mapping_mode,
            QUEST_MAPPING_MODES,
            "mapping mode",
        )
        self.websocket_url = websocket_url
        self.hand = selected_hand
        self.mapping_mode = selected_mapping_mode
        self.mailbox: LatestValueMailbox[QuestSample] = LatestValueMailbox()
        self.bad_messages = 0
        self.reconnects = 0
        self.last_error: str | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._websocket_lock = threading.Lock()
        self._websocket = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="quest-websocket", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        with self._websocket_lock:
            websocket = self._websocket
        if websocket is not None:
            try:
                websocket.close()
            except Exception:
                pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def publish(self, payload: dict[str, Any]) -> bool:
        """Publish a control-state event through the relay connection."""

        with self._websocket_lock:
            websocket = self._websocket
        if websocket is None:
            return False
        try:
            websocket.send(json.dumps(payload, separators=(",", ":")))
        except Exception as exc:
            self.last_error = f"{type(exc).__name__}: {exc}"
            return False
        return True

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                with connect(self.websocket_url, open_timeout=2.0, close_timeout=1.0) as websocket:
                    with self._websocket_lock:
                        self._websocket = websocket
                    try:
                        self.reconnects += 1
                        self.last_error = None
                        for raw in websocket:
                            if self._stop.is_set():
                                break
                            try:
                                sample = parse_pose_message(
                                    raw,
                                    self.hand,
                                    self.mapping_mode,
                                )
                            except (ValueError, KeyError, TypeError, json.JSONDecodeError):
                                self.bad_messages += 1
                                continue
                            self.mailbox.publish(sample)
                    finally:
                        with self._websocket_lock:
                            if self._websocket is websocket:
                                self._websocket = None
            except Exception as exc:
                self.last_error = f"{type(exc).__name__}: {exc}"
                self._stop.wait(0.5)
