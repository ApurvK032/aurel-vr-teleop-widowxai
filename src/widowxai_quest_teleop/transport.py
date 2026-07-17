from __future__ import annotations

import json
import threading
import time
from typing import Any

import numpy as np
from websockets.sync.client import connect

from .sample_buffer import LatestValueMailbox
from .types import Pose, QuestSample


def parse_pose_message(raw: str | dict[str, Any]) -> QuestSample:
    payload = json.loads(raw) if isinstance(raw, str) else raw
    if payload.get("type") != "pose":
        raise ValueError("not a pose message")
    left = payload.get("left")
    if not isinstance(left, dict):
        raise ValueError("pose message has no left controller")
    xyzw = np.asarray(left["orientation_xyzw"], dtype=float).reshape(4)
    controller = Pose(
        np.asarray(left["position"], dtype=float),
        np.array([xyzw[3], xyzw[0], xyzw[1], xyzw[2]], dtype=float),
    )
    head_wxyz = None
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
        grip=float(left.get("grip", 0.0)),
        trigger=float(left.get("trigger", 0.0)),
        head_quaternion_wxyz=head_wxyz,
    )


class QuestReceiver:
    """Reconnecting WebSocket subscriber backed by a capacity-one mailbox."""

    def __init__(self, websocket_url: str) -> None:
        self.websocket_url = websocket_url
        self.mailbox: LatestValueMailbox[QuestSample] = LatestValueMailbox()
        self.bad_messages = 0
        self.reconnects = 0
        self.last_error: str | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="quest-websocket", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                with connect(self.websocket_url, open_timeout=2.0, close_timeout=1.0) as websocket:
                    self.reconnects += 1
                    self.last_error = None
                    for raw in websocket:
                        if self._stop.is_set():
                            break
                        try:
                            sample = parse_pose_message(raw)
                        except (ValueError, KeyError, TypeError, json.JSONDecodeError):
                            self.bad_messages += 1
                            continue
                        self.mailbox.publish(sample)
            except Exception as exc:
                self.last_error = f"{type(exc).__name__}: {exc}"
                self._stop.wait(0.5)

