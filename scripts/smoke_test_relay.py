from __future__ import annotations

import json
import subprocess
import sys
import time
import urllib.request

from websockets.sync.client import connect


def main() -> None:
    creation_flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
    relay = subprocess.Popen(
        [sys.executable, "-m", "widowxai_quest_teleop.relay", "--host", "127.0.0.1", "--port", "8443"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        creationflags=creation_flags,
    )
    try:
        health = None
        for _ in range(30):
            try:
                with urllib.request.urlopen("http://127.0.0.1:8443/health", timeout=0.5) as response:
                    health = json.load(response)
                break
            except OSError:
                time.sleep(0.1)
        assert health is not None and health["status"] == "ok"
        with connect("ws://127.0.0.1:8443/ws") as subscriber, connect(
            "ws://127.0.0.1:8443/ws"
        ) as source:
            source.send(
                json.dumps(
                    {
                        "type": "pose",
                        "schema_version": 1,
                        "sequence": 42,
                        "capture_monotonic_ms": 100.0,
                        "capture_epoch_ms": 200.0,
                        "send_monotonic_ms": 101.0,
                        "left": {
                            "position": [0.1, 0.2, 0.3],
                            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                            "grip": 1.0,
                            "trigger": 0.0,
                        },
                    }
                )
            )
            received = json.loads(subscriber.recv(timeout=2.0))
        assert received["sequence"] == 42
        assert "relay_arrival_monotonic_ns" in received
        assert "relay_arrival_epoch_ns" in received
        assert received["reconnect_generation"] > 0
        with urllib.request.urlopen("http://127.0.0.1:8443/", timeout=2.0) as response:
            assert response.status == 200
        print("relay smoke test passed")
    finally:
        relay.terminate()
        relay.wait(timeout=3.0)


if __name__ == "__main__":
    main()
