#!/usr/bin/env bash
set -euo pipefail

project_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$project_root"

# Stop only this project's Quest relay. The bracketed expressions keep pgrep
# from matching its own command line.
mapfile -t relay_pids < <(
  pgrep -f '([w]idowxai-quest-relay|[p]ython(3(\.[0-9]+)?)? -m widowxai_quest_teleop\.relay)' || true
)
for pid in "${relay_pids[@]}"; do
  kill "$pid" 2>/dev/null || true
done

for _ in $(seq 1 30); do
  relay_alive=false
  for pid in "${relay_pids[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      relay_alive=true
      break
    fi
  done
  if ! $relay_alive; then
    break
  fi
  sleep 0.1
done

if ! command -v adb >/dev/null 2>&1; then
  echo "ERROR: adb is not installed or is not on PATH" >&2
  exit 1
fi
if ! adb get-state 2>/dev/null | grep -qx device; then
  echo "ERROR: Quest is not visible to adb. Connect it over USB and accept USB debugging." >&2
  exit 1
fi

adb reverse --remove tcp:8443 >/dev/null 2>&1 || true
adb reverse tcp:8443 tcp:8443 >/dev/null

# Keep development passthrough/WebXR alive while the headset is off the face.
# Horizon OS clears the proximity override on reboot, so reapply it whenever
# the relay is restarted. The stay-on mask covers USB or AC-powered hubs.
adb shell taskset 0000000F am broadcast \
  -a com.oculus.vrpowermanager.prox_close >/dev/null
adb shell svc power stayon true
adb shell input keyevent KEYCODE_WAKEUP

echo "Quest relay: http://localhost:8443"
echo "Quest proximity sleep disabled until the next headset reboot."
echo "Press Ctrl+C to stop it."
exec env -u PYTHONPATH .venv/bin/widowxai-quest-relay --host 0.0.0.0 --port 8443
