"""Trossen's right-leader / left-follower WidowXAI teleoperation demo.

This keeps the official demo's seven-axis copy and force-feedback loop.  Only
the controller IPs and default one-hour duration are bench-specific.
"""

from __future__ import annotations

import argparse
import time

import numpy as np
import trossen_arm


RIGHT_LEADER_IP = "192.168.1.3"
LEFT_FOLLOWER_IP = "192.168.1.2"
HOME_POSITIONS = np.array([0.0, np.pi / 2, np.pi / 2, 0.0, 0.0, 0.0, 0.0])
FORCE_FEEDBACK_GAIN = 0.1


def main() -> None:
    parser = argparse.ArgumentParser(description="Right WidowXAI leads the left WidowXAI")
    parser.add_argument("--duration", type=float, default=3600.0)
    args = parser.parse_args()
    if not np.isfinite(args.duration) or args.duration <= 0.0:
        raise SystemExit("--duration must be positive")

    leader = trossen_arm.TrossenArmDriver()
    follower = trossen_arm.TrossenArmDriver()
    configured = []
    try:
        print(f"Configuring right leader at {RIGHT_LEADER_IP}...")
        leader.configure(
            trossen_arm.Model.wxai_v0,
            trossen_arm.StandardEndEffector.wxai_v0_leader,
            RIGHT_LEADER_IP,
            False,
        )
        configured.append(leader)
        print(f"Configuring left follower at {LEFT_FOLLOWER_IP}...")
        follower.configure(
            trossen_arm.Model.wxai_v0,
            trossen_arm.StandardEndEffector.wxai_v0_follower,
            LEFT_FOLLOWER_IP,
            False,
        )
        configured.append(follower)

        print("Moving both arms to the shared home pose...")
        leader.set_all_modes(trossen_arm.Mode.position)
        follower.set_all_modes(trossen_arm.Mode.position)
        leader.set_all_positions(HOME_POSITIONS, 2.0, True)
        follower.set_all_positions(HOME_POSITIONS, 2.0, True)

        print("Hold the RIGHT leader. Teleoperation starts in 1 second...")
        time.sleep(1.0)
        leader.set_all_modes(trossen_arm.Mode.external_effort)
        follower.set_all_modes(trossen_arm.Mode.position)

        print(f"Following for {args.duration:g} seconds. Press Ctrl-C to stop.")
        end_time = time.monotonic() + args.duration
        while time.monotonic() < end_time:
            leader.set_all_external_efforts(
                -FORCE_FEEDBACK_GAIN
                * np.asarray(follower.get_all_external_efforts(), dtype=float),
                0.0,
                False,
            )
            follower.set_all_positions(
                leader.get_all_positions(),
                0.0,
                False,
                leader.get_all_velocities(),
            )
    except KeyboardInterrupt:
        print("Operator stop.")
    finally:
        if len(configured) == 2:
            print("Release the RIGHT arm. Returning both arms to home and rest...")
            leader.set_all_modes(trossen_arm.Mode.position)
            follower.set_all_modes(trossen_arm.Mode.position)
            leader.set_all_positions(HOME_POSITIONS, 2.0, True)
            follower.set_all_positions(HOME_POSITIONS, 2.0, True)
            leader.set_all_positions(np.zeros(leader.get_num_joints()), 2.0, True)
            follower.set_all_positions(np.zeros(follower.get_num_joints()), 2.0, True)
        for driver in reversed(configured):
            driver.cleanup()
        print("Done.")


if __name__ == "__main__":
    main()
