from __future__ import annotations

import json
import subprocess

import pytest

from scripts.check_workstation import robot_ips_from_config
from widowxai_quest_teleop.workstation import (
    WorkstationSafetyError,
    run_workstation_preflight,
)


def power_supply(tmp_path, *, online: bool = True):
    mains = tmp_path / "AC"
    mains.mkdir(exist_ok=True)
    (mains / "type").write_text("Mains\n", encoding="utf-8")
    (mains / "online").write_text("1\n" if online else "0\n", encoding="utf-8")
    return tmp_path


def runner_for(*, source="192.168.1.10", gateway=None, profile="performance"):
    def run(command, **_kwargs):
        if command == ["powerprofilesctl", "get"]:
            return subprocess.CompletedProcess(command, 0, stdout=f"{profile}\n", stderr="")
        robot_ip = command[-1]
        route = {"dst": robot_ip, "dev": "enxrobot", "prefsrc": source}
        if gateway is not None:
            route["gateway"] = gateway
        return subprocess.CompletedProcess(
            command, 0, stdout=json.dumps([route]), stderr=""
        )

    return run


def test_workstation_preflight_accepts_ac_performance_and_direct_routes(tmp_path) -> None:
    report = run_workstation_preflight(
        {},
        ["192.168.1.2", "192.168.1.3"],
        power_supply_root=power_supply(tmp_path),
        runner=runner_for(),
    )
    assert report.power_supply == "AC"
    assert report.power_profile == "performance"
    assert report.route_devices == ("enxrobot", "enxrobot")


@pytest.mark.parametrize(
    ("runner", "message"),
    [
        (runner_for(profile="balanced"), "power profile"),
        (runner_for(source="10.54.4.131"), "dedicated robot Ethernet"),
        (runner_for(gateway="10.54.0.1"), "gateway"),
    ],
)
def test_workstation_preflight_rejects_known_live_run_risks(
    tmp_path, runner, message
) -> None:
    with pytest.raises(WorkstationSafetyError, match=message):
        run_workstation_preflight(
            {},
            ["192.168.1.2"],
            power_supply_root=power_supply(tmp_path),
            runner=runner,
        )


def test_workstation_preflight_rejects_battery_and_cannot_be_disabled(tmp_path) -> None:
    with pytest.raises(WorkstationSafetyError, match="external AC"):
        run_workstation_preflight(
            {},
            ["192.168.1.2"],
            power_supply_root=power_supply(tmp_path, online=False),
            runner=runner_for(),
        )
    with pytest.raises(WorkstationSafetyError, match="cannot disable"):
        run_workstation_preflight(
            {"workstation_preflight": {"enabled": False}},
            ["192.168.1.2"],
            power_supply_root=power_supply(tmp_path),
            runner=runner_for(),
        )


def test_workstation_checker_extracts_single_and_dual_robot_ips() -> None:
    assert robot_ips_from_config({"hardware": {"robot_ip": "192.168.1.2"}}) == [
        "192.168.1.2"
    ]
    assert robot_ips_from_config(
        {
            "arms": {
                "left": {"robot_ip": "192.168.1.2"},
                "right": {"robot_ip": "192.168.1.3"},
            }
        }
    ) == ["192.168.1.2", "192.168.1.3"]
