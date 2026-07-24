from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = PROJECT_ROOT / "configs" / "baseline.yaml"
TASK_PROFILE_CATALOG = (
    PROJECT_ROOT
    / "configs"
    / "calibrations"
    / "task_profiles.json"
)
TASK_PROFILE_NAMES = (
    "left_behind",
    "left_mirror",
    "right_behind",
    "right_mirror",
)


def task_profile_for_quest_selection(hand: str, mapping_mode: str) -> str:
    selected_hand = str(hand).lower()
    selected_mode = str(mapping_mode).lower()
    if selected_hand not in ("left", "right"):
        raise ValueError(f"unsupported Quest hand: {hand}")
    semantics_by_mode = {"real": "behind", "mirror": "mirror"}
    if selected_mode not in semantics_by_mode:
        raise ValueError(f"unsupported Quest mapping mode: {mapping_mode}")
    return f"{selected_hand}_{semantics_by_mode[selected_mode]}"


def resolve_project_path(value: str | Path) -> Path:
    path = Path(value)
    return path if path.is_absolute() else PROJECT_ROOT / path


def load_config(path: str | Path = DEFAULT_CONFIG) -> dict[str, Any]:
    resolved = resolve_project_path(path)
    with resolved.open("r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    if not isinstance(config, dict):
        raise ValueError(f"configuration must be a mapping: {resolved}")
    config["_config_path"] = str(resolved)
    return config


def apply_task_profile(
    config: dict[str, Any],
    profile_name: str,
    catalog_path: str | Path = TASK_PROFILE_CATALOG,
) -> dict[str, Any]:
    """Apply one fixed hand/semantic mapping to an already loaded config."""

    if profile_name not in TASK_PROFILE_NAMES:
        raise ValueError(
            f"unknown task profile {profile_name!r}; choose one of {TASK_PROFILE_NAMES}"
        )
    resolved_catalog = resolve_project_path(catalog_path)
    catalog = json.loads(resolved_catalog.read_text(encoding="utf-8"))
    profile_path = catalog.get("profiles", {}).get(profile_name)
    if not isinstance(profile_path, str):
        raise ValueError(f"task profile catalog has no path for {profile_name}")
    resolved_profile = resolve_project_path(profile_path)
    profile = json.loads(resolved_profile.read_text(encoding="utf-8"))
    quest_input = profile.get("quest_input")
    if not isinstance(quest_input, dict):
        raise ValueError(f"task profile {profile_name} has no quest_input")
    expected_hand, expected_semantics = profile_name.split("_", maxsplit=1)
    hand = quest_input.get("hand")
    mapping_mode = quest_input.get("mapping_mode")
    semantics = profile.get("task_mapping_semantics", {}).get("name")
    if hand != expected_hand:
        raise ValueError(
            f"task profile {profile_name} reports hand {hand!r}, expected {expected_hand!r}"
        )
    expected_mode = "real" if expected_semantics == "behind" else "mirror"
    if mapping_mode != expected_mode:
        raise ValueError(
            f"task profile {profile_name} reports mode {mapping_mode!r}, "
            f"expected {expected_mode!r}"
        )
    if semantics != expected_semantics:
        raise ValueError(
            f"task profile {profile_name} reports semantics {semantics!r}, "
            f"expected {expected_semantics!r}"
        )

    config["quest"]["hand"] = hand
    config["quest"]["mapping_mode"] = mapping_mode
    config["quest"]["calibration"] = profile_path
    config["project"]["mode"] = (
        f"{config['project']['mode']}__task_profile_{profile_name}"
    )
    config["_task_profile"] = {
        "name": profile_name,
        "catalog": str(resolved_catalog),
        "calibration": str(resolved_profile),
        "hand": hand,
        "mapping_mode": mapping_mode,
        "semantics": semantics,
    }
    return config
