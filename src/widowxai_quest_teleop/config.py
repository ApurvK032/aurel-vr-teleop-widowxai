from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = PROJECT_ROOT / "configs" / "baseline.yaml"


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

