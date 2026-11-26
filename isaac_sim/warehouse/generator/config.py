"""Config loading for the warehouse scene generator (Stage 0, implemented).

This module is dependency-light (PyYAML only) and Isaac-Sim independent, so it can
be unit-tested on any machine.
"""
from __future__ import annotations

import os
from pathlib import Path

try:
    import yaml  # type: ignore
except ImportError:  # pragma: no cover
    yaml = None

ROOT = Path(__file__).resolve().parent.parent
CONFIG_DIR = ROOT / "config"

CONFIG_FILES = {
    "warehouse": "warehouse.yaml",
    "obstacles": "obstacles.yaml",
    "difficulty": "difficulty.yaml",
    "asset_registry": "asset_registry.yaml",
}


def _require_yaml():
    if yaml is None:
        raise RuntimeError(
            "PyYAML is required to load configs. On this machine use system "
            "python3 (yaml 5.4.1) or the Isaac Sim python (yaml 6.0.1)."
        )


def load(name: str) -> dict:
    """Load one config by logical name (see CONFIG_FILES)."""
    _require_yaml()
    fname = CONFIG_FILES[name]
    path = CONFIG_DIR / fname
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def load_warehouse() -> dict:
    return load("warehouse")


def load_obstacles() -> dict:
    return load("obstacles")


def load_difficulty() -> dict:
    return load("difficulty")


def load_asset_registry() -> dict:
    return load("asset_registry")


def asset_root_override():
    """$ISAAC_ASSET_ROOT, else ~/isaacsim_assets/Assets/Isaac/4.5/Isaac if present."""
    env = os.environ.get("ISAAC_ASSET_ROOT")
    if env:
        return env
    cand = os.path.expanduser("~/isaacsim_assets/Assets/Isaac/4.5/Isaac")
    return cand if os.path.isdir(cand) else None


def load_all() -> dict:
    cfgs = {k: load(k) for k in CONFIG_FILES}
    root = asset_root_override()
    if root:
        for k, v in cfgs.items():
            if isinstance(v, dict) and "source_root" in v:
                v["source_root"] = root
    return cfgs


def asset_abs_path(registry: dict, asset_id: str) -> str:
    """Resolve a registry asset id to an absolute USD path."""
    a = registry["assets"][asset_id]
    return os.path.join(registry["source_root"], a["path"])


def config_hash() -> str:
    """Stable sha256 over the 4 YAML configs (name + bytes, sorted)."""
    import hashlib
    h = hashlib.sha256()
    for name in sorted(CONFIG_FILES.values()):
        p = CONFIG_DIR / name
        with open(p, "rb") as f:
            h.update(name.encode())
            h.update(b"\0")
            h.update(f.read())
            h.update(b"\0")
    return h.hexdigest()
