#!/usr/bin/env python3
"""Derive an isolated motion copy; reject changed input or existing output."""
import ast
import hashlib
import json
import shutil
import subprocess
from pathlib import Path

scripts = Path(__file__).resolve().parent
root = Path("/home/admin/go2-warehouse")
source, target = root / "work", root / "motion_work_20261015"
for line in (scripts / "motion_base.sha256").read_text().splitlines():
    expected, name = line.split(maxsplit=1)
    assert hashlib.sha256((source / name).read_bytes()).hexdigest() == expected, name
if target.exists():
    raise SystemExit("Motion copy already exists; inspect it, do not overwrite")
shutil.copytree(source, target, ignore=shutil.ignore_patterns("outputs", "__pycache__", ".git"))
patch = str(scripts / "motion_runtime.patch")
subprocess.run(["patch", "--dry-run", "-p1", "-i", patch], cwd=target, check=True)
subprocess.run(["patch", "-p1", "-i", patch], cwd=target, check=True)
for name in ("isaac_go2_ros2.py", "ros2/go2_ros2_bridge.py"):
    ast.parse((target / name).read_text())
metadata = {"source": str(source), "target": str(target), "main_loop_command_timeout_wall_s": .5,
            "runtime_vx_limit_mps": .8, "runtime_yaw_limit_radps": .6,
            "keyboard_enabled": False, "cmd_vel_alias_enabled": False,
            "patch_sha256": hashlib.sha256(Path(patch).read_bytes()).hexdigest()}
(target / "MOTION_COPY.json").write_text(json.dumps(metadata, indent=2) + "\n")
print(json.dumps(metadata, indent=2))
