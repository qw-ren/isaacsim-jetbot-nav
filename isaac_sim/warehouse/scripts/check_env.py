#!/usr/bin/env python3
"""Environment check for the warehouse scene module.

All team machines have Isaac Sim 4.5 assets at the SAME location, so the scene
references are authored as absolute paths and no relinking is needed:

    /home/admin/isaacsim_assets/Assets/Isaac/4.5/Isaac/...

The only thing that depends on the clone location is `jetbot_sim.usd`, whose
internal references are `../../../isaacsim_assets/...`. It therefore has to live
one of these two places (never edited):

    /home/admin/Desktop/jetbot_project/project/jetbot_sim.usd     (canonical)
    $JETBOT_USD                                                   (override)

This script verifies everything and, when run with --copy-robot, copies the
in-repo copy to the canonical location.

    python3 isaac_sim/warehouse/scripts/check_env.py
    python3 isaac_sim/warehouse/scripts/check_env.py --copy-robot
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
from pathlib import Path

WAREHOUSE = Path(__file__).resolve().parents[1]      # isaac_sim/warehouse
REPO = WAREHOUSE.parents[1]                          # repo root
SCENES = WAREHOUSE / "scenes"
SCENE = SCENES / "warehouse_base.usd"
ROBOT_IN_REPO = WAREHOUSE / "project" / "jetbot_sim.usd"
ROBOT_CANONICAL = Path("/home/admin/Desktop/jetbot_project/project/jetbot_sim.usd")
ASSETS_HOME = Path("/home/admin/isaacsim_assets")
ISAAC = ASSETS_HOME / "Assets/Isaac/4.5/Isaac"
REQUIRED = [
    "Environments/Simple_Warehouse/full_warehouse.usd",
    "Robots/Jetbot/jetbot.usd",
]


def check_assets() -> bool:
    print("[1] official assets at %s" % ISAAC)
    if not ISAAC.is_dir():
        print("    MISSING asset root")
        return False
    ok = True
    for rel in REQUIRED:
        p = ISAAC / rel
        print("    %-58s %s" % (rel, "OK" if p.exists() else "MISSING"))
        ok &= p.exists()
    lidar = list(ISAAC.glob("**/XT-32*.usd"))
    print("    %-58s %s" % ("Hesai XT-32", "OK" if lidar else "MISSING"))
    return ok and bool(lidar)


def check_scene() -> bool:
    print("[2] scenes/warehouse_base.usd")
    if not SCENE.exists():
        print("    MISSING")
        return False
    h = hashlib_sha(SCENE)
    meta = json.loads((SCENES / "base_meta.json").read_text())
    mapmeta = json.loads((SCENES / "base_map_meta.json").read_text())
    ok = True
    for name, m in (("base_meta.json", meta), ("base_map_meta.json", mapmeta)):
        same = m.get("derived_usd_sha256") == h
        print("    hash matches %-20s %s" % (name, "OK" if same else "MISMATCH"))
        ok &= same
    try:
        from pxr import Usd
        st = Usd.Stage.Open(str(SCENE))
        n = len(st.GetPrimAtPath("/World/Warehouse").GetChildren())
        print("    /World/Warehouse children: %d %s" % (n, "OK" if n else "EMPTY"))
        ok &= n > 0
    except ImportError:
        print("    (pxr unavailable - skipped stage open)")
    return ok


def check_robot(copy_robot: bool) -> bool:
    print("[3] jetbot_sim.usd")
    env = os.environ.get("JETBOT_USD")
    target = Path(env) if env else ROBOT_CANONICAL
    print("    expected at: %s" % target)
    if not target.exists():
        print("    MISSING")
        if copy_robot and ROBOT_IN_REPO.exists():
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(ROBOT_IN_REPO, target)
            print("    copied from %s" % ROBOT_IN_REPO)
        else:
            print("    run: python3 isaac_sim/warehouse/scripts/check_env.py --copy-robot")
            print("    or:  export JETBOT_USD=<path to jetbot_sim.usd>")
            return False
    try:
        from pxr import Usd
        st = Usd.Stage.Open(str(target))
        bot = st.GetPrimAtPath("/World/jetbot").IsValid()
        lidar = st.GetPrimAtPath("/World/jetbot/chassis/lidar/XT_32_10hz").IsValid()
        graphs = len(st.GetPrimAtPath("/World/ROS2").GetChildren())
        print("    robot: %s | lidar: %s | graphs: %d" % (bot, lidar, graphs))
        return bot and lidar and graphs > 0
    except ImportError:
        print("    (pxr unavailable - file exists, skipped stage open)")
        return True


def hashlib_sha(p: Path) -> str:
    import hashlib
    h = hashlib.sha256()
    h.update(p.read_bytes())
    return h.hexdigest()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--copy-robot", action="store_true",
                    help="copy jetbot_sim.usd to the canonical location if missing")
    a = ap.parse_args()

    print("repo:", REPO, "\n")
    ok = check_assets()
    ok &= check_scene()
    ok &= check_robot(a.copy_robot)
    print("\nRESULT:", "READY" if ok else "NOT READY")
    if ok:
        print("next:")
        print("  cd isaac_sim/warehouse")
        print("  <isaac>/python.sh examples/inspect_warehouse_jetbot.py")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
