#!/usr/bin/env python3
"""Build the DERIVED base warehouse (non-destructive) + version metadata.

Order matters (per user correction): the derived USD with the 7 floor cartons
DISABLED must exist BEFORE the 2D occupancy map is built. The map is then built
from this derived USD, so 2D and 3D can never diverge.

Outputs
  scenes/warehouse_base.usd     root layer: reference to official + active=False overrides
  scenes/base_meta.json         checksums of official USD, derived USD, all referenced
                                assets, and the generation config

Requires pxr (Isaac Sim bindings), e.g.:
  python3 scripts/build_derived_base.py
"""
from __future__ import annotations

import hashlib
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pxr import Usd, UsdGeom, Sdf  # noqa: E402

from generator import config  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def sha256_file(path, chunk=1 << 20):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while True:
            b = f.read(chunk)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


def config_hash():
    """Stable hash over the 4 YAML configs (raw bytes, sorted by name)."""
    h = hashlib.sha256()
    for name in sorted(config.CONFIG_FILES.values()):
        p = os.path.join(ROOT, "config", name)
        with open(p, "rb") as f:
            h.update(name.encode())
            h.update(b"\0")
            h.update(f.read())
            h.update(b"\0")
    return h.hexdigest()


def main():
    wh = config.load_warehouse()
    reg = config.load_asset_registry()
    src_root = reg["source_root"]
    official = os.path.join(src_root, wh["base"]["usd"])
    if not os.path.exists(official):
        print("ERROR official base not found:", official)
        return 2

    clean_list = list(wh["existing_clutter"]["clean_list"])
    out = os.path.join(ROOT, wh["base"]["entry_point"])
    os.makedirs(os.path.dirname(out), exist_ok=True)

    # ---- build derived stage: reference + active=False overrides ----
    if os.path.exists(out):
        os.remove(out)
    st = Usd.Stage.CreateNew(out)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(st, "/World")
    st.SetDefaultPrim(world.GetPrim())
    holder = UsdGeom.Xform.Define(st, "/World/Warehouse")
    rel = os.path.relpath(official, os.path.dirname(out))
    holder.GetPrim().GetReferences().AddReference(rel)

    for name in clean_list:
        path = "/World/Warehouse/%s" % name
        prim = st.OverridePrim(path)
        prim.SetActive(False)
    st.GetRootLayer().Save()

    # ---- verify the override actually took effect ----
    st2 = Usd.Stage.Open(out)
    whp = st2.GetPrimAtPath("/World/Warehouse")
    inactive = 0
    missing = []
    for name in clean_list:
        p = st2.GetPrimAtPath("/World/Warehouse/%s" % name)
        if not p or not p.IsValid():
            missing.append(name)
            continue
        if not p.IsActive():
            inactive += 1
    active_children = len(whp.GetChildren())
    total_prims = len(list(st2.Traverse()))

    print("=== derived base built ===")
    print("  file            :", out, os.path.getsize(out), "bytes")
    print("  reference       :", rel)
    print("  clean_list      :", len(clean_list))
    print("  inactive ok     :", inactive, "/", len(clean_list))
    print("  missing prims   :", missing)
    print("  active children :", active_children)
    print("  total prims     :", total_prims)
    if inactive != len(clean_list) or missing:
        print("ERROR overrides did not apply")
        return 3

    # ---- metadata: checksums of official base + all referenced assets ----
    used = []
    C = config.load_obstacles()["classes"]
    for cname, c in C.items():
        for v in c.get("variants", []):
            if v in reg["assets"]:
                used.append(v)
        for strat in c.get("strategies", {}).values():
            for u in strat.get("units", []):
                if u in reg["assets"]:
                    used.append(u)
    used = sorted(set(used))

    assets_meta = {}
    for aid in used:
        p = os.path.join(src_root, reg["assets"][aid]["path"])
        assets_meta[aid] = {
            "path": reg["assets"][aid]["path"],
            "dims_m": reg["assets"][aid].get("dims_m"),
            "sha256": sha256_file(p),
            "size_bytes": os.path.getsize(p),
        }

    meta = {
        "schema_version": "1.0",
        "base_source": wh["base"]["usd"],
        "official_usd": official,
        "official_usd_size_bytes": os.path.getsize(official),
        "official_usd_sha256": sha256_file(official),
        "derived_usd": os.path.relpath(out, ROOT),
        "derived_usd_sha256": sha256_file(out),
        "derived_usd_size_bytes": os.path.getsize(out),
        "clean_list": clean_list,
        "clean_applied": inactive,
        "active_children": active_children,
        "total_prims": total_prims,
        "config_hash": config_hash(),
        "generator_version": "1.0.0",
        "asset_version": reg.get("asset_version"),
        "referenced_assets": assets_meta,
    }
    mp = os.path.join(ROOT, "scenes", "base_meta.json")
    with open(mp, "w") as f:
        json.dump(meta, f, indent=2, sort_keys=True)
    print("\n=== metadata ===")
    print("  official sha256 :", meta["official_usd_sha256"][:16], "...")
    print("  derived  sha256 :", meta["derived_usd_sha256"][:16], "...")
    print("  config_hash     :", meta["config_hash"][:16], "...")
    print("  referenced assets:", len(assets_meta))
    print("  wrote", mp)
    print("DERIVED_BASE_DONE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
