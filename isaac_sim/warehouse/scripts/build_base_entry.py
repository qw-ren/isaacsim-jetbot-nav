#!/usr/bin/env python3
"""Build scenes/warehouse_base.usd — a NON-DESTRUCTIVE reference entry point.

The official full_warehouse.usd is never modified. This file only *references* it
under /World/Warehouse, giving teammates a stable, project-local stage path.

Stage 2 will add an override layer on top for (a) deactivating existing clutter
and (b) injecting seed-generated obstacles. Both are overrides, never edits to the
official asset.

Requires Isaac Sim's pxr bindings. Run e.g.:
    python3 scripts/build_base_entry.py --out scenes/warehouse_base.usd
or inside python.sh with pxr importable.
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pxr import Usd, UsdGeom  # noqa: E402

from generator import config  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=None)
    ap.add_argument("--relative", action="store_true",
                    help="store a path relative to the output file")
    args = ap.parse_args()

    wh = config.load_warehouse()
    reg = config.load_asset_registry()
    src_root = reg["source_root"]
    src = os.path.join(src_root, wh["base"]["usd"])
    if not os.path.exists(src):
        print("ERROR source not found:", src)
        return 2

    root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out = args.out or os.path.join(root_dir, "scenes", "warehouse_base.usd")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    if os.path.exists(out):
        os.remove(out)

    st = Usd.Stage.CreateNew(out)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(st, "/World")
    st.SetDefaultPrim(world.GetPrim())
    holder = UsdGeom.Xform.Define(st, "/World/Warehouse")
    ref = os.path.relpath(src, os.path.dirname(out)) if args.relative else src
    holder.GetPrim().GetReferences().AddReference(ref)
    st.GetRootLayer().Save()

    print("wrote", out, "size", os.path.getsize(out), "bytes")
    print("  reference ->", ref)

    # verify by reopening
    st2 = Usd.Stage.Open(out)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    dp = st2.GetDefaultPrim()
    rng = cache.ComputeWorldBound(dp).ComputeAlignedRange()
    mn, mx = rng.GetMin(), rng.GetMax()
    print("  verify: prims=%d" % len(list(st2.Traverse())))
    print("  verify: world bbox min", [round(float(v), 2) for v in mn],
          "max", [round(float(v), 2) for v in mx])
    whp = st2.GetPrimAtPath("/World/Warehouse")
    print("  verify: /World/Warehouse children =", len(whp.GetChildren()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
