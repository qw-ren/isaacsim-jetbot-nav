#!/usr/bin/env python3
"""Inspect the shipped USD collision bounds to select a conservative test spawn."""
import argparse
import json
import math
import os
import sys
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
if not os.environ.get("GO2_USD_CPU_READY"):
    usd_lib = next((Path(os.environ["GO2_ISAAC_ROOT"]) / "extscache").glob("omni.usd.libs-*"))
    cpu_env = os.environ.copy()
    cpu_env["PYTHONPATH"] = f"{usd_lib}:{cpu_env.get('PYTHONPATH', '')}"
    cpu_env["LD_LIBRARY_PATH"] = f"{usd_lib / 'bin'}:{cpu_env.get('LD_LIBRARY_PATH', '')}"
    cpu_env["GO2_USD_CPU_READY"] = "1"
    os.execvpe(sys.executable, [sys.executable, *sys.argv], cpu_env)
from pxr import Usd, UsdGeom, UsdPhysics

asset = Path(os.environ["GO2_ASSET_ROOT"]) / "Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd"
stage = Usd.Stage.Open(str(asset))
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
boxes = []
for prim in stage.Traverse():
    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        continue
    bounds = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    lo, hi = list(bounds.GetMin()), list(bounds.GetMax())
    if not all(math.isfinite(v) for v in lo + hi):
        continue
    if hi[2] <= 0.03 or lo[2] >= 0.85:
        continue
    boxes.append({"path": str(prim.GetPath()), "min": lo, "max": hi})


def occupied(x, y, margin=0.6):
    return any(b["min"][0] - margin <= x <= b["max"][0] + margin and
               b["min"][1] - margin <= y <= b["max"][1] + margin for b in boxes)


candidates = []
for ix in range(-20, 21):
    for iy in range(-26, 31):
        x, y = ix * 0.5, iy * 0.5
        # Reserve test motion and a rear camera corridor, not a navigation route.
        for yaw in (0.0, math.pi / 2, math.pi, -math.pi / 2):
            if not any(occupied(x + math.cos(yaw) * step * 0.2,
                                y + math.sin(yaw) * step * 0.2) for step in range(-20, 16)):
                candidates.append({"x": x, "y": y, "yaw": yaw,
                                   "distance_from_origin": math.hypot(x, y)})
candidates.sort(key=lambda row: row["distance_from_origin"])
near_origin = [b for b in boxes if b["min"][0] <= 0.6 and b["max"][0] >= -0.6 and
               b["min"][1] <= 0.6 and b["max"][1] >= -0.6]
result = {"asset": str(asset), "meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage),
          "up_axis": str(UsdGeom.GetStageUpAxis(stage)), "low_collision_boxes": len(boxes),
          "origin_near_colliders": near_origin[:15], "free_candidates": candidates[:12],
          "collision_boxes": boxes,
          "footprint_margin_m": 0.6, "corridor_offsets_m": [-4, 3]}
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps(result, indent=2), flush=True)
