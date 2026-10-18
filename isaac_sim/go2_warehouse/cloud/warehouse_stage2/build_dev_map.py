#!/usr/bin/env python3
"""Build a conservative USD collision-AABB map for development, not SLAM."""
import argparse
import hashlib
import json
import math
import os
import sys
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--scene", type=Path, required=True)
args = parser.parse_args()
if not os.environ.get("GO2_USD_CPU_READY"):
    usd_lib = next((Path(os.environ["GO2_ISAAC_ROOT"]) / "extscache").glob("omni.usd.libs-*"))
    environment = os.environ.copy()
    environment["PYTHONPATH"] = f"{usd_lib}:{environment.get('PYTHONPATH', '')}"
    environment["LD_LIBRARY_PATH"] = f"{usd_lib / 'bin'}:{environment.get('LD_LIBRARY_PATH', '')}"
    environment["GO2_USD_CPU_READY"] = "1"
    os.execvpe(sys.executable, [sys.executable, *sys.argv], environment)

import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics

asset = Path(os.environ["GO2_ASSET_ROOT"]) / "Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd"
scene = json.loads(args.scene.read_text())
assert scene["env_name"] == "warehouse-shelves" and scene["meters_per_unit"] == 1.0 and scene["up_axis"] == "Z"
stage = Usd.Stage.CreateInMemory()
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
warehouse = stage.DefinePrim("/World/Warehouse", "Xform")
warehouse.GetReferences().AddReference(str(asset))
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
bounds = cache.ComputeWorldBound(warehouse).ComputeAlignedRange()
lo, hi = np.array(bounds.GetMin()), np.array(bounds.GetMax())
assert np.max(np.abs(lo - scene["bounds_min"])) < .001
assert np.max(np.abs(hi - scene["bounds_max"])) < .001
colliders = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
assert len(colliders) == scene["warehouse_collision_prims"]
boxes = []
for prim in colliders:
    if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:
        continue
    r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    a, b = np.array(r.GetMin()), np.array(r.GetMax())
    if not np.isfinite(np.r_[a, b]).all() or b[2] <= .05 or a[2] >= .85:
        continue
    boxes.append({"path": str(prim.GetPath()), "min": a.tolist(), "max": b.tolist()})

resolution = .05
origin = np.floor((lo[:2] - .5) / resolution) * resolution
size = np.ceil((hi[:2] + .5 - origin) / resolution).astype(int)
x = origin[0] + (np.arange(size[0]) + .5) * resolution
y = origin[1] + (np.arange(size[1]) + .5) * resolution
# Free space is a development assumption within the rectangular scene extent;
# it is not an observed free-space or SLAM guarantee. Unknown outer border.
pixels = np.full((size[1], size[0]), 128, dtype=np.uint8)
inside = ((y[:, None] >= lo[1]) & (y[:, None] <= hi[1]) &
          (x[None, :] >= lo[0]) & (x[None, :] <= hi[0]))
pixels[inside] = 254
for box in boxes:
    a, b = np.array(box["min"]), np.array(box["max"])
    start = np.maximum(0, np.floor((a[:2] - origin) / resolution).astype(int))
    end = np.minimum(size - 1, np.floor((b[:2] - origin) / resolution).astype(int))
    if np.all(end >= start):
        pixels[start[1]:end[1]+1, start[0]:end[0]+1] = 0

args.output.mkdir(parents=True, exist_ok=False)
pgm = f"P5\n# DEVELOPMENT USD AABB PROJECTION - NOT SLAM\n{size[0]} {size[1]}\n255\n".encode() + pixels[::-1].tobytes()
(args.output / "map.pgm").write_bytes(pgm)
(args.output / "map.yaml").write_text(
    f"image: map.pgm\nmode: trinary\nresolution: {resolution}\norigin: [{origin[0]}, {origin[1]}, 0.0]\n"
    "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
(args.output / "collision_boxes.json").write_text(json.dumps(boxes, indent=2) + "\n")
metadata = {
    "date": "2026-10-15", "kind": "development_collision_aabb_projection_NOT_SLAM",
    "asset_relative": "Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd",
    "root_asset_sha256": hashlib.sha256(asset.read_bytes()).hexdigest(),
    "source_scene": scene, "collision_prims_total": len(colliders), "selected_boxes": len(boxes),
    "height_band_m": [.05, .85], "resolution_m": resolution, "origin": origin.tolist(),
    "width": int(size[0]), "height": int(size[1]),
    "occupied_cells": int((pixels == 0).sum()), "free_cells": int((pixels == 254).sum()),
    "unknown_cells": int((pixels == 128).sum()),
    "pgm_values": {"occupied": 0, "free": 254, "unknown": 128},
    "limitations": ["AABBs overestimate rotated/disconnected mesh obstacles", "Rectangular interior assumed free where no selected collider exists",
                    "Not ray-traced, observed, or SLAM-generated", "Not a body swept-volume collision certification"],
}
(args.output / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
(args.output / "map.sha256").write_text("".join(hashlib.sha256((args.output / name).read_bytes()).hexdigest() + "  " + name + "\n"
                                                for name in ("map.pgm", "map.yaml", "metadata.json", "collision_boxes.json")))
print(json.dumps({key: metadata[key] for key in ("kind", "selected_boxes", "width", "height", "occupied_cells", "free_cells")}, indent=2))
