#!/usr/bin/env python3
"""v1.6 experiment: A-class layout (aisle pref 2.0 vs 1.4), B-class blockage,
and the three modes A_only / B_only / A_B_mixed.

Outputs JSON specs + a metrics table to outputs/v16_experiment/.
"""
import json
import os
import sys
from collections import Counter

import numpy as np
import yaml
from scipy import ndimage

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from generator import config  # noqa
from generator.scene_generator import SceneGenerator, layout_hash  # noqa
from generator import scene_mode  # noqa
from generator.topology import Topology  # noqa

OUT = os.path.join(ROOT, "outputs", "v16_experiment")
os.makedirs(OUT, exist_ok=True)

cfgs = config.load_all()
with open(os.path.join(ROOT, "config", "difficulty_v15_experiment.yaml")) as f:
    cfgs["difficulty_v15"] = yaml.safe_load(f)
gen = SceneGenerator.from_config()
md = gen.md
topo = Topology(md, cfgs["warehouse"])

SEGS = []
res = md["res"]; ox, oy = md["origin"]; A = res * res
for region, key in (("aisle", "region_aisle"), ("shelf_edge", "region_edge")):
    lab, n = ndimage.label(md[key])
    for i in range(1, n + 1):
        sub = (lab == i); c = int(sub.sum())
        if c * A < 8.0:
            continue
        SEGS.append({"region": region, "mask": sub, "area": c * A})
gs = int(round(6.0 / res)); m = md["region_open"]; ny, nx = m.shape
for gy in range(0, ny, gs):
    for gx in range(0, nx, gs):
        sub = m[gy:gy + gs, gx:gx + gs]; c = int(sub.sum())
        if c < int(0.35 * gs * gs):
            continue
        full = np.zeros_like(m); full[gy:gy + gs, gx:gx + gs] = sub
        SEGS.append({"region": "open", "mask": full, "area": c * A})


def metrics(spec):
    objs = spec["objects"]
    per = Counter(); per_region = Counter()
    for o in objs:
        ix = int((o["position"][0] - ox) / res); iy = int((o["position"][1] - oy) / res)
        ix = max(0, min(nx - 1, ix)); iy = max(0, min(ny - 1, iy))
        hit = -1
        for k, s in enumerate(SEGS):
            if s["mask"][iy, ix]:
                hit = k; break
        per[hit] += 1
        per_region[o.get("region", "?")] += 1
    occ = len([k for k in per if k >= 0])
    mx = max(per.values()) if per else 0
    yy, xx = np.nonzero(md["placeable"])
    px = ox + (xx + 0.5) * res; py = oy + (yy + 0.5) * res
    near = np.zeros(len(xx), dtype=bool)
    for o in objs:
        near |= ((px - o["position"][0]) ** 2 + (py - o["position"][1]) ** 2) <= 9.0
    return {"n_total": len(objs),
            "n_A": sum(1 for o in objs if o.get("kind") == "A"),
            "n_B": sum(1 for o in objs if o.get("kind") == "B"),
            "coverage_pct": round(100.0 * occ / len(SEGS), 1),
            "max_per_segment": mx,
            "empty_area_pct": round(100.0 * (1.0 - near.mean()), 1),
            "per_region": dict(per_region),
            "classes": dict(Counter(o["class"] for o in objs))}


SCENES = [
    ("A_1001M_pref20", dict(seed=1001, density="medium", mode="A_only", aisle_pref=2.0)),
    ("A_1001M_pref14", dict(seed=1001, density="medium", mode="A_only", aisle_pref=1.4)),
    ("A_1001H_pref20", dict(seed=1001, density="hard", mode="A_only", aisle_pref=2.0)),
    ("A_1001H_pref14", dict(seed=1001, density="hard", mode="A_only", aisle_pref=1.4)),
    ("A_1002M_pref20", dict(seed=1002, density="medium", mode="A_only", aisle_pref=2.0)),
    ("A_1002M_pref14", dict(seed=1002, density="medium", mode="A_only", aisle_pref=1.4)),
    ("A_1001E_pref20", dict(seed=1001, density="easy", mode="A_only", aisle_pref=2.0)),
    ("B_1001_1", dict(seed=1001, density="medium", mode="B_only", aisle_pref=2.0,
                      num_blocked_aisles=1)),
    ("M_1001M_1", dict(seed=1001, density="medium", mode="A_B_mixed", aisle_pref=2.0,
                       num_blocked_aisles=1)),
    ("M_1002H_1", dict(seed=1002, density="hard", mode="A_B_mixed", aisle_pref=2.0,
                       num_blocked_aisles=1)),
]

report = {"scenes": [], "segments": len(SEGS)}
for name, kw in SCENES:
    spec = scene_mode.build_scene(md, cfgs, **kw)
    spec["layout_hash"] = layout_hash([{k: o[k] for k in ("id", "class", "asset_key",
                                                          "position", "yaw_rad")}
                                       for o in spec["objects"]])
    spec["scene_name"] = name
    path = os.path.join(OUT, name + ".json")
    with open(path, "w") as f:
        json.dump(spec, f, indent=2, sort_keys=True, default=str)
    m = metrics(spec)
    m["name"] = name
    m["mode"] = spec["mode"]
    m["hash"] = spec["layout_hash"][:12]
    m["valid"] = spec["validation"]["valid"]
    m["blocked_aisles"] = spec.get("blocked_aisles", [])
    m["aisles_passable"] = [k for k, v in spec["aisle_state"].items() if v["passable"]]
    m["blockage_ok"] = all(b["verify"]["ok"] for b in spec.get("blockages", []))
    report["scenes"].append(m)
    print("%-16s n=%2d (A%2d B%d) cov=%5.1f%% maxSeg=%d empty=%5.1f%% valid=%s blocked=%s passable=%s B_ok=%s"
          % (name, m["n_total"], m["n_A"], m["n_B"], m["coverage_pct"], m["max_per_segment"],
             m["empty_area_pct"], m["valid"], m["blocked_aisles"], m["aisles_passable"],
             m["blockage_ok"]))

with open(os.path.join(OUT, "summary.json"), "w") as f:
    json.dump(report, f, indent=2, sort_keys=True)
print("\nwrote", os.path.join(OUT, "summary.json"))
print("V16_EXPERIMENT_DONE")
