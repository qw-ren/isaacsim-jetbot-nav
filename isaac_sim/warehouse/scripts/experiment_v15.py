#!/usr/bin/env python3
"""Compare layout v1.4.0 (accepted baseline) vs v1.5.0-exp (candidate).

Generates scenes for the comparison seeds with BOTH samplers, saves JSON,
validates legality, and reports distribution metrics:

  coverage   = segments that received >=1 object / total segments
  max/seg    = mean over scenes of the busiest segment's object count
  top1%      = share of objects sitting in the busiest segment
  emptyA%    = share of placeable cells farther than 3 m from any object
  nn_med     = median nearest-neighbour distance
  minLane    = narrowest remaining lateral gap inside any aisle strip (m)
"""
import json
import os
import sys
from collections import Counter

import numpy as np
from scipy import ndimage

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from generator import config  # noqa
from generator.scene_generator import SceneGenerator  # noqa
from generator import sampler_v2  # noqa
import yaml  # noqa

OUT = os.path.join(ROOT, "outputs", "v15_experiment")
os.makedirs(OUT, exist_ok=True)


def load_v15():
    with open(os.path.join(ROOT, "config", "difficulty_v15_experiment.yaml")) as f:
        return yaml.safe_load(f)


def make_segments(md):
    segs = []
    res = md["res"]; ox, oy = md["origin"]; A = res * res
    for region, key in (("aisle", "region_aisle"), ("shelf_edge", "region_edge")):
        lab, n = ndimage.label(md[key])
        for i in range(1, n + 1):
            sub = (lab == i); c = int(sub.sum())
            if c * A < 8.0:
                continue
            ys, xs = np.nonzero(sub)
            segs.append({"region": region, "mask": sub, "area": c * A,
                         "bbox": [ox + xs.min() * res, oy + ys.min() * res,
                                  ox + (xs.max() + 1) * res, oy + (ys.max() + 1) * res]})
    gs = int(round(6.0 / res)); m = md["region_open"]; ny, nx = m.shape
    for gy in range(0, ny, gs):
        for gx in range(0, nx, gs):
            sub = m[gy:gy + gs, gx:gx + gs]; c = int(sub.sum())
            if c < int(0.35 * gs * gs):
                continue
            full = np.zeros_like(m); full[gy:gy + gs, gx:gx + gs] = sub
            segs.append({"region": "open", "mask": full, "area": c * A,
                         "bbox": [ox + gx * res, oy + gy * res,
                                  ox + (gx + gs) * res, oy + (gy + gs) * res]})
    return segs


def metrics(objs, md, segs, placeable_area):
    res = md["res"]; ox, oy = md["origin"]
    ny, nx = md["region_open"].shape
    per = Counter()
    for o in objs:
        iy = int((o["position"][1] - oy) / res); ix = int((o["position"][0] - ox) / res)
        iy = max(0, min(ny - 1, iy)); ix = max(0, min(nx - 1, ix))
        hit = -1
        for k, s in enumerate(segs):
            if s["mask"][iy, ix]:
                hit = k; break
        per[hit] += 1
    occ = len([k for k in per if k >= 0])
    mx = max(per.values()) if per else 0

    # empty-area fraction (>3 m from any object)
    yy, xx = np.nonzero(md["placeable"])
    px = ox + (xx + 0.5) * res; py = oy + (yy + 0.5) * res
    near = np.zeros(len(xx), dtype=bool)
    for o in objs:
        near |= ((px - o["position"][0]) ** 2 + (py - o["position"][1]) ** 2) <= 9.0
    empty = 1.0 - near.mean()

    # nearest neighbour
    nn = []
    for i, o in enumerate(objs):
        best = 1e9
        for j, p in enumerate(objs):
            if i == j:
                continue
            dd = ((o["position"][0] - p["position"][0]) ** 2 +
                  (o["position"][1] - p["position"][1]) ** 2) ** 0.5
            best = min(best, dd)
        if best < 1e8:
            nn.append(best)

    # narrowest lateral gap inside each aisle strip (1D per y-row)
    min_lane = None
    for s in segs:
        if s["region"] != "aisle":
            continue
        x0, y0, x1, y1 = s["bbox"]
        inside = []
        for o in objs:
            ix = int((o["position"][0] - ox) / res)
            iy = int((o["position"][1] - oy) / res)
            ix = max(0, min(nx - 1, ix)); iy = max(0, min(ny - 1, iy))
            if s["mask"][iy, ix]:
                inside.append(o)
        if not inside:
            continue
        ys, xs = np.nonzero(s["mask"])
        rows = {}
        for iy in np.unique(ys):
            rows[iy] = [ox + xs[ys == iy].min() * res, ox + (xs[ys == iy].max() + 1) * res]
        worst = None
        for iy, (rx0, rx1) in rows.items():
            if (rx1 - rx0) < 1.0:
                continue      # sliver rows of the eroded mask are not real passages
            wy = oy + (iy + 0.5) * res
            blocks = []
            for o in inside:
                h = (o["size"][1] if abs(np.cos(o["yaw_rad"])) >= 0.5 else o["size"][0]) / 2.0
                if abs(o["position"][1] - wy) <= h:
                    w = (o["size"][0] if abs(np.cos(o["yaw_rad"])) >= 0.5 else o["size"][1]) / 2.0
                    blocks.append((o["position"][0] - w, o["position"][0] + w))
            blocks.sort()
            gaps = []; cur = rx0
            for (b0, b1) in blocks:
                if b0 > cur:
                    gaps.append(b0 - cur)
                cur = max(cur, b1)
            gaps.append(rx1 - cur)
            g = max(gaps) if gaps else (rx1 - rx0)
            worst = g if worst is None else min(worst, g)
        if worst is not None:
            min_lane = worst if min_lane is None else min(min_lane, worst)
    return {"count": len(objs), "coverage_pct": 100.0 * occ / len(segs),
            "max_per_segment": mx, "top1_pct": 100.0 * mx / max(1, len(objs)),
            "empty_area_pct": 100.0 * empty,
            "nn_median_m": round(float(np.median(nn)), 2) if nn else None,
            "per_region": dict(Counter(o["region"] for o in objs)),
            "n_segments_used": occ, "min_aisle_lane_m": (round(min_lane, 2) if min_lane else None)}


gen = SceneGenerator.from_config()
md = gen.md
v15cfg = load_v15()
segs = make_segments(md)
placeable_area = float(md["placeable"].sum()) * md["res"] ** 2
print("segments: %d (aisle %d / edge %d / open %d), placeable %.0f m2" % (
    len(segs), sum(1 for s in segs if s["region"] == "aisle"),
    sum(1 for s in segs if s["region"] == "shelf_edge"),
    sum(1 for s in segs if s["region"] == "open"), placeable_area))

COMPARE = [(1001, "medium"), (1001, "hard"), (1002, "medium"), (1002, "hard")]
report = {"segments": len(segs), "placeable_area_m2": round(placeable_area, 1), "scenes": []}

for seed, dens in COMPARE:
    # ---- v1.4.0 baseline ----
    sp14 = gen.generate(seed=seed, density=dens)
    p14 = os.path.join(OUT, "%04d_%s_v14.json" % (seed, dens))
    gen.save(sp14, p14)
    v14 = gen.generate(seed=seed, density=dens)
    m14 = metrics(v14["objects"], md, segs, placeable_area)
    val14 = v14["validation"]["valid"]

    # ---- v1.5.0-exp candidate ----
    from generator.seed_manager import layout_rng
    rng = layout_rng(seed)
    objs, st = sampler_v2.sample_layout(rng, md, gen.cfgs["obstacles"], v15cfg,
                                        gen.cfgs["asset_registry"], density=dens)
    spec15 = {"schema_version": "1.5-exp", "scene_id": "warehouse_%06d_%s_v15exp" % (seed, dens),
              "seed": seed, "density": dens, "objects": objs, "object_count": len(objs),
              "counts_by_class": {c: sum(1 for o in objs if o["class"] == c) for c in sampler_v2.CLASS_ORDER},
              "generation_stats": st,
              "layout_version": v15cfg["layout_version"]}
    from generator.placement_validator import validate
    spec15["validation"] = validate(spec15, md, md)
    from generator.scene_generator import layout_hash as lh
    spec15["layout_hash"] = lh(objs)
    p15 = os.path.join(OUT, "%04d_%s_v15exp.json" % (seed, dens))
    gen.save(spec15, p15)
    m15 = metrics(objs, md, segs, placeable_area)

    report["scenes"].append({
        "seed": seed, "density": dens,
        "v14": {**m14, "valid": val14, "hash": v14["layout_hash"][:12], "json": os.path.relpath(p14, ROOT)},
        "v15": {**m15, "valid": spec15["validation"]["valid"],
                "hash": spec15["layout_hash"][:12], "json": os.path.relpath(p15, ROOT),
                "target": st["target_total"], "failed": len(st["failed"]),
                "clustered_boxes": st["clustered_boxes"]},
    })
    print("%d %-6s v14: n=%2d cov=%5.1f%% maxSeg=%d empty=%5.1f%% lane=%s | "
          "v15: n=%2d cov=%5.1f%% maxSeg=%d empty=%5.1f%% lane=%s"
          % (seed, dens, m14["count"], m14["coverage_pct"], m14["max_per_segment"],
             m14["empty_area_pct"], m14["min_aisle_lane_m"],
             m15["count"], m15["coverage_pct"], m15["max_per_segment"],
             m15["empty_area_pct"], m15["min_aisle_lane_m"]))

with open(os.path.join(OUT, "comparison.json"), "w") as f:
    json.dump(report, f, indent=2, sort_keys=True)
print("\nwrote", os.path.join(OUT, "comparison.json"))
print("EXPERIMENT_DONE")
