#!/usr/bin/env python3
"""EXPERIMENTAL layout sampler v1.5 鈥?segment-aware, coverage-balanced.

Pipeline (one deterministic layout RNG):
 1. SEGMENTS from the real map: aisle strips, shelf-edge strips, 6x6 m open cells.
 2. DENSITY: per-scene total (difficulty_v15_experiment.yaml) split into classes.
 3. ALLOCATION: per class, proportional to area x class-region multiplier, with a
    per-segment capacity (area/45 .. 3 for aisles, ..2 for edges/open).
 4. PLACEMENT inside the segment with the SAME geometric checks as v1.4.0, plus:
      * AISLE LANE GUARD: after a hypothetical placement, at every y-row the
        object spans, the widest remaining lateral gap must stay >= min_lane_width
        (0.90 m). This prevents blocking a rack aisle, and is checked against the
        REAL footprint (not just the centre point).
      * CLUSTERING: ~35% of boxes are pulled to a 0.6-1.3 m ring around an
        already placed pallet (pallet + nearby cartons), if geometry allows.
This module is experimental; the accepted sampler is untouched.
"""
from __future__ import annotations

import numpy as np
from scipy import ndimage

from .seed_manager import InstanceCounter
from . import placement_validator as pv

CLASS_ORDER = ["box", "pallet", "barrier", "cart"]
RK = {"aisle": "aisle", "shelf_edge": "edge", "open": "open"}


# ----------------------------------------------------------------- segments
def build_segments(md, cfg, min_area_m2=8.0):
    res = md["res"]; ox, oy = md["origin"]; A = res * res
    segs = []

    def add(region, mask, cells, bbox):
        area = cells * A
        if area < min_area_m2:
            return
        key = RK[region]
        per = cfg["capacity"]["%s_area_per_object_m2" % key]
        cap = max(1, int(round(area / per)))
        cap = min(cap, int(cfg["capacity"]["max_per_segment"][key]))
        # per-row lateral extents (world x) for the lane guard
        rows = {}
        ys, xs = np.nonzero(mask)
        for iy in np.unique(ys):
            sel = xs[ys == iy]
            rows[int(iy)] = (ox + sel.min() * res, ox + (sel.max() + 1) * res)
        segs.append({"region": region, "mask": mask, "area_m2": area, "bbox": bbox,
                     "cap": cap, "rows": rows})

    for region, key in (("aisle", "region_aisle"), ("shelf_edge", "region_edge")):
        m = md[key]
        lab, n = ndimage.label(m)
        for i in range(1, n + 1):
            sub = (lab == i); c = int(sub.sum())
            if c == 0:
                continue
            ys, xs = np.nonzero(sub)
            add(region, sub, c, [ox + xs.min() * res, oy + ys.min() * res,
                                 ox + (xs.max() + 1) * res, oy + (ys.max() + 1) * res])

    gs = int(round(6.0 / res)); m = md["region_open"]; ny, nx = m.shape
    for gy in range(0, ny, gs):
        for gx in range(0, nx, gs):
            sub = m[gy:gy + gs, gx:gx + gs]; c = int(sub.sum())
            if c < int(0.35 * gs * gs):
                continue
            full = np.zeros_like(m); full[gy:gy + gs, gx:gx + gs] = sub
            add("open", full, c, [ox + gx * res, oy + gy * res,
                                  ox + (gx + gs) * res, oy + (gy + gs) * res])
    return segs


# ----------------------------------------------------------------- helpers
def _largest_remainder(total, weights, caps):
    n = len(weights)
    if total <= 0 or n == 0:
        return [0] * n
    W = sum(weights)
    if W <= 0:
        return [0] * n
    raw = [total * w / W for w in weights]
    alloc = [min(int(np.floor(raw[i])), caps[i]) for i in range(n)]
    left = total - sum(alloc)
    order = sorted(range(n), key=lambda i: (-(raw[i] - np.floor(raw[i])), i))
    while left > 0:
        moved = False
        for i in order:
            if left <= 0:
                break
            if alloc[i] < caps[i]:
                alloc[i] += 1; left -= 1; moved = True
        if not moved:
            break
    return alloc


def _lane_ok(md, seg, seg_objs, cand, min_lane):
    """Widest free lateral gap at every y-row the candidate spans must stay >= min_lane."""
    if seg["region"] != "aisle" or not seg["rows"]:
        return True
    res = md["res"]; oy = md["origin"][1]
    cx0, cy0, cx1, cy1 = pv._aabb(cand)
    iy0 = int(np.floor((cy0 - oy) / res))
    iy1 = int(np.ceil((cy1 - oy) / res))
    rows = [(iy, seg["rows"][iy]) for iy in range(iy0, iy1 + 1) if iy in seg["rows"]]
    if not rows:
        # never skip the guard: fall back to the row under the object's centre
        iyc = int(np.floor(((cy0 + cy1) / 2 - oy) / res))
        candidates = sorted(seg["rows"].items(), key=lambda kv: abs(kv[0] - iyc))
        if not candidates:
            return True
        rows = [candidates[0]]
    for iy, (rx0, rx1) in rows:
        wy = oy + (iy + 0.5) * res
        blocks = []
        for o in seg_objs:
            ox0, oy0, ox1, oy1 = pv._aabb(o)
            if oy0 - res <= wy <= oy1 + res:
                blocks.append((ox0, ox1))
        blocks.append((cx0, cx1))
        blocks.sort()
        gaps = []; cur = rx0
        for (b0, b1) in blocks:
            if b0 > cur:
                gaps.append(b0 - cur)
            cur = max(cur, b1)
        gaps.append(rx1 - cur)
        if (max(gaps) if gaps else (rx1 - rx0)) < min_lane:
            return False
    return True


def _resolve_variants(entry):
    v = entry.get("variants")
    if v:
        return list(v)
    strat = entry.get("strategies") or {}
    key = entry.get("strategy")
    if key and key in strat:
        return list(strat[key].get("units") or [])
    return []


def _fits(obj, md):
    x0, y0, x1, y1 = pv._aabb(obj)
    ix0, iy0, ix1, iy1 = pv.rect_cells(x0, y0, x1, y1, md["origin"], md["res"], md["nx"], md["ny"])
    ncell = (ix1 - ix0 + 1) * (iy1 - iy0 + 1)
    return pv.rect_sum(md["int_placeable"], ix0, iy0, ix1, iy1) == ncell


def _too_close(obj, placed, clear):
    a = pv._aabb(obj, clear / 2)
    for o in placed:
        if pv._intersect(a, pv._aabb(o, clear / 2)):
            return True
    return False


def _hits_protected(obj, zones):
    x0, y0, x1, y1 = pv._aabb(obj)
    for (px0, py0, px1, py1) in zones:
        if not (x1 <= px0 or px1 <= x0 or y1 <= py0 or py1 <= y0):
            return True
    return False


# ----------------------------------------------------------------- main
def sample_layout(rng, md, obstacles_cfg, diff_v15, registry, density="medium",
                  protected_zones=None, preplaced=None, aisle_pref=None, counter=None):
    C = obstacles_cfg["classes"]
    clear = float(obstacles_cfg["clearance"]["obstacle"])
    max_attempts = int(diff_v15["limits"]["max_placement_attempts_per_object"])
    min_lane = float(diff_v15["aisle_lane"]["min_lane_width_m"])
    res = md["res"]; ox, oy = md["origin"]
    proto = protected_zones or []

    segs = build_segments(md, diff_v15)
    if not segs:
        raise RuntimeError("no placement segments")
    areas = np.array([s["area_m2"] for s in segs])
    caps = [s["cap"] for s in segs]
    cell_lists = []
    for s in segs:
        ys, xs = np.nonzero(s["mask"])
        cell_lists.append((xs, ys))

    lo, hi = diff_v15["totals"][density]
    total = int(rng.integers(lo, hi + 1))
    total = min(total, int(diff_v15["limits"]["max_objects_per_scene_hard_cap"]), sum(caps))

    cw = diff_v15["class_weights"]
    names = [c for c in CLASS_ORDER if c in C and cw.get(c, 0) > 0]
    wsum = sum(cw[c] for c in names)
    counts = {}
    acc = 0
    for i, c in enumerate(names):
        if i == len(names) - 1:
            counts[c] = total - acc
        else:
            counts[c] = int(round(total * cw[c] / wsum))
            acc += counts[c]

    counter = counter or InstanceCounter()
    objs = list(preplaced or [])
    new_objs = []
    seg_objs = [[] for _ in segs]
    remaining = caps[:]
    # objects already in the scene (e.g. B-class blockage) occupy segment capacity
    for o in objs:
        si = o.get("segment")
        if isinstance(si, int) and 0 <= si < len(segs):
            seg_objs[si].append(o)
            remaining[si] = max(0, remaining[si] - 1)
    stats = {"target_total": total, "requested": dict(counts), "placed": {},
             "by_region": {"aisle": 0, "shelf_edge": 0, "open": 0},
             "by_segment": [0] * len(segs), "failed": [], "attempts": 0,
             "clustered_boxes": 0, "lane_rejections": 0}

    # placement order: pallets first so boxes can cluster around them
    PLACE_ORDER = ["pallet", "box", "barrier", "cart"]
    for cls in [c for c in PLACE_ORDER if c in names]:
        variants = _resolve_variants(C[cls])
        if not variants:
            continue
        yaws = [float(np.deg2rad(y)) for y in (C[cls].get("yaw_choices") or [0])]
        mult = dict(diff_v15["region_multiplier"][cls])
        if aisle_pref:
            mult["aisle"] = mult["aisle"] * float(aisle_pref) / 2.0
        weights = [areas[i] * mult[RK[segs[i]["region"]]] * (remaining[i] / max(1, caps[i]))
                   for i in range(len(segs))]
        alloc = _largest_remainder(counts[cls], weights, remaining)
        stats["placed"][cls] = 0

        for si in sorted(range(len(segs)), key=lambda i: (-alloc[i], i)):
            xs, ys = cell_lists[si]
            if len(xs) == 0:
                continue
            for _k in range(alloc[si]):
                anchor = None
                if cls == "box" and objs:
                    if rng.random() < float(diff_v15["clustering"]["box_near_pallet_prob"]):
                        pals = [o for o in objs
                                if o["class"] == "pallet" and o.get("segment") == si]
                        if pals:
                            anchor = pals[int(rng.integers(0, len(pals)))]
                done = False
                for _a in range(max_attempts):
                    if anchor is not None and _a >= 8:
                        anchor = None      # give up clustering, fall back to uniform
                    stats["attempts"] += 1
                    vidx = int(rng.integers(0, len(variants)))
                    yidx = int(rng.integers(0, len(yaws)))
                    ci = int(rng.integers(0, len(xs)))
                    cx = ox + (int(xs[ci]) + 0.5) * res
                    cy = oy + (int(ys[ci]) + 0.5) * res
                    if anchor is not None:
                        d0, d1 = diff_v15["clustering"]["distance_m"]
                        dd = ((cx - anchor["position"][0]) ** 2 +
                              (cy - anchor["position"][1]) ** 2) ** 0.5
                        if not (float(d0) <= dd <= float(d1)):
                            continue
                    aid = variants[vidx]
                    dims = registry["assets"][aid]["dims_m"]
                    obj = {"id": counter.next(cls), "class": cls, "asset_key": aid,
                           "region": segs[si]["region"], "segment": si,
                           "position": [round(cx, 3), round(cy, 3), 0.0],
                           "yaw_rad": round(yaws[yidx], 6),
                           "size": [float(dims[0]), float(dims[1]), float(dims[2])]}
                    if not _fits(obj, md):
                        continue
                    if _too_close(obj, objs, clear):
                        continue
                    if _hits_protected(obj, proto):
                        continue
                    if segs[si]["region"] == "aisle" and not _lane_ok(md, segs[si], seg_objs[si], obj, min_lane):
                        stats["lane_rejections"] += 1
                        continue
                    objs.append(obj)
                    new_objs.append(obj)
                    seg_objs[si].append(obj)
                    stats["placed"][cls] += 1
                    stats["by_region"][segs[si]["region"]] += 1
                    stats["by_segment"][si] += 1
                    remaining[si] -= 1
                    if anchor is not None:
                        stats["clustered_boxes"] += 1
                    done = True
                    break
                if not done:
                    stats["failed"].append({"class": cls, "segment": si,
                                            "reason": "attempts_exhausted"})
    return new_objs, stats
