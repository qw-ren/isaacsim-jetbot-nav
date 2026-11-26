#!/usr/bin/env python3
"""B-class: deliberate full-cross-section aisle blockage (experimental).

A B-class blockage is a ROW of official cartons across one aisle, sealed edge to
edge, at one station. It is NOT "more A-class objects": the mechanism is
topological (the aisle's two ends stop being connected through that aisle).

Verification is programmatic, on the robot-inflated grid:
  1. before  : the aisle is passable end-to-end
  2. after   : it is NOT passable end-to-end any more (local disconnection)
  3. global  : the two mouths are STILL connected through the rest of the
               warehouse (an alternative route exists)
  4. legal   : footprints inside free space, no overlap, outside protected_zones
"""
from __future__ import annotations

import numpy as np

from . import placement_validator as pv
from .seed_manager import InstanceCounter

TAG = "blockage"
EPS = 1e-4          # touching pieces are legal; only a real overlap is illegal


def _overlap(a, b, eps=EPS):
    """True only if the intersection has positive area beyond float noise."""
    dx = min(a[2], b[2]) - max(a[0], b[0])
    dy = min(a[3], b[3]) - max(a[1], b[1])
    return dx > eps and dy > eps


def _variants(entry):
    v = entry.get("variants")
    if v:
        return list(v)
    strat = entry.get("strategies") or {}
    key = entry.get("strategy")
    if key and key in strat:
        return list(strat[key].get("units") or [])
    return []


def candidate_aisles(topo, pass_grid):
    """Aisles that are through-passable and have a global alternative."""
    out = []
    for seg in topo.aisle_segments():
        ok, _ = topo.aisle_traversable(seg, pass_grid)
        if not ok:
            continue
        pts = topo.mouth_points(seg)
        if len(pts) != 2:
            continue
        if not topo.global_connected(pts[0], pts[1], pass_grid):
            continue
        out.append(seg)
    return out


def choose_aisles(rng, topo, pass_grid, n_block, prefer_width_m=0.0):
    """Deterministically pick n_block distinct aisles from the candidates."""
    cand = candidate_aisles(topo, pass_grid)
    if not cand:
        return []
    if prefer_width_m:
        cand = sorted(cand, key=lambda s: (-s["width_m"], s["index"]))
    else:
        cand = sorted(cand, key=lambda s: s["index"])
    if n_block >= len(cand):
        return cand
    idx = rng.permutation(len(cand))[:n_block]
    picked = sorted(int(i) for i in idx)
    return [cand[i] for i in picked]


def _safe_span(topo, iy, depth_m, x_hint):
    """Free-x run that stays free across ALL rows the object's depth covers."""
    half = int(np.ceil(depth_m / topo.res / 2.0))
    free = topo.free_mask()
    lo = max(0, iy - half); hi = min(topo.ny - 1, iy + half)
    best = None
    for r in range(lo, hi + 1):
        row = free[r]
        ix = int((x_hint - topo.ox) / topo.res)
        ix = max(0, min(topo.nx - 1, ix))
        if not row[ix]:
            return None
        a = ix
        while a > 0 and row[a - 1]:
            a -= 1
        b = ix
        while b < topo.nx - 1 and row[b + 1]:
            b += 1
        run = (topo.ox + a * topo.res, topo.ox + (b + 1) * topo.res)
        if best is None:
            best = run
        else:
            best = (max(best[0], run[0]), min(best[1], run[1]))
        if best[1] - best[0] <= 0.05:
            return None
    return best


def _row_plans(width, pieces, k=8):
    """Candidate rows, best (smallest leftover) first.

    Several candidates are kept on purpose: the tightest row has almost no slack
    to dodge a rack post at the aisle edge, so a slightly looser row that can be
    shifted sideways may be the one that actually fits.
    """
    cands = []
    maxs = [int(width // w) for (_, w, _) in pieces]
    for a in range(maxs[0] + 1):
        for b in range(maxs[1] + 1):
            s = a * pieces[0][1] + b * pieces[1][1]
            if s > width + 1e-9:
                continue
            cands.append(((round(width - s, 6), -(a + b), -a), a, b, s))
    cands.sort(key=lambda t: t[0])
    out = []
    for _, a, b, s in cands[:k]:
        if a == 0 and b == 0:
            continue
        plan = []
        if a:
            plan.append((pieces[0][0], a))
        if b:
            plan.append((pieces[1][0], b))
        out.append((plan, s))
    return out


def build_blockage(rng, topo, seg, registry, obstacles_cfg, protected_zones=None,
                   counter=None, cat_cfg=None):
    """Place one sealed carton row across `seg`. Returns (objects, info)."""
    cat_cfg = cat_cfg or {}
    C = obstacles_cfg["classes"]
    variants = _variants(C["box"])
    # only cartons that are tall enough (>= 0.5 m) and wide enough to matter
    wide = []
    for a in variants:
        dims = registry["assets"][a]["dims_m"]
        if float(dims[2]) >= 0.45 and float(dims[0]) >= 0.45:
            wide.append((a, float(dims[0]), float(dims[1])))
    if not wide:
        return [], {"ok": False, "reason": "no suitable carton"}
    wide.sort(key=lambda t: -t[1])
    pieces = wide[:2]
    if len(pieces) < 2:
        pieces.append(pieces[0])

    y0, y1 = seg["row_range"]
    lo = int(y0 + 0.20 * (y1 - y0))
    hi = int(y0 + 0.80 * (y1 - y0))
    cx_hint = (seg["bbox"][0] + seg["bbox"][2]) / 2.0
    counter = counter or InstanceCounter()
    proto = protected_zones or []

    # deterministic sweep of candidate stations (random start, fixed order)
    stations = list(range(lo, max(lo + 1, hi), 4))
    if not stations:
        return [], {"ok": False, "reason": "no station"}
    start = int(rng.integers(0, len(stations)))
    order = stations[start:] + stations[:start]

    diag = []
    for iy in order:
        span = _safe_span(topo, iy, pieces[0][2], cx_hint)
        if span is None:
            diag.append({'row': iy, 'reason': 'no safe span'})
            continue
        left, right = span
        width = right - left
        plans = _row_plans(width, pieces)
        if not plans:
            diag.append({'row': iy, 'reason': 'no row plan'})
            continue
        cy = topo.oy + (iy + 0.5) * topo.res
        objs, bad = None, None
        for plan, covered in plans:
            seq = []
            for aid_, cnt in plan:
                dims = registry["assets"][aid_]["dims_m"]
                for _ in range(cnt):
                    seq.append((aid_, float(dims[0]), float(dims[1]), float(dims[2])))
            slack = width - covered       # leftover, always kept on ONE side
            if slack > 0.45:
                continue                  # too loose to be a reliable seal
            for shift in (0.0, slack * 0.5, slack):
                if shift > slack + 1e-6:
                    break
                trial, why = [], None
                cursor = left + shift
                for (aid_, w_, d_, h_) in seq:
                    cx = cursor + w_ / 2.0
                    cursor += w_
                    obj = {"id": "tmp", "class": TAG, "kind": "B", "asset_key": aid_,
                           "region": "aisle", "segment": seg["index"],
                           "blockage_aisle": seg["index"],
                           "blockage_station_y": round(cy, 3),
                           "position": [round(float(cx), 3), round(cy, 3), 0.0],
                           "yaw_rad": 0.0, "size": [w_, d_, h_]}
                    x0, y0f, x1, y1f = pv._aabb(obj)
                    ix0, iy0, ix1, iy1 = pv.rect_cells(x0, y0f, x1, y1f, (topo.ox, topo.oy),
                                                       topo.res, topo.nx, topo.ny)
                    if pv.rect_sum(topo._int_free(), ix0, iy0, ix1, iy1) != \
                            (ix1 - ix0 + 1) * (iy1 - iy0 + 1):
                        why = "outside free space"
                        break
                    if any(_overlap(pv._aabb(obj), pv._aabb(o)) for o in trial):
                        why = "overlap"
                        break
                    for (px0, py0, px1, py1) in proto:
                        if not (x1 <= px0 or px1 <= x0 or y1f <= py0 or py1 <= y0f):
                            why = "protected zone"
                            break
                    if why:
                        break
                    trial.append(obj)
                if why is None and len(trial) == len(seq):
                    objs, bad = trial, None
                    best_plan, best_cov, best_slack = plan, covered, slack
                    break
                bad = why
            if objs:
                break
        if bad or not objs:
            diag.append({'row': iy, 'span': [round(left, 2), round(right, 2)],
                         'reason': bad})
            continue
        for o in objs:
            o["id"] = counter.next(TAG)
        info = {"ok": True, "aisle": seg["index"], "station_y": round(cy, 3),
                "span_m": round(width, 2), "covered_m": round(best_cov, 2),
                "left_gap_m": 0.0, "right_gap_m": round(best_slack, 2),
                "pieces": len(objs),
                "asset": best_plan[0][0], "assets": [a for a, _ in best_plan],
                "piece_size": [round(max(s[1] for s in seq), 3),
                               round(max(s[2] for s in seq), 3),
                               round(max(s[3] for s in seq), 3)]}
        return objs, info
    return [], {"ok": False, "reason": "no station fits a sealed row", "diag": diag[:8]}


def verify_blockage(topo, seg, blockage_objects, all_objects, protected_zones=None):
    """Programmatic before/after/local/global legality check."""
    base = topo.grid()
    with_b = topo.grid(all_objects or blockage_objects)
    before, _ = topo.aisle_traversable(seg, base)
    after, _ = topo.aisle_traversable(seg, with_b)
    pts = topo.mouth_points(seg)
    glob = topo.global_connected(pts[0], pts[1], with_b) if len(pts) == 2 else None

    legal = True
    why = []
    proto = protected_zones or []
    for o in blockage_objects:
        x0, y0, x1, y1 = pv._aabb(o)
        ix0, iy0, ix1, iy1 = pv.rect_cells(x0, y0, x1, y1, (topo.ox, topo.oy),
                                           topo.res, topo.nx, topo.ny)
        if pv.rect_sum(topo._int_free(), ix0, iy0, ix1, iy1) != (ix1 - ix0 + 1) * (iy1 - iy0 + 1):
            legal = False; why.append("%s outside free" % o["id"])
        for (px0, py0, px1, py1) in proto:
            if not (x1 <= px0 or px1 <= x0 or y1 <= py0 or py1 <= y0):
                legal = False; why.append("%s in protected" % o["id"])
    for i in range(len(blockage_objects)):
        for j in range(i + 1, len(blockage_objects)):
            if _overlap(pv._aabb(blockage_objects[i]),
                             pv._aabb(blockage_objects[j])):
                legal = False; why.append("overlap %s/%s" % (blockage_objects[i]["id"],
                                                             blockage_objects[j]["id"]))
    return {"aisle": seg["index"], "before_passable": bool(before),
            "after_local_passable": bool(after), "global_alternative_exists": glob,
            "legal": legal, "problems": why,
            "ok": bool(before) and (not bool(after)) and bool(glob) and legal}
