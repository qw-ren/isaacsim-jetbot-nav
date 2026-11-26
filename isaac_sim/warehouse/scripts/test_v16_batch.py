#!/usr/bin/env python3
"""v1.6.0 acceptance: 100-seed batch for Medium and Hard (pure Python).

Per seed: generate the official A+B mixed scene and check
  * A count inside the configured tier range
  * B present: exactly 1 sealed aisle with a global alternative
  * legality (A clearance rules, B seal rules, A/B no overlap)
  * connectivity: every non-blocked aisle stays end-to-end passable
  * no duplicate ids, no id clash between A and B
"""
import json
import os
import sys
import traceback
from collections import Counter

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from generator import config, scene_mode  # noqa
from generator.scene_generator import SceneGenerator  # noqa

OUT = os.path.join(ROOT, "outputs", "v16_formal")
os.makedirs(OUT, exist_ok=True)

cfgs = config.load_all()
cfgs["difficulty_v15"] = yaml.safe_load(open(os.path.join(
    ROOT, "config", "difficulty_v15_experiment.yaml")))
gen = SceneGenerator.from_config()

TIERS = {"medium": (18, 24), "hard": (28, 40)}


def check(spec, density):
    objs = spec["objects"]
    a = [o for o in objs if o.get("kind") == "A"]
    b = [o for o in objs if o.get("kind") == "B"]
    lo, hi = TIERS[density]
    issues = []
    if not (lo <= len(a) <= hi):
        issues.append("A count %d outside %d..%d" % (len(a), lo, hi))
    if len(b) == 0:
        issues.append("no B objects")
    if len(spec.get("blockages", [])) != 1:
        issues.append("blockages=%d" % len(spec.get("blockages", [])))
    if not spec["validation"].get("B_global_after_A", True):
        issues.append("no global alternative after all objects")
    if not spec["validation"]["valid"]:
        issues.append("validation invalid")
    blocked = set(spec.get("blocked_aisles", []))
    for si, st in spec["aisle_state"].items():
        if si in blocked:
            if st["passable"]:
                issues.append("blocked aisle %s still passable" % si)
        else:
            if st.get("was_passable", True) and not st["passable"]:
                issues.append("open aisle %s became impassable" % si)
    ids = [o["id"] for o in objs]
    if len(ids) != len(set(ids)):
        issues.append("duplicate ids")
    if any(i.startswith("blockage_") for i in ids) and len(b) != len(set(
            o["id"] for o in b)):
        issues.append("B id clash")
    for bl in spec.get("blockages", []):
        v = bl["verify"]
        if not (v["before_passable"] and not v["after_local_passable"]
                and v["global_alternative_exists"] and v["legal"]):
            issues.append("blockage verify failed")
    return {"a": len(a), "b": len(b), "total": len(objs),
            "removed": len(spec.get("a_stats", {}).get("connectivity_removed", [])),
            "gap": spec["blockages"][0]["right_gap_m"] if spec.get("blockages") else None,
            "blocked": sorted(blocked), "issues": issues, "ok": not issues}


summary = {"layout_version": scene_mode.LAYOUT_VERSION,
           "aisle_pref": scene_mode.AISLE_PREF, "tiers": TIERS, "runs": {}}

for density in ("medium", "hard"):
    ok = 0
    fails = []
    stats = Counter()
    per_seed = []
    for seed in range(1, 101):
        try:
            spec = scene_mode.build_mixed(gen.md, cfgs, seed, density=density)
            r = check(spec, density)
        except Exception as e:
            r = {"ok": False, "issues": ["exception: %s" % e], "a": 0, "b": 0, "total": 0,
                 "blocked": []}
            fails.append({"seed": seed, "error": traceback.format_exc(limit=2)})
        if r["ok"]:
            ok += 1
        else:
            fails.append({"seed": seed, "issues": r["issues"]})
        stats["a_total"] += r["a"]
        stats["b_total"] += r["b"]
        for b in r["blocked"]:
            stats["aisle_%d" % b] += 1
        per_seed.append({"seed": seed, "a": r["a"], "b": r["b"], "blocked": r["blocked"],
                         "ok": r["ok"], "removed": r.get("removed", 0), "gap": r.get("gap")})
        stats["removed_total"] += r.get("removed", 0)
        stats["gap_max"] = max(stats["gap_max"], r.get("gap") or 0.0)
    summary["runs"][density] = {
        "success": ok, "total": 100,
        "a_mean": round(stats["a_total"] / 100.0, 2),
        "a_min": min(s["a"] for s in per_seed), "a_max": max(s["a"] for s in per_seed),
        "removed_total": stats["removed_total"], "gap_max": round(stats["gap_max"], 3),
        "b_mean": round(stats["b_total"] / 100.0, 2),
        "blocked_aisle_histogram": {k: v for k, v in stats.items()
                                    if k.startswith("aisle_")},
        "failures": fails[:10], "per_seed": per_seed,
    }
    print("%-6s success %d/100  A mean %.1f  B mean %.1f  blocked aisle hist %s"
          % (density, ok, stats["a_total"] / 100.0, stats["b_total"] / 100.0,
             {k: v for k, v in stats.items() if k.startswith("aisle_")}))
    if fails:
        print("   first failures:", json.dumps(fails[:3])[:400])

with open(os.path.join(OUT, "batch_100.json"), "w") as f:
    json.dump(summary, f, indent=2, sort_keys=True)
print("wrote", os.path.join(OUT, "batch_100.json"))
print("BATCH_DONE")
