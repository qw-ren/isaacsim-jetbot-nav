#!/usr/bin/env python3
"""v1.6.0 follow-up checks.

 Q1  B-class gap: one-sided or two-sided? how big? is it passable for the
     REAL 3x-scaled JetBot (0.491 x 0.400 m)?
 Q2  batch statistics: histogram of blocked aisles per density
 Q3  global alternative verified AFTER all A and B objects are placed
"""
import json
import os
import sys
from collections import Counter

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from generator import config, scene_mode, blockage  # noqa
from generator.scene_generator import SceneGenerator  # noqa
from generator.topology import Topology  # noqa
from generator.seed_manager import layout_rng  # noqa

OUT = os.path.join(ROOT, "outputs", "v16_formal")
os.makedirs(OUT, exist_ok=True)

cfgs = config.load_all()
cfgs["difficulty_v15"] = yaml.safe_load(open(os.path.join(
    ROOT, "config", "difficulty_v15_experiment.yaml")))
gen = SceneGenerator.from_config()
md = gen.md

# --- real 3x JetBot (measured from the official asset) ---
JETBOT = {"size_x": 0.4908, "size_y": 0.4002, "size_z": 0.7798,
          "half_width": 0.2001, "half_diagonal": 0.3166}

cfg_r = float(cfgs["warehouse"]["clearance"]["robot_radius_m"])
cfg_m = float(cfgs["warehouse"]["clearance"]["safety_margin_m"])

# a topology that uses the REAL robot radius (+ margin)
wh_real = json.loads(json.dumps(cfgs["warehouse"]))
wh_real["clearance"]["robot_radius_m"] = round(JETBOT["half_diagonal"], 4)

res = {"robot": {"config_radius_m": cfg_r, "config_margin_m": cfg_m,
                 "jetbot_scaled": JETBOT,
                 "jetbot_required_radius_m": round(JETBOT["half_diagonal"], 4),
                 "config_inflation_m": round(cfg_r + cfg_m, 4),
                 "jetbot_inflation_m": round(JETBOT["half_diagonal"] + cfg_m, 4)},
       "gaps": {}, "blocked_hist": {}, "global_after_all": {}, "real_robot": {}}

for density in ("medium", "hard"):
    gaps, hist = [], Counter()
    glob_ok = glob_bad = 0
    for seed in range(1, 101):
        spec = scene_mode.build_mixed(md, cfgs, seed, density=density)
        bl = spec["blockages"][0]
        gaps.append({"seed": seed, "span": bl["span_m"], "covered": bl["covered_m"],
                     "left": bl["left_gap_m"], "right": bl["right_gap_m"],
                     "pieces": bl["pieces"]})
        for a in spec["blocked_aisles"]:
            hist[a] += 1
        if spec["validation"].get("B_global_after_A"):
            glob_ok += 1
        else:
            glob_bad += 1
    res["gaps"][density] = {
        "max_left_gap": max(g["left"] for g in gaps),
        "max_right_gap": max(g["right"] for g in gaps),
        "min_right_gap": min(g["right"] for g in gaps),
        "rows_with_two_sided_gap": sum(1 for g in gaps if g["left"] > 1e-6 and g["right"] > 1e-6),
        "detail_sample": gaps[:3],
    }
    res["blocked_hist"][density] = dict(hist)
    res["global_after_all"][density] = {"ok": glob_ok, "fail": glob_bad}

# --- real-robot passability of the blocked cross-section (subset) ---
topo_real = Topology(md, wh_real)
base_real = topo_real.grid()
real = {"aisles_before": {}, "blocked_after": {}, "others_after": {}}
for density in ("medium",):
    for seed in (1, 2, 3, 1001):
        spec = scene_mode.build_mixed(md, cfgs, seed, density=density)
        objs = spec["objects"]
        g_after = topo_real.grid(objs)
        blocked = spec["blocked_aisles"]
        for seg in topo_real.aisle_segments():
            sid = seg["index"]
            before, _ = topo_real.aisle_traversable(seg, base_real)
            after, _ = topo_real.aisle_traversable(seg, g_after)
            real["aisles_before"].setdefault(sid, before)
            if sid in blocked:
                real["blocked_after"].setdefault(seed, {})[sid] = after
            else:
                real["others_after"].setdefault(seed, {})[sid] = after

res["real_robot"] = {
    "radius_used_m": wh_real["clearance"]["robot_radius_m"],
    "aisles_passable_before": real["aisles_before"],
    "blocked_aisle_passable_after": real["blocked_after"],
    "other_aisles_passable_after": real["others_after"],
}

with open(os.path.join(OUT, "followup_checks.json"), "w") as f:
    json.dump(res, f, indent=2, sort_keys=True)

print("robot:", res["robot"])
for d in ("medium", "hard"):
    g = res["gaps"][d]
    print("%-6s gap: left_max=%.2f right=%.2f..%.2f two_sided=%d" %
          (d, g["max_left_gap"], g["min_right_gap"], g["max_right_gap"],
           g["rows_with_two_sided_gap"]))
    print("        blocked hist:", res["blocked_hist"][d],
          " global_after_all:", res["global_after_all"][d])
print("real robot:", json.dumps(res["real_robot"])[:600])
print("FOLLOWUP_DONE")
