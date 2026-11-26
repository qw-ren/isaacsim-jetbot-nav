#!/usr/bin/env python3
"""Verify the blocked aisle stays sealed for the REAL 3x-scaled JetBot.

Uses the measured JetBot footprint (0.4908 x 0.4002 m) as the connectivity
radius: half_diagonal 0.3166 + safety 0.12 = 0.4366 m inflation, i.e. a STRICTER
robot than the configured 0.30 + 0.12 = 0.42 m. If the aisle is still impassable
and the alternative still exists at this radius, the conclusion holds for the
real robot in any orientation.
"""
import json
import os
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from generator import config, scene_mode  # noqa
from generator.scene_generator import SceneGenerator  # noqa
from generator.topology import Topology  # noqa

OUT = os.path.join(ROOT, "outputs", "v16_formal")
os.makedirs(OUT, exist_ok=True)

JET = {"size_x": 0.4908, "size_y": 0.4002, "size_z": 0.7798,
       "half_width": 0.2001, "half_diagonal": 0.3166}

cfgs = config.load_all()
cfgs["difficulty_v15"] = yaml.safe_load(open(os.path.join(
    ROOT, "config", "difficulty_v15_experiment.yaml")))
gen = SceneGenerator.from_config()
md = gen.md

margin = float(cfgs["warehouse"]["clearance"]["safety_margin_m"])
cfg_r = float(cfgs["warehouse"]["clearance"]["robot_radius_m"])
wh_real = json.loads(json.dumps(cfgs["warehouse"]))
wh_real["clearance"]["robot_radius_m"] = JET["half_diagonal"]

cfg_topo = Topology(md, cfgs["warehouse"])
real_topo = Topology(md, wh_real)
base_cfg = cfg_topo.grid()
base_real = real_topo.grid()

rows = []
for seed, dens in ((1001, "medium"), (1001, "hard"), (1002, "medium"), (1003, "hard")):
    spec = scene_mode.build_mixed(md, cfgs, seed, density=dens)
    objs = spec["objects"]
    g_cfg = cfg_topo.grid(objs)
    g_real = real_topo.grid(objs)
    for bi in spec["blocked_aisles"]:
        seg_c = cfg_topo.aisle_segments()[bi]
        seg_r = real_topo.aisle_segments()[bi]
        before_r, _ = real_topo.aisle_traversable(seg_r, base_real)
        after_r, _ = real_topo.aisle_traversable(seg_r, g_real)
        pts_c = seg_c["mouth_points"] if "mouth_points" in seg_c else cfg_topo.mouth_points(seg_c)
        pts_r = real_topo.mouth_points(seg_r)
        glob_c = cfg_topo.global_connected(pts_c[0], pts_c[1], g_cfg)
        glob_r = real_topo.global_connected(pts_r[0], pts_r[1], g_real)
        rows.append({
            "seed": seed, "density": dens, "aisle": bi,
            "config_radius_m": cfg_r, "real_radius_m": round(JET["half_diagonal"], 4),
            "real_before_passable": bool(before_r),
            "real_after_impassable": (not bool(after_r)),
            "config_global_alternative": bool(glob_c),
            "real_global_alternative": bool(glob_r),
            "gap_m": spec["blockages"][0]["right_gap_m"],
            "jetbot_width_m": JET["size_y"],
            "gap_smaller_than_robot": bool(spec["blockages"][0]["right_gap_m"] < JET["size_y"]),
        })
    print("%04d %-6s aisle=%s gap=%.2f  real: before=%s after_blocked=%s global_alt=%s"
          % (seed, dens, spec["blocked_aisles"], rows[-1]["gap_m"],
             rows[-1]["real_before_passable"], rows[-1]["real_after_impassable"],
             rows[-1]["real_global_alternative"]))

res = {"jetbot": JET, "safety_margin_m": margin,
       "config_inflation_m": round(cfg_r + margin, 4),
       "real_inflation_m": round(JET["half_diagonal"] + margin, 4),
       "rows": rows,
       "all_ok": all(r["real_before_passable"] and r["real_after_impassable"]
                     and r["real_global_alternative"] and r["gap_smaller_than_robot"]
                     for r in rows)}
with open(os.path.join(OUT, "real_robot_check.json"), "w") as f:
    json.dump(res, f, indent=2, sort_keys=True)
print("all_ok:", res["all_ok"])
print("REAL_ROBOT_CHECK_DONE")
