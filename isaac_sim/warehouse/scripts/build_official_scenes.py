#!/usr/bin/env python3
"""Build the official v1.6.0 mixed scenes that are shipped as examples."""
import json
import os
import sys

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

SCENES = [(1001, "medium"), (1001, "hard"), (1002, "medium"), (1003, "hard")]
for seed, dens in SCENES:
    spec = scene_mode.build_mixed(gen.md, cfgs, seed, density=dens)
    spec["scene_name"] = "official_%04d_%s" % (seed, dens)
    path = os.path.join(OUT, "scene_%04d_%s.json" % (seed, dens))
    with open(path, "w") as f:
        json.dump(spec, f, indent=2, sort_keys=True, default=str)
    print("%04d %-6s A=%2d B=%d total=%2d blocked=%s hash=%s"
          % (seed, dens,
             sum(1 for o in spec["objects"] if o["kind"] == "A"),
             sum(1 for o in spec["objects"] if o["kind"] == "B"),
             spec["object_count"], spec["blocked_aisles"], spec["layout_hash"][:12]))
print("OFFICIAL_SCENES_DONE")
