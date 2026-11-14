#!/usr/bin/env python3
"""Generate one A+B mixed scene and save its JSON.

    python3 scripts/make_scene.py --seed 1001 --density medium
    python3 scripts/make_scene.py --seed 1002 --density hard --out /tmp/s.json

Pure Python (numpy/scipy/PyYAML); no Isaac Sim required.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import yaml  # noqa: E402

from generator import config, scene_mode  # noqa: E402
from generator.scene_generator import SceneGenerator  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--density", default="medium",
                    choices=["easy", "medium", "hard"])
    ap.add_argument("--out", default=None)
    a = ap.parse_args()

    cfgs = config.load_all()
    with open(os.path.join(ROOT, "config", "difficulty_v15_experiment.yaml")) as f:
        cfgs["difficulty_v15"] = yaml.safe_load(f)

    gen = SceneGenerator.from_config()
    spec = scene_mode.build_mixed(gen.md, cfgs, a.seed, density=a.density)

    out = a.out or os.path.join(ROOT, "outputs",
                                "scene_%04d_%s.json" % (a.seed, a.density))
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with open(out, "w") as f:
        json.dump(spec, f, indent=2, sort_keys=True, default=str)

    a_n = sum(1 for o in spec["objects"] if o["kind"] == "A")
    b_n = sum(1 for o in spec["objects"] if o["kind"] == "B")
    print("seed=%d density=%-6s A=%d B=%d total=%d blocked=%s"
          % (a.seed, a.density, a_n, b_n, len(spec["objects"]),
             spec["blocked_aisles"]))
    print("layout_hash:", spec["layout_hash"][:16])
    print("json       :", out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
