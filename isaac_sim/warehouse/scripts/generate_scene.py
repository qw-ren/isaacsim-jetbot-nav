#!/usr/bin/env python3
"""CLI: generate one deterministic seed layout (Stage 1).

Examples
    python3 scripts/generate_scene.py --seed 1001
    python3 scripts/generate_scene.py --seed 1001 --density hard --out outputs/hard/
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from generator.scene_generator import SceneGenerator  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--density", default="medium", choices=["easy", "medium", "hard"])
    ap.add_argument("--out", default="outputs", help="output directory or .json path")
    args = ap.parse_args()

    gen = SceneGenerator.from_config()
    spec = gen.generate(seed=args.seed, density=args.density)

    out = args.out
    if not out.endswith(".json"):
        out = os.path.join(out, "scene_%06d.json" % args.seed)
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    gen.save(spec, out)

    v = spec["validation"]
    print("seed=%d density=%s objects=%d" % (args.seed, args.density, spec["object_count"]))
    print("  counts       :", spec["counts_by_class"])
    print("  layout_hash  :", spec["layout_hash"])
    print("  validation   :", "VALID" if v["valid"] else "INVALID")
    if not v["valid"]:
        for r in v["reasons"]:
            print("    -", r)
    print("  json         :", out)
    return 0 if v["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
