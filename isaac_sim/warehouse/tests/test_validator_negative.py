#!/usr/bin/env python3
"""Negative tests: the validator MUST reject illegal layouts (Stage 1).

    python3 tests/test_validator_negative.py
"""
from __future__ import annotations

import copy
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from generator import placement_validator as pv  # noqa: E402
from generator.scene_generator import SceneGenerator  # noqa: E402

PASS = FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print("  PASS %-56s %s" % (name, detail))
    else:
        FAIL += 1
        print("  FAIL %-56s %s" % (name, detail))


def main():
    gen = SceneGenerator.from_config()
    md = gen.md
    base = gen.generate(seed=1001, density="medium")
    check("baseline layout is valid", base["validation"]["valid"])

    def one_cell_of(mask):
        ys, xs = np.nonzero(mask)
        i = 0
        return (md["origin"][0] + (xs[i] + 0.5) * md["res"],
                md["origin"][1] + (ys[i] + 0.5) * md["res"])

    print("== negative cases ==")

    # 1. overlap: move object[1] onto object[0]
    s = copy.deepcopy(base)
    s["objects"][1]["position"][0] = s["objects"][0]["position"][0]
    s["objects"][1]["position"][1] = s["objects"][0]["position"][1]
    r = pv.validate(s, md, md)
    check("overlap rejected", (not r["valid"]) and r["checks"]["too_close"] > 0,
          str(r["checks"]))

    # 2. on fixed structure
    s = copy.deepcopy(base)
    cx, cy = one_cell_of(md["structure_occ"])
    s["objects"][0]["position"][0] = cx
    s["objects"][0]["position"][1] = cy
    r = pv.validate(s, md, md)
    check("on-structure rejected", (not r["valid"]) and r["checks"]["on_structure"] > 0,
          str(r["checks"]))

    # 3. outside the floor
    s = copy.deepcopy(base)
    s["objects"][0]["position"][0] = md["floor_extent"]["x"][0] - 50.0
    r = pv.validate(s, md, md)
    check("out-of-bounds rejected", (not r["valid"]) and
          (r["checks"]["in_bounds"] > 0 or r["checks"]["outside_placeable"] > 0),
          str(r["checks"]))

    # 4. inside a keep-clear corridor
    s = copy.deepcopy(base)
    kc = gen.cfgs["warehouse"]["placement"]["keep_clear"][0]
    s["objects"][0]["position"][0] = (kc["x"][0] + kc["x"][1]) / 2
    s["objects"][0]["position"][1] = (kc["y"][0] + kc["y"][1]) / 2
    r = pv.validate(s, md, md)
    check("keep-clear violation rejected",
          (not r["valid"]) and (r["checks"]["in_keep_clear"] > 0 or
                                r["checks"]["outside_placeable"] > 0),
          str(r["checks"]))

    # 5. protected zone
    s = copy.deepcopy(base)
    o = s["objects"][0]
    z = (o["position"][0] - 1.0, o["position"][1] - 1.0,
         o["position"][0] + 1.0, o["position"][1] + 1.0)
    r = pv.validate(s, md, md, protected_zones=[z])
    check("protected-zone violation rejected",
          (not r["valid"]) and r["checks"]["in_protected"] > 0, str(r["checks"]))

    # 6. outside the walled interior but still on the floor bbox
    s = copy.deepcopy(base)
    s["objects"][0]["position"][0] = 7.4   # right of the outer wall
    s["objects"][0]["position"][1] = 12.0
    r = pv.validate(s, md, md)
    check("outside-interior rejected",
          (not r["valid"]) and (r["checks"].get("outside_interior", 0) > 0),
          str({k: v for k, v in r["checks"].items() if v}))

    # 7. baseline still valid after all the mutation attempts
    r2 = pv.validate(base, md, md)
    check("baseline unaffected by mutations", r2["valid"])

    print("\n== SUMMARY: %d passed, %d failed ==" % (PASS, FAIL))
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
