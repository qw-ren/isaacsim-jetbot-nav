#!/usr/bin/env python3
"""Determinism tests for the Stage-1 generator.

    python3 tests/test_determinism.py

Covers
  1. same seed, same process, generated twice  -> identical layout
  2. same seed, DIFFERENT PROCESSES            -> byte-identical JSON
  3. different seeds                            -> different layouts
  4. layout_hash ignores the seed (hash of layout only)
  5. densities produce counts inside their configured ranges
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from generator.scene_generator import SceneGenerator  # noqa: E402

PASS = FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print("  PASS %-58s %s" % (name, detail))
    else:
        FAIL += 1
        print("  FAIL %-58s %s" % (name, detail))


def canon_objects(spec):
    return json.dumps(spec["objects"], sort_keys=True)


def run_cli(seed, outdir):
    r = subprocess.run([sys.executable, os.path.join(ROOT, "scripts", "generate_scene.py"),
                        "--seed", str(seed), "--out", outdir],
                       cwd=ROOT, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError("CLI failed (%d): %s" % (r.returncode, r.stderr[-800:]))
    return os.path.join(outdir, "scene_%06d.json" % seed)


def main():
    gen = SceneGenerator.from_config()

    print("== 1. same seed, same process ==")
    a = gen.generate(seed=1001, density="medium")
    b = gen.generate(seed=1001, density="medium")
    check("object count equal", a["object_count"] == b["object_count"], str(a["object_count"]))
    check("objects identical (canonical)", canon_objects(a) == canon_objects(b))
    check("layout_hash equal", a["layout_hash"] == b["layout_hash"], a["layout_hash"][:16])
    check("counts_by_class equal", a["counts_by_class"] == b["counts_by_class"])

    print("== 2. same seed, different processes ==")
    with tempfile.TemporaryDirectory() as t1, tempfile.TemporaryDirectory() as t2:
        p1 = run_cli(1001, t1)
        p2 = run_cli(1001, t2)
        b1 = open(p1, "rb").read()
        b2 = open(p2, "rb").read()
        check("JSON byte-identical across processes", b1 == b2, "%d bytes" % len(b1))
        s1 = json.loads(b1)
        check("layout_hash identical across processes",
              s1["layout_hash"] == a["layout_hash"])

    print("== 3. different seeds differ ==")
    c = gen.generate(seed=1002, density="medium")
    check("seed 1001 != 1002 layout",
          a["layout_hash"] != c["layout_hash"],
          "%s vs %s" % (a["layout_hash"][:8], c["layout_hash"][:8]))

    print("== 4. layout_hash excludes seed ==")
    # craft a copy of spec 1001 with a different seed: hash must be unchanged
    x = json.loads(json.dumps(a))
    x["seed"] = 999999
    from generator.scene_generator import layout_hash
    check("layout_hash unchanged when seed field changes",
          layout_hash(x["objects"]) == a["layout_hash"])

    print("== 5. density ranges respected ==")
    tiers = gen.cfgs["difficulty"]["tiers"]
    for dens in ("easy", "medium", "hard"):
        s = gen.generate(seed=1001, density=dens)
        okall = True
        for cls, (lo, hi) in ((k, v) for k, v in tiers[dens].items()
                              if isinstance(v, list)):
            n = s["counts_by_class"].get(cls, 0)
            if not (lo <= n <= hi):
                okall = False
        check("density %-6s counts in range" % dens, okall,
              str(s["counts_by_class"]))

    print("== 6. every obstacle lies inside the walled interior ==")
    from generator import placement_validator as pv
    md = gen.md

    def outside(obj):
        x0, y0, x1, y1 = pv._aabb(obj)
        ix0, iy0, ix1, iy1 = pv.rect_cells(x0, y0, x1, y1, md["origin"], md["res"],
                                           md["nx"], md["ny"])
        n = (ix1 - ix0 + 1) * (iy1 - iy0 + 1)
        return pv.rect_sum(md["int_interior"], ix0, iy0, ix1, iy1) < n

    bad_total = 0
    checked = 0
    for dens in ("medium", "hard"):
        for s in range(1, 21):
            sp = gen.generate(seed=s, density=dens)
            checked += sp["object_count"]
            bad_total += sum(1 for o in sp["objects"] if outside(o))
    check("no obstacle outside interior (20x2 seeds)", bad_total == 0,
          "%d objects checked, %d outside" % (checked, bad_total))

    print("\n== SUMMARY: %d passed, %d failed ==" % (PASS, FAIL))
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
