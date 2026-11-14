#!/usr/bin/env python3
"""CLI: batch-generate N seeds and report statistics (Stage 1).

    python3 scripts/batch_generate.py --n 100 --density medium
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from generator.scene_generator import SceneGenerator  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=100)
    ap.add_argument("--start", type=int, default=1)
    ap.add_argument("--density", default="medium", choices=["easy", "medium", "hard"])
    ap.add_argument("--outdir", default=None, help="if given, save every JSON here")
    args = ap.parse_args()

    gen = SceneGenerator.from_config()
    hashes = {}
    ok = 0
    bad = 0
    failed_objs = 0
    attempts = 0
    placed = 0
    times = []
    counts = {"easy": 0, "medium": 0, "hard": 0}
    per_class_total = {}
    region_total = {}
    outside_int = 0

    t_all = time.time()
    for i in range(args.n):
        seed = args.start + i
        t0 = time.time()
        spec = gen.generate(seed=seed, density=args.density)
        dt = time.time() - t0
        times.append(dt)
        attempts += sum(spec["generation_stats"]["attempts"].values())
        placed += sum(spec["generation_stats"]["placed"].values())
        failed_objs += len(spec["generation_stats"]["failed"])
        for c, n in spec["counts_by_class"].items():
            per_class_total[c] = per_class_total.get(c, 0) + n
        for r, n in (spec.get("regions", {}).get("placed_by_region") or {}).items():
            region_total[r] = region_total.get(r, 0) + n
        counts[args.density] += 1
        if spec["validation"]["valid"]:
            ok += 1
        else:
            bad += 1
        outside_int += int(spec["validation"]["checks"].get("outside_interior", 0))
        h = spec["layout_hash"]
        hashes.setdefault(h, []).append(seed)
        if args.outdir:
            gen.save(spec, os.path.join(args.outdir, "scene_%06d.json" % seed))

    total_t = time.time() - t_all
    dup = {h: s for h, s in hashes.items() if len(s) > 1}

    print("=== batch summary ===")
    print("  density       :", args.density)
    print("  seeds         : %d..%d  (n=%d)" % (args.start, args.start + args.n - 1, args.n))
    print("  valid         :", ok)
    print("  invalid       :", bad)
    print("  outside-interior obstacles:", outside_int)
    print("  objects placed: %d   failed-objects: %d" % (placed, failed_objs))
    print("  placement attempts total:", attempts)
    print("  avg attempts/object     : %.2f" % (attempts / placed if placed else 0))
    print("  unique layouts: %d / %d" % (len(hashes), args.n))
    print("  duplicate layouts       :", len(dup))
    print("  per-class total:", dict(sorted(per_class_total.items())))
    tot_r = sum(region_total.values()) or 1
    print("  per-region total:", {k: "%d (%.1f%%)" % (v, 100.0 * v / tot_r)
                                  for k, v in sorted(region_total.items())})
    print("  time total    : %.2f s   mean/seed: %.3f s   max: %.3f s"
          % (total_t, sum(times) / len(times), max(times)))

    summary = {
        "density": args.density, "n": args.n,
        "seed_range": [args.start, args.start + args.n - 1],
        "valid": ok, "invalid": bad,
        "objects_outside_interior": outside_int,
        "objects_placed": placed, "failed_objects": failed_objs,
        "attempts_total": attempts,
        "avg_attempts_per_object": round(attempts / placed, 3) if placed else None,
        "unique_layouts": len(hashes), "duplicate_layout_groups": len(dup),
        "per_class_total": per_class_total,
        "per_region_total": region_total,
        "per_region_pct": {k: round(100.0 * v / (sum(region_total.values()) or 1), 2)
                           for k, v in region_total.items()},
        "time_total_s": round(total_t, 3),
        "time_mean_s": round(sum(times) / len(times), 4),
        "time_max_s": round(max(times), 4),
    }
    out = os.path.join("outputs", "batch_summary_%s.json" % args.density)
    os.makedirs("outputs", exist_ok=True)
    with open(out, "w") as f:
        json.dump(summary, f, indent=2, sort_keys=True)
    print("  summary json:", out)
    return 0 if bad == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
