#!/usr/bin/env python3
"""v1.6.0 determinism: same-process 1001->1002->1001 + cross-process fingerprint.

Same process:
  * object tuples (id/class/kind/asset/pos/yaw) of run 1 == run 3
  * layout_hash of run 1 == run 3
  * blocked aisle of run 1 == run 3
  * no residue of seed 1002 after re-applying 1001
Cross process:
  * canonical JSON sha256 must be identical between two separate invocations
    (run scripts/dump_v16_fingerprint.py twice and compare)
"""
import hashlib
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

DENS = "medium"


def key(spec):
    return sorted([(o["id"], o["class"], o.get("kind"), o["asset_key"],
                    round(o["position"][0], 3), round(o["position"][1], 3),
                    round(o["yaw_rad"], 6)) for o in spec["objects"]])


def build(seed, density=DENS):
    return scene_mode.build_mixed(gen.md, cfgs, seed, density=density)


seq = []
for seed in (1001, 1002, 1001):
    sp = build(seed)
    seq.append({"seed": seed, "count": sp["object_count"],
                "a": sum(1 for o in sp["objects"] if o["kind"] == "A"),
                "b": sum(1 for o in sp["objects"] if o["kind"] == "B"),
                "blocked": sp["blocked_aisles"], "hash": sp["layout_hash"],
                "key": key(sp), "ids": [o["id"] for o in sp["objects"]]})

a, b, c = seq
res = {
    "density": DENS,
    "counts": [x["count"] for x in seq],
    "a_counts": [x["a"] for x in seq],
    "b_counts": [x["b"] for x in seq],
    "blocked_aisles": [x["blocked"] for x in seq],
    "hashes": [x["hash"][:16] for x in seq],
    "first_equals_third": a["key"] == c["key"],
    "hash_first_equals_third": a["hash"] == c["hash"],
    "ids_equal": a["ids"] == c["ids"],
    "residue_of_1002_after_reapply": len(set(c["ids"]) - set(a["ids"])),
    "differs_1001_1002": a["key"] != b["key"],
}
canon = json.dumps({"objects": [list(x) for x in key(build(1001))],
                    "blocked": build(1001)["blocked_aisles"]}, sort_keys=True)
res["fingerprint_sha256"] = hashlib.sha256(canon.encode()).hexdigest()

with open(os.path.join(OUT, "determinism.json"), "w") as f:
    json.dump(res, f, indent=2, sort_keys=True)
print(json.dumps({k: v for k, v in res.items() if k != "objects"}, indent=2))
print("V16_FORMAL_DET_DONE")
