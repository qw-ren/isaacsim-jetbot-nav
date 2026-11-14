"""Stage-0 config consistency tests (run with SYSTEM python3, no Isaac Sim).

    python3 tests/test_config.py

Exit code 0 = all checks passed.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from generator import config  # noqa: E402

PASS = FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print("  PASS %-58s %s" % (name, detail))
    else:
        FAIL += 1
        print("  FAIL %-58s %s" % (name, detail))


def main():
    print("== load configs ==")
    cfg = config.load_all()
    for k in config.CONFIG_FILES:
        check("config loaded: %s" % k, isinstance(cfg[k], dict))

    wh = cfg["warehouse"]; ob = cfg["obstacles"]; reg = cfg["asset_registry"]; df = cfg["difficulty"]
    R = reg["assets"]

    print("== base geometry ==")
    b = wh["base"]
    fx, fy = b["floor_extent"]["x"], b["floor_extent"]["y"]
    wx, wy = b["world_bbox"]["x"], b["world_bbox"]["y"]
    check("base.usd declared", isinstance(b["usd"], str), b["usd"])
    check("floor inside world bbox",
          fx[0] >= wx[0] and fx[1] <= wx[1] and fy[0] >= wy[0] and fy[1] <= wy[1])
    check("floor size matches extent",
          abs((fx[1] - fx[0]) - b["floor_size_m"][0]) < 1e-6 and
          abs((fy[1] - fy[0]) - b["floor_size_m"][1]) < 1e-6,
          "%.1f x %.1f" % (b["floor_size_m"][0], b["floor_size_m"][1]))
    check("world bbox larger than floor (overshoots)",
          (wy[1] - wy[0]) > (fy[1] - fy[0]),
          "world %.1f > floor %.1f" % (wy[1] - wy[0], fy[1] - fy[0]))

    print("== placement areas inside floor ==")
    for key in ("primary_area", "secondary_area"):
        a = wh["placement"][key]
        ok = (a["x"][0] >= fx[0] and a["x"][1] <= fx[1] and
              a["y"][0] >= fy[0] and a["y"][1] <= fy[1])
        check("%s inside floor" % key, ok, str(a))
    for kc in wh["placement"]["keep_clear"]:
        ok = (kc["x"][0] >= fx[0] and kc["x"][1] <= fx[1] and
              kc["y"][0] >= fy[0] and kc["y"][1] <= fy[1])
        check("keep_clear %s inside floor" % kc["name"], ok)

    print("== sampling regions ==")
    rg = wh["placement"]["regions"]
    w = rg["weights"]
    check("region keys complete", set(w) == {"aisle", "shelf_edge", "open"}, str(sorted(w)))
    check("region weights sum to 1", abs(sum(w.values()) - 1.0) < 1e-6, str(sum(w.values())))
    check("aisle band inside floor",
          rg["aisle_band_y"][0] >= fy[0] and rg["aisle_band_y"][1] <= fy[1],
          str(rg["aisle_band_y"]))
    check("edge margin positive", float(rg["edge_margin_m"]) > 0, str(rg["edge_margin_m"]))
    check("aisle weight is the largest", w["aisle"] == max(w.values()))
    check("region mode valid", rg.get("mode") in ("quota", "stochastic"), str(rg.get("mode")))

    print("== three-layer model ==")
    L = wh["layers"]
    for name in ("fixed", "random", "decor"):
        check("layer %s declared" % name, name in L)
    check("fixed layer marked keep", L["fixed"].get("keep") is True)
    check("random layer has prim_root", "/World/" in L["random"].get("prim_root", ""))

    print("== existing clutter: selective cleanup ==")
    ec = wh["existing_clutter"]
    inv = ec["investigation"]
    check("policy == derived_clean_base_improved",
          ec["policy"] == "derived_clean_base_improved", ec["policy"])
    check("counts consistent (keep + clean == total)",
          inv["keep_on_shelf_or_under_rack"] + inv["clean_floor_or_aisle"] == inv["total_objects"],
          "%d + %d = %d" % (inv["keep_on_shelf_or_under_rack"], inv["clean_floor_or_aisle"],
                            inv["total_objects"]))
    check("clean_list length == clean count",
          len(ec["clean_list"]) == inv["clean_floor_or_aisle"],
          "%d vs %d" % (len(ec["clean_list"]), inv["clean_floor_or_aisle"]))
    check("clean_list names unique", len(set(ec["clean_list"])) == len(ec["clean_list"]))
    check("source file NOT modified",
          ec["mechanics"].get("source_file_modified") is False)
    pc = ec["mechanics"].get("parent_child_check", "")
    check("parent/child check recorded",
          bool(pc) and ("fixed" in pc or "candidates" in pc), pc)
    check("fixed collision retention recorded",
          set(ec["mechanics"]["fixed_collision_retained"]) == {"wall", "shelf", "floor"})
    check("fallback policies available", len(ec["fallback_policies"]) >= 1)

    print("== class scope ==")
    C = ob["classes"]
    check("class count == 8", len(C) == 8, ",".join(sorted(C)))
    for c in ("shelf", "wall"):
        check("%s is fixed" % c, C[c].get("fixed") is True and C[c].get("random") is False)
    for c in ("box", "pallet", "barrier"):
        check("%s random+required" % c,
              C[c].get("random") is True and C[c].get("required_this_phase") is True)
    check("person NOT implemented", C["person"].get("implemented") is False)
    check("sign fixed-mount only", C["sign"].get("random") is False)

    print("== barrier strategy ==")
    bar = C["barrier"]
    check("default = cone_isolation", bar["strategy"] == "cone_isolation", bar["strategy"])
    for u in bar["strategies"][bar["strategy"]]["units"]:
        check("barrier unit %s in registry" % u, u in R)
    check("carton wall marked temporary",
          bar["strategies"]["carton_wall"]["status"] == "temporary_fallback")

    print("== variants in registry ==")
    for cname, c in C.items():
        for v in c.get("variants", []):
            check("variant %s.%s" % (cname, v), v in R)

    print("== registry / disk ==")
    sr = reg["source_root"]
    if os.path.isdir(sr):
        missing = [a for a, v in R.items() if not os.path.exists(os.path.join(sr, v["path"]))]
        check("all registry assets exist", not missing, ",".join(missing))
        check("base warehouse exists", os.path.exists(os.path.join(sr, b["usd"])))
        ep = os.path.join(Path(__file__).resolve().parent.parent, b["entry_point"])
        check("base entry point built", os.path.exists(ep), b["entry_point"])
    else:
        print("  SKIP source_root absent")

    print("== difficulty (density only) ==")
    for tier in ("easy", "medium", "hard"):
        check("tier %s" % tier, tier in df["tiers"])
        for cls in ("box", "pallet", "barrier"):
            rg = df["tiers"][tier].get(cls)
            check("  %s.%s range" % (tier, cls),
                  isinstance(rg, list) and len(rg) == 2 and rg[0] <= rg[1], str(rg))
    check("density guard-rails", "limits" in df)

    print("== scoping / demo markers ==")
    check("start/goal demo-only", wh.get("start_goal_demo_only") is True)
    check("protected_zones empty default", wh.get("protected_zones") == [])
    check("map cache declared (2D/3D same source)",
          wh["map"].get("cache", "").endswith(".npz") and
          wh["map"].get("meta", "").endswith(".json"))

    print("\n== SUMMARY: %d passed, %d failed ==" % (PASS, FAIL))
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
