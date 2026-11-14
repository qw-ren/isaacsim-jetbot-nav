#!/usr/bin/env python3
"""最小调用示例 —— 队友只需要这个文件就能用场景模块（v1.6.0）。

在 Isaac Sim 的 python 下运行:
    /home/admin/isaacsim/python.sh examples/minimal_usage.py

每个场景 = A 类随机障碍 + B 类封闭通道，不需要传 mode 参数。
"""
import os
import sys

from isaacsim import SimulationApp

app = SimulationApp({"headless": True, "width": 640, "height": 480})

import omni.usd  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from isaac_adapter.scene_manager import WarehouseSceneManager  # noqa: E402


def show(tag, info):
    print("%-10s seed=%s %-6s | A=%d B=%d total=%d | blocked=%s | hash=%s"
          % (tag, info["seed"], info["density"], info["a_count"], info["b_count"],
             info["count"], info["blocked_aisles"], (info["layout_hash"] or "")[:12]))


def main():
    # 1) 加载固定仓库
    mgr = WarehouseSceneManager(ctx=omni.usd.get_context())
    mgr.load_base_scene()
    print("base versions:", mgr.base_versions()["derived_sha256"][:16], "...")

    # 2) 生成并应用一个 seed（A+B 混合，无需 mode）
    spec = mgr.reset_scene(seed=1001, density="medium")
    info = mgr.get_scene_info()
    show("1001M", info)
    print("   blockage:", {k: spec["blockages"][0][k]
                           for k in ("aisle", "station_y", "pieces", "asset")})

    # 3) 查看单个物体
    for o in info["objects"][:3]:
        print("   %-14s %-22s pos=%s yaw=%s size=%s col=%d"
              % (o["id"], (o["reference"] or "").split("/")[-1],
                 o["world_translate"], o["world_yaw_deg"], o["size"],
                 o["collision_prims"]))
    print("   B-class prims:",
          [o["id"] for o in info["objects"] if o["id"].startswith("blockage_")])

    # 4) 切换 seed / 密度
    mgr.reset_scene(seed=1002, density="hard")
    show("1002H", mgr.get_scene_info())

    # 5) 保护区域（导航同学可传入出生点/目标点）
    pz = [(-26.0, -20.0, -23.0, -17.0)]
    spec_pz = mgr.reset_scene(seed=1003, density="medium", protected_zones=pz)
    inside = [o for o in spec_pz["objects"]
              if pz[0][0] <= o["position"][0] <= pz[0][2]
              and pz[0][1] <= o["position"][1] <= pz[0][3]]
    print("protected_zones respected:", len(inside) == 0, "(%d inside)" % len(inside))

    # 6) 保存 / 恢复 JSON
    os.makedirs(os.path.join(ROOT, "outputs"), exist_ok=True)
    path = os.path.join(ROOT, "outputs", "demo_scene_1002_hard.json")
    mgr.reset_scene(seed=1002, density="hard")
    mgr.save_scene(path)
    print("saved ->", path)
    mgr.reset_scene(seed=1001, density="medium")
    loaded = mgr.load_scene(path)
    info2 = mgr.get_scene_info()
    print("restored -> seed=%s density=%s A=%d B=%d total=%d hash_match=%s"
          % (info2["seed"], info2["density"], info2["a_count"], info2["b_count"],
             info2["count"], loaded["layout_hash"] == info2["layout_hash"]))

    # 7) 清空随机层
    n = mgr.clear_random_obstacles()
    print("cleared", n, "objects; now", mgr.get_scene_info()["count"])

    print("DONE")
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
