# 仓储随机场景模块（warehouse）

在团队现有的 `isaac_sim/jetbot_sim.usd` 之上，把**固定仓库**替换为**可复现的 A+B 随机布局**，
并保证 JetBot（3 个传感器 + 7 个 ROS 2 OmniGraph）不被破坏。

> 本模块**不包含**导航、SLAM、Nav2、PPO 或视觉训练代码，只负责"场景生成 + 机器人接入"。

---

## 1. 它做了什么

| 能力 | 说明 |
| --- | --- |
| 固定底座 | 官方 `full_warehouse` 引用 + 7 条 `active=false` 覆盖（可还原到原始仓库） |
| A 类障碍 | 随机货箱/托盘/障碍物，按 Seed 采样，做碰撞、通道与可见性校验 |
| B 类封闭 | 随机选 1 条货架通道，用混合宽度纸箱行封死（单侧余隙 0.00–0.10 m） |
| 可复现 | 同一 Seed + 同一 `config/` + 同一 `scenes/base_map.npz` → 相同 `layout_hash` |
| 机器人保护 | 出生点 1.2 m × 1.2 m 保护区，A/B 都不会压到机器人 |
| 二维一致性 | 三维场景与二维占据栅格由同一份布局生成，并有 SHA256 守卫 |

---

## 2. 目录

```text
isaac_sim/warehouse/
├── config/          4 个 YAML（仓库 / 障碍 / 难度 / 资产登记）
├── generator/       生成、拓扑、封闭、校验、地图构建（纯 Python，可单测）
├── isaac_adapter/   WarehouseSceneManager（加载、应用布局、统计）
├── integration/     JetBot 集成层 + 运动测量探针
├── visualization/   二维俯视图与区域图
├── examples/        可直接运行的示例
├── tests/           纯 Python 回归测试（73 项）
├── scripts/         建库、批量生成、实验、环境检查
├── scenes/          warehouse_base.usd + base_map.npz + 元数据 + examples/
└── project/         jetbot_sim.usd（本模块使用的机器人副本）
```

---

## 3. 依赖

| 依赖 | 说明 |
| --- | --- |
| Isaac Sim 4.5 | 三维层需要；纯 Python 层不需要 |
| Python 3.10+ | numpy / scipy / PyYAML（二维图另需 Pillow） |
| 官方资产 | `/home/admin/isaacsim_assets/Assets/Isaac/4.5/Isaac/`（各机器同路径） |
| ROS 2 Humble | 只在需要验证话题时使用 |

场景 USD 使用**绝对路径**引用官方资产，克隆后无需重连。

---

## 4. 三步跑通

```bash
# 1) 环境检查（资产 / 场景哈希 / 机器人位置）
python3 isaac_sim/warehouse/scripts/check_env.py --copy-robot

# 2) 纯 Python 回归（不需要 Isaac）
cd isaac_sim/warehouse && python3 tests/test_config.py      # 期望 73 passed, 0 failed

# 3) 集成冒烟（需要 Isaac）
cd isaac_sim/warehouse
<isaac>/python.sh examples/inspect_warehouse_jetbot.py      # 期望 "ok": true
```

`check_env.py` 会校验三个官方资产、`warehouse_base.usd` 的哈希与 `/World/Warehouse` 是否可展开，
并在需要时把 `project/jetbot_sim.usd` 复制到 `/home/admin/Desktop/jetbot_project/project/`
（该文件内部引用 `../../../isaacsim_assets`，必须位于这个深度；**文件本身不会被修改**）。
也可以用 `JETBOT_USD` 环境变量指定其它位置。

---

## 5. 常用命令

```bash
cd isaac_sim/warehouse

# 生成一个 A+B 场景（JSON）
python3 scripts/make_scene.py --seed 1001 --density medium

# 生成 4 个正式示例场景
python3 scripts/build_official_scenes.py

# 批量生成
python3 scripts/batch_generate.py --seeds 1001-1200 --density medium

# 二维俯视图
python3 visualization/draw_scene.py --spec scenes/examples/scene_1001_medium.json --out /tmp/s.png

# 集成检查 / Seed 切换 / 运动验证（需要 Isaac）
<isaac>/python.sh examples/inspect_warehouse_jetbot.py
<isaac>/python.sh examples/integration_seed_switch.py
<isaac>/python.sh examples/motion_test.py
```

---

## 6. 集成层接口

| 函数 | 作用 |
| --- | --- |
| `ji.import_jetbot(stage)` | 引用导入 `/World/jetbot` 与 `/World/ROS2`，重绑 7 条跨子树关系（幂等） |
| `ji.apply_spawn(stage)` | 设置出生位姿 `(-9.97, -7.37, 0.1009)`，yaw 0° |
| `ji.reset_and_apply(mgr, seed, density, protected_zones)` | 暂停 → 清 `cmd_vel` → 复位 → 清零速度 → 换布局 → 碰撞校验 |
| `ji.zero_motion()` | 轮关节 + 底盘线/角速度归零 |
| `ji.verify_bindings(stage)` | 校验 7 条绑定 |
| `ji.status(stage)` | 汇总状态（机器人 prims、图节点、轮几何、绑定） |
| `MotionProbe` | 用 PhysX 刚体状态 + 仿真时钟测真实运动（不要用 USD Xform + 墙钟） |

---

## 7. Seed 与复现

```python
import omni.usd
from isaac_adapter.scene_manager import WarehouseSceneManager
from integration import jetbot_integration as ji

mgr = WarehouseSceneManager(ctx=omni.usd.get_context())
mgr.load_base_scene()                       # 固定仓库
ji.import_jetbot(mgr.stage)                 # JetBot + 传感器 + 7 图
mgr.reset_scene(seed=1001, density="medium",
                protected_zones=ji.PROTECTED_ZONE)

# 换 Seed（不重建机器人）
r = ji.reset_and_apply(mgr, seed=1002, density="hard",
                       protected_zones=ji.PROTECTED_ZONE, resume=False)
print(r["scene"]["hash"][:12], r["collision"]["ok"])
```

基线（可用于核对）：

| Seed | 密度 | A | B | 封闭通道 | layout_hash |
| --- | --- | --- | --- | --- | --- |
| 1001 | medium | 20 | 6 | [1] | `3d87ce3ecfff…` |
| 1002 | hard | 32 | 7 | [3] | `6146da7590ff…` |

`scenes/examples/` 里附了 4 个正式场景 JSON 与一份确定性报告，可直接对照。

---

## 8. 已知限制

| # | 限制 |
| --- | --- |
| 1 | 原地旋转控制误差约 **−22.5%**（指令 0.60 → 实际 0.465 rad/s），源于球体轮接触；换圆柱碰撞体实测更差，已保留原状 |
| 2 | `/odom` 与 PhysX 实际运动一致，可作反馈；但**不要假设 `cmd_vel` 会被精确执行** |
| 3 | 直线纵向滑移约 11% |
| 4 | B 类通道封闭余隙 0.00–0.10 m（单侧），换更大机器人需重验 |
| 5 | 随机障碍为静态摆放，没有刚体 |
| 6 | 测量必须用仿真时钟：headless 下仿真/墙钟 ≈ 0.39，用墙钟会把速率低估约 2.6 倍 |
| 7 | `jetbot_sim.usd` 内部引用深度固定（`../../../isaacsim_assets`），移动该文件会失效 |

设计与验收数据见仓库根目录 `docs/warehouse_random_scene.md`。
