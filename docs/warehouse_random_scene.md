# 随机仓库场景模块（isaac_sim/warehouse）

本文件说明 `isaac_sim/warehouse/` 的设计、接口与验收数据，供队友接手时快速定位。

## 1. 目标与边界

| 做 | 不做 |
| --- | --- |
| 固定仓库 → A+B 随机布局（Seed 可复现） | SLAM / Nav2 / PPO / 视觉训练 |
| 保证 JetBot（3 传感器 + 7 个 ROS 2 图）不被破坏 | 修改官方资产或 `jetbot_sim.usd` |
| 提供集成层（导入、出生点、保护区、安全重置） | 修改差速控制器参数（除实测几何） |

## 2. 分层

```
config/*.yaml      →  generator/  →  JSON 场景定义  →  isaac_adapter/  →  USD 会话层
                                        ↓
                                  visualization/（二维图）
integration/ 负责把 JetBot 放进随机场景，并保证 ROS 2 图可用
```

| 层 | 依赖 | 可独立测试 |
| --- | --- | --- |
| `generator/`（采样、拓扑、封闭、校验） | numpy / scipy / PyYAML | ✅ `tests/test_config.py` 73 项 |
| `visualization/` | Pillow | ✅ |
| `isaac_adapter/`、`integration/` | Isaac Sim 4.5 + pxr | ❌ 需要 Isaac |

## 3. 关键参数

| 项 | 值 |
| --- | --- |
| 布局版本 | 1.6.0（A_B_mixed，A 偏好 1.4，B 封闭 1 条通道） |
| 密度 | Easy 6–10 / Medium 18–24 / Hard 28–40 |
| 机器人出生点 | `(-9.97, -7.37, 0.1009)`，yaw 0° |
| 保护区 | `(-10.57, -7.97, -9.37, -6.77)`，1.2 m × 1.2 m |
| 轮半径 / 轮距（实测） | 0.1005 m / 0.2802 m |

## 4. 复现

```bash
cd isaac_sim/warehouse
python3 tests/test_config.py                       # 73/0
python3 scripts/generate_scene.py --seed 1001 --density medium
```

用同一 Seed 生成的 `layout_hash` 在任何机器上一致（只依赖 `config/` 与 `scenes/base_map.npz`）。

## 5. 集成层接口

| 函数 | 作用 |
| --- | --- |
| `import_jetbot(stage)` | 引用导入 `/World/jetbot` 与 `/World/ROS2`，重绑 7 条跨子树关系（幂等） |
| `apply_spawn(stage)` | 设置出生位姿 |
| `reset_and_apply(mgr, seed, density, protected_zones)` | 暂停 → 清 cmd_vel → 复位 → 清零速度 → 换布局 → 碰撞校验 |
| `zero_motion()` | 轮关节 + 底盘线/角速度归零 |
| `verify_bindings(stage)` | 校验 7 条绑定 |
| `status(stage)` | 汇总状态 |
| `motion_probe.MotionProbe` | 用 PhysX + 仿真时钟测真实运动（不要用 USD Xform + 墙钟） |

## 6. 验收数据（2026-11-14）

| 项 | 结果 |
| --- | --- |
| 集成检查 | 机器人 439 prims、7 图 / 39 节点、7/7 绑定有效、轮底 3e-05 m |
| 场景 | Seed 1001M：A=20 B=6 blocked=[1] hash `3d87ce3ecfff` |
| Seed 切换 | 1001M → 1002H → 1001M，机器人/传感器/图恒定，布局可复现 |
| ROS 2 | `/clock /rgb /imu /scan /point_cloud /odom /tf /tf_static /cmd_vel` 全部实收 |
| 直线 | 指令 0.15 → 实测 0.1507 m/s |
| 原地旋转 | 指令 0.60 → 实际 0.465 rad/s（−22.5%，`/odom` 与 PhysX 一致） |

## 7. 已知限制

见 `isaac_sim/warehouse/README.md` 的「已知限制」表；其中旋转误差已定位为球体轮接触问题，
换圆柱碰撞体实验反而更差（−67.7%），因此保持原状并记录。

## 8. 变更记录

| 版本 | 内容 |
| --- | --- |
| 1.6.0 | A+B 混合布局定稿 |
| v1.6.0-jetbot | JetBot + 传感器 + 7 图集成 |
| v1.6.0-jetbot-r1 | 相机 orient 修正、安全重置、旋转误差复核 |
| v1.6.0-jetbot-r2 | 修正测量时钟，确认 `/odom` 准确 |
| v1.6.0-delivery | 交付整理（文档、清单、限制） |
