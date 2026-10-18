# Go2 仓储阶段二进度快照（2026-10-18）

负责人：任权伟，导航与系统集成。仅Isaac Sim 4.5仿真，不上真机；JetBot/PPO/VLA未接入。

| 工作 | 已验证结果 | 限制 |
|---|---|---|
| 世界坐标点云修复与虚拟scan | 原锁停轨迹及完整重启通过 | RTX辅助逐点时间此前无效，尚未用于LIO |
| 速度保护与Nav2 | 独立保护/看门狗/直通道导航通过 | 精准驻足与一般朝向仍有失败记录 |
| SLAM先验与AMCL | 真实scan建图、一次已知起点初始化、目标导航 | 建图输入是仿真真实odom；地图非全仓库覆盖 |
| A开阔带固定障碍 | 独立冷启动16项通过，巡航.7057m/s | stateful目标容差；停车后精度仍未通过 |
| B隔断上侧封路、下侧改道 | 静止规划11项、实际行走17项通过，巡航.7137m/s | 单轮；未做独立冷启动/运动中封路 |
| 障碍移除 | 标记/绕行可用 | 移除后回波残留未解决，原实验整体false |

导航唯一输出经过保护桥，.8m/s线速/.6rad/s角速上限，原.55m净距和1.5秒预测保留。控制与到点减速允许0；实测巡航仍要求>=.6m/s。正式AMCL导航关闭Bridge map→odom，不运行SLAM，避免重复TF。

## 查阅

- [A类实际绕行与冷启动](GO2_WAREHOUSE_OPEN_OBSTACLE_A.md)
- [B类封路改道与实际轨迹](GO2_WAREHOUSE_GLOBAL_B.md)
- [SLAM/AMCL历史结果](GO2_WAREHOUSE_SLAM_M2.md)
- [世界点云修复](GO2_WAREHOUSE_LIDAR_WORLD_M1.md)
- [当前计划与后续FAST-LIO2](GO2_WAREHOUSE_NAV_PLAN.md)
- [阶段二脚本入口](../isaac_sim/go2_warehouse/cloud/warehouse_stage2/README.md)
- [B场景复现边界](../isaac_sim/go2_warehouse/cloud/warehouse_stage2/BRANCH_OBSTACLE_README.md)

## 运行与证据范围

阶段一目录是稳定源码，不能用旧Nav2配置启动当前导航。阶段二脚本通过prepare_*构造独立云端运行副本；须按每个里程碑README组合使用，不能把所有manager同时启动。当前公开的stage2目录是107文件的云端一致快照；source_cloud.sha256核对该快照，不包含后来新文件。

默认云根目录/home/admin/go2-warehouse，复用现有Isaac 4.5及隔离ROS运行库。原副本work/baseline保留。权重、完整USD资产、ROS安装包、缓存及完整历史遥测不提交；通过负责人持久化备份取得。experiments中是精选证据，目录内完整云SHA清单可能覆盖未入Git的原始文件，不能将其当精选目录清单。

本快照收尾时无仿真/导航残留，云电脑未关闭；baseline12项、稳定work3项、107源码校验通过。已有原始失败JSON保留，未将规划成功当成实际行走成功。

## 下一步

按用户最新安排，暂不做行进中突然封路；尝试独立FAST-LIO2传感器接入与并行评测。现有XT32点云仅XYZ、没有发布IMU，因此先补可验证的原生IMU及点云时间契约，再构建运行LIO。LIO未通过前保持原导航输入，不将两个节点同时接到同一odom TF。
