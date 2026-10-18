# 阶段二 M1：仓储辅助地图、全局Costmap与Navfn（无运动验收通过）

日期2026-10-15（Asia/Shanghai），任权伟，导航与系统集成。
本轮完成M1的地图与规划子任务，完整DWB目标行走尚未完成；不声明SLAM/AMCL或新增障碍导航已实现。
代码/精选证据在独立克隆的feature/go2-warehouse-plan-m1当前工作目录，尚未提交或推送。

## 今天实时状态与工程保护

50796 SSH认证成功；实例本次启动时间10:30:05，原仿真/scan/RViz会话均已不存在。
/home数据盘正常，系统/数据剩余64G/49G。昨天会话持续运行的结论不适用于今天。
使用现有manage_warehouse.sh启动一个仿真，--run-seconds 1800，artifacts/m1_plan_20261015/simulation；
不是重启/关机/释放云电脑。随后恢复独立scan，并新增规划/RViz会话。
没有覆盖work、baseline、策略或已有验收目录；不修改原Git暂存区。

## 系统与改动

```text
仓储USD → CPU碰撞AABB投影开发图 → map_server /map
  → global_costmap（StaticLayer + InflationLayer）
  → Navfn /compute_path_to_pose → 规划线 /plan
仿真Bridge真值odom与map→odom → 起始位姿TF
原点云 → 昨天scan适配 → /scan → RViz（本轮未接入global障碍层）
```

新增build_dev_map.py、nav_env.sh、prepare_nav_runtime.sh、nav_plan.yaml、start_plan.sh、
manage_plan.sh、check_plan.py、plan_view.rviz、start_plan_rviz.sh、plot_plan_check.py和PLAN_README.md。
部署仍在/home/admin/go2-warehouse/nav_stage2/cloud/warehouse_stage2。
只启动map_server、planner_server（内含global Costmap）和lifecycle_manager_plan。
未启动controller_server/bt_navigator/behavior_server/AMCL/SLAM，检查器仅调用ComputePathToPose。

## 开发辅助地图

同一实际warehouse_multiple_shelves.usd，以与仿真相同的/World/Warehouse引用读取。
与今天scene.json核对：米制、Z-up、边界一致，1878个碰撞prim；
1077个启用碰撞且与离地0.05–0.85m相交的AABB投影到5cm栅格。
warehouse_dev_v2尺寸501×797，原点(-12.5,-18.5)，占据53946、自由318534、未知26817格。
原始USD入口文件哈希、碰撞AABB清单、scene元数据和PGM/YAML哈希均保存。

这是保守AABB几何开发图：对旋转/非凸/不连通网格会扩大占据；
没有碰撞盒的矩形内部假设自由，非LiDAR已观测自由空间。根USD哈希不代表整个资产库哈希。
地图能用于排查链路，不能代替SLAM成果、精确可通行性或全身碰撞保证。

## 实际验证

v2首次及仅规划独立停止/重启后，两轮各17项检查全部通过。

| 内容 | 结果 |
| --- | --- |
| 生命周期 | map_server、planner_server、global_costmap均active；use_sim_time=true经参数服务确认 |
| 地图 | /map单发布者；文件经map_server解析后，ROS栅格与PGM逐格一致，行列方向和未知区正确 |
| 代价图 | 与地图网格对齐，StaticLayer与InflationLayer成功加载，存在实际膨胀代价值 |
| 正常目标 | use_start=false，使用当前真实TF位姿到(2,-1)；SUCCEEDED，78路径点，约2.000m |
| 端点 | 终点误差0，起点相对当前odom约0.00493m（地图/TF以恒等map→odom对齐） |
| 独立路径检查 | 1cm密采样，全部位于地图/代价图范围且无未知、占据或禁行代价值；本例代价值为0 |
| 几何距离 | 路径中心到选入碰撞AABB最小距离约1.205m，大于暂定0.45m半径；非全身认证 |
| 障碍内目标 | (0,0.3) ABORTED、无路径 |
| 图外目标 | (100,100) ABORTED、无路径；日志worldToMap错误为此负例的预期输出 |
| 无运动 | 无外部cmd_vel发布者，两轮最大平面位置变化约2.1e-7/2.6e-7m，未发速度 |
| 地图持久性 | 规划前后PGM/YAML哈希不变；仿真时钟推进 |
| 启停管理 | 重复start拒绝；stop后规划进程无残留，原仿真和scan保留；独立重启通过 |
| 可视化 | 实际RViz截图已目视：地图、膨胀代价、绿色scan与橙色计算路径，无目标行走 |

规划动作结果中的planning_time字段为0，未将其解释为零耗时性能；独立重启报告另保存墙钟请求时长。

## 保留的失败与修复

v1使用灰度205作未知边界，但free_thresh=0.25使其占据概率约0.196被分类为自由。
首轮15/16项通过，整体失败，不计作成功版本。
v2使用灰度128（占据概率约0.498，落在free/occupied阈值之间）表示未知，新增文件→ROS逐格往返验证。
v1地图、失败报告及首次日志均保留，未放宽验证门限。
MapIO阈值行为核对官方1.1.20源码：
https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_map_server/src/map_io.cpp

## 依赖与运行状态

复用昨天RViz隔离库中的KDL/yaml-cpp等；本轮另下载4个Ubuntu运行库包1845kB，
仅解包在nav_stage2/deps/nav_runtime，为规划进程局部设置LD_LIBRARY_PATH。
其中GraphicsMagick库在usr/lib而非multiarch目录，已按实际布局修正路径。
系统包、Isaac/CUDA/PyTorch、原仿真环境均未替换。
prepare_nav_runtime.sh固定实测版本，拒绝在规划进程活跃时覆盖其库。

本轮保留go2-warehouse、go2-scan-m1、go2-plan-m1、go2-plan-rviz供查看。
仿真有1800秒运行上限，到期仅仿真退出；后续先检查manager，不能假定仍运行。
完整证据云端artifacts/m1_plan_20261015、本地data/cloud_warehouse/m1_plan_20261015。
可复用命令已同步command.txt，详细操作见阶段二PLAN_README.md。

## 应理解的原理及下一步

地图保存环境底图；Costmap用机器人包络和膨胀表达靠近障碍的代价；Navfn据此计算路线。
/scan可见不代表已更新全局代价图，本轮global只有静态+膨胀层，新增障碍融合仍属于M3。
ComputePathToPose只是算路，障碍目标失败不等于已完成M4无路时的运动停车。
当前位姿是仿真真值，不是SLAM/AMCL定位；后续M2仍需一次建图保存和正式定位。

下一步最小任务：建立独立仿真速度入口、低速限幅、墙钟看门狗与停车流程，
再启动DWB与局部Costmap，验证这一个宽走廊目标的实际低速行走和终点停车。
不直接启用旧1.8m/s控制配置，不引入JetBot、视觉或PPO训练。
