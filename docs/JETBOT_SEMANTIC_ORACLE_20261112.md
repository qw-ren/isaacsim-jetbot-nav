# 交互式对象语义导航（实仿闭环验证，2026-11-12）

基于 PR6 的 `0d957b984c7c5cfb7570caf3be255eabdda149e2`，独立分支
`feature/jetbot-semantic-oracle-20261112`。只用于仿真。导航参数沿用
`jetbot_3x_nav_v2_r040_20261101`；这不是新的定位精度声明。

旧`extension_contract.json`保持原样。本独立配置仅对语义对象观测显式启用
`semantic.mode=ground_truth_oracle`；机器人定位与控制不允许使用机器人真值。
不把oracle实验描述为原无对象真值基线或视觉模型验收。

## 模块边界

`USD → oracle → /semantic/objects/observations → mapper → /semantic/objects/map
→ 中文规则查询/goal manager → ComputePathToPose → NavigateToPose → 原安全门`。

实现包：`ros2_ws/src/jetbot_semantic_nav`。oracle 需要 USD 的仿真线程，
因此由 `run_sim.py` 显式启用；它不是第二个仿真进程。mapper、manager
是独立 ROS 节点，后者替代旧 `operator_session.py`，避免两个任务所有者。
默认旧配置没有 `navigation.semantic`，不启用新模块。

独立测试场景用 NVIDIA 原生 `SM_PushcartA_02.usd`、`pallet.usd`、
`SM_CardBoxA_01_467.usd` 各两个，保留原生碰撞。它们在仿真内新增，
不进入已有规划或定位地图。原 `semantic_places.json` 保持人工地点目录。
这六个实例是闭环测试夹具，不是团队最终资产选择或随机场景生成器。

类别与资产分开：内部类别 `shelf/wall/box/pallet/barrier/cart/sign/person`。
中文目标支持货架、纸箱、托盘、推车、标牌；墙体、挡板、人当前不作为指令目标。
资产适配器内部 `asset_kind=pushcart` 对应标准类别 `cart`，不要求视觉采用 USD 文件名。
固定场景货架可用 `registered_prims` 显式登记，无须复制货架或修改先验；
未登记类别返回 NOT_FOUND，不能宣称全仓库所有对象都已标注。
当前推车是静止目标，移动目标只支持取消，不实现追踪。

## 坐标与真值边界

`build_prior_map.py` 直接把 `ComputeWorldBound` 得到的世界 XY 栅格化；
`build_localization_mesh_prior.py` 直接变换 Mesh 顶点至同一世界坐标。
地图 YAML 的 origin 是栅格左下角在该坐标系的位置，不是应再加给对象的偏移。
本场景 `usd_world → map` 注册因此为单位变换，明确记录在 CASE 中；
oracle 启动核验 prepared USD 和规划 PGM 的 SHA256，拒绝不匹配场景。
这由地图生成关系确定，不是根据机器人真值与 AMCL 在线拟合。

oracle 只读取已登记物体 Prim 的实时世界包围盒和 Prim 朝向，不读机器人位姿，
不发布 map/odom/chassis TF，不订阅或校正定位。
AMCL 与 FAST-LIVO2 继续提供机器人估计位姿。全场景 oracle 的 source 明确为
ground_truth，不能称视觉识别。

发布的碰撞几何是世界轴对齐包围盒，orientation=单位四元数；真实 Prim 朝向
另存审计字段 `prim_orientation_wxyz`，两者不能混用。转换到其他 map 注册后，
盒子随该刚体变换旋转。相机坐标观测由 mapper 在 image stamp 查询
map←optical TF；不可用时拒绝该帧，不取 latest TF 掩盖时序问题。

## 对象与命令接口 v1

现有 contract 的 vision_msgs 是预留建议，并没有稳定 ID/source/生命周期的正式协议。
这里的独立协议使用已有 `std_msgs/msg/String` 承载 UTF-8 JSON，无额外消息编译依赖。
机器可读字段定义见`ros2_ws/src/jetbot_semantic_nav/config/object_observation.schema.json`；
运行时另检查有限数、单位四元数、实例ID一致性、时序和TF可用性。
它不占用或改动 `/perception/detections`、`/perception/object_poses`。
后续 Detection3DArray/YOLO 通过薄适配器转成同一协议，mapper 及下游保持不变。

对象观测：

```json
{"version":1,"frame_id":"usd_world","stamp":12.5,"sequence":25,
 "source":"ground_truth","objects":[
   {"object_id":"/World/SemanticOracle/cart_near","class_name":"cart",
    "confidence":1.0,"position":[-3.0,7.0,0.2],
    "orientation_xyzw":[0.0,0.0,0.0,1.0],"dimensions":[0.94,1.65,0.38],
    "source":"ground_truth"}]}
```

position 是包围盒中心，dimensions 是该 orientation 对应的盒子全长，单位米；
stamp 是仿真秒，非 wall time。视觉用 source=vision、稳定实例 ID、测量时间和
相机 optical_frame，且必须先由 RGB-D/LiDAR 关联得到米制三维盒子。
单目二维像素框不能直接填 position。不同观测源应使用各自 ID 命名空间，
更换源时停止旧 oracle；同一 ID 不允许改变类别或 source。

map topic 输出相同记录，加 last_seen/status。同 ID 更新，不每帧增新对象。
过期同时检查仿真年龄与本地接收 wall 年龄，默认 5 秒；历史 JSON 快照不自动载入
在线地图，防止重启后复活陈旧对象。时钟回退需重启节点。

|接口|类型/作用|
|---|---|
|`/semantic/objects/observations`|String，对象观测 v1|
|`/semantic/objects/map`|String，活跃 map 对象快照|
|`/semantic/objects/markers`|MarkerArray，map 中的包围盒，可在 RViz 检查|
|`/semantic/command`|String，`version/task_id/text/dry_run`|
|`/semantic/cancel`|String，`version/task_id`，只取消对应活跃任务|
|`/semantic/status`|String，解析、候选、选择、导航及终态|
|`/semantic/selected_goal`|PoseStamped，可视化接近目标|

topic/action/service/frame/几何/阈值集中在 `CASE.json.navigation.semantic`。
指令“到最近的推车那里”“去纸箱旁边”等解析为单目标 navigate_to_object。
现已扩展最远对象及带排号货架参照的关系查询，详见[关系指令说明](JETBOT_SEMANTIC_RELATIONAL_20261112.md)。
不支持的组合、否定或多步骤文本拒绝，不使用 LLM 猜测。UUID 关联结果，重复 ID 不重复执行。
忙时返回 BUSY，不排队替换当前任务。客户端断线不会自动取消 ROS 任务；Ctrl-C/超时
主动发出对应取消请求，必须看到终态才算取消确认。

## 接近点与可达性

在对象盒子周围生成若干方向候选。要求中心到盒子至少
`规划半径 .40 + padding .01 + 接近额外余量 .60 = 1.01m`。
.60 是语义靠近任务的独立余量，不是修改 Nav2 膨胀；不是要求贴物体停在 5cm 处。
首轮 .25 额外余量出现连续任务无法出发、接近阶段进度失败，保留原记录；
本候选把语义停靠点移到更低的障碍软代价区域，另要求目标 cost ≤20。
终点面向对象中心，不把物体中心作为机器人目标。

三维盒子向 XY 作保守投影，再生成二维接近目标；不是丢弃高度尺寸后直接使用
原始宽长。倾斜物体的高度也会贡献平面投影。当前 oracle 原生输出 AABB，
此投影对它保持原大小。高处标牌等投影也按保守几何处理，当前未实现按机器人
高度分离悬空物体与地面障碍的语义碰撞优化。

候选先按 global costmap 排除高软代价和未知/inscribed/lethal 单元，再调用真实
ComputePathToPose（GridBased，use_start=false，机器人起点来自估计 TF）。
验证返回路径终点距候选 ≤5cm，防止 Navfn tolerance 只给出邻近替代点。
以 2.5cm 步长复查整个路径的代价和已登记对象包围盒，拒绝穿物体的路径。
最近查询按返回路径长度取最短，确定性打破并列；最远查询先取每个对象的最短安全接近路径，再按对象比较。
这不是欧氏最近，也不是把所有目标综合风险量化后的全局最优性保证。

只查询 native path，不修改冻结规划器或静态栅格，也不往 costmap 注入语义标签。
尚未被 LiDAR 看到的 oracle 障碍可能使现有规划器给出穿物体的路径，本层会拒绝，
不保证能搜索到所有理论可行绕路。标签代价层融合应作为后续独立工作验证。

最终候选直接把选定接近位姿交给 Nav2。中途曾试过固定当前XY的附加朝向目标，
但实际连续任务出现了XY修正循环，已撤回；它不是当前执行流程。

启动至少等10仿真秒，并要求 clock/odom/scan 接收持续健康2墙钟秒，再报告 READY。
这不放宽门控阈值，不修复/清除估计器故障；发现 ODOM_FAULT 就拒绝启动任务。
最终执行仍走 native NavigateToPose，开启输出前检查 arm 返回值。
取消/目标过期/移动超过15cm、旋转超过.2rad或尺寸变化超过10cm时，先 disarm，
再取消 action 并等终态。安全门 fault 不重置、不自动恢复、不切控制源。
不会发布 Twist，也不会绕过现有 mux（若启用）及 sole cmd_vel 安全门。

## 使用（云端 admin，纯仿真）

先把 `isaac_sim/jetbot_fastlivo` 放到一个新代码目录，并把 Python 模块目录
`ros2_ws/src/jetbot_semantic_nav/jetbot_semantic_nav` 放入该目录。
本轮独立部署目录会在运行记录中列出。保留既有依赖，不重装定位后端。

```bash
CODE=/home/admin/jetbot-warehouse/semantic_20261112_07/code
bash "$CODE/manage_jetbot.sh" status
# 确认无其他实例后，始终换新ID；这是有头、录像、无Depth配置。
bash "$CODE/manage_jetbot.sh" start UNIQUE_NEW_ID 600 operator stationary \
  "$CODE/warehouse_semantic_oracle_20261112.json"
```

另一个云端 admin 终端：

```bash
source /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_stage2/nav_env.sh
export ROS_DOMAIN_ID=51
CASE=/home/admin/jetbot-warehouse/artifacts/UNIQUE_NEW_ID
export PYTHONPATH="$CASE/source:$PYTHONPATH"
# 先等 SEMANTIC_STATUS.json=READY，数据阶段检查 SEMANTIC_OBJECTS.json。
/home/admin/isaacsim/python.sh -m jetbot_semantic_nav.cli --case "$CASE" \
  --dry-run '到最近的推车那里'
/home/admin/isaacsim/python.sh -m jetbot_semantic_nav.cli --case "$CASE" '到最近的推车那里'
# Ctrl-C 发出取消，或在另一终端用原任务ID：
/home/admin/isaacsim/python.sh -m jetbot_semantic_nav.cli --case "$CASE" --id ORIGINAL_ID --cancel
```

supervisor 已启动 mapper/manager，不要再运行 launch，也不要同时使用旧submit_goal/operator_session提交任务。
标准 ament_python 包也可在匹配的 ROS2 Humble 环境构建；独立
`ros2 launch jetbot_semantic_nav semantic_nav.launch.py case:=...`
仅用于尚未启动这两个节点的既有仿真，节点持有 case 独占锁防重复。
oracle 由仿真内启用，launch 不会另开 Isaac。

## 本地验证

```bash
PYTHONPATH=ros2_ws/src/jetbot_semantic_nav python3 -m unittest discover \
  -s ros2_ws/src/jetbot_semantic_nav/test -v
```

最终06真实仿真和域52接口验证已完成，逐轮失败、最终误差及证据路径见
[验收记录](JETBOT_SEMANTIC_VALIDATION_20261112.md)。所有会话已停止；以上命令供后续新case复现。

对象入口及活跃查询一致允许最多.1s消息时钟先后偏差，避免浮点取整导致新对象误删；
仍按5s仿真年龄和接收墙钟年龄过期，不改变安全门时钟规则。
此交互配置关闭原600墙钟秒自动退出的只读controller_probe，防止探针结束带停长会话；
冻结的Nav2参数、其他诊断与保护阈值保持一致。
