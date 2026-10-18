# B：已知隔断的双侧通路与新增封路（纯仿真）

2026-10-18：SLAM 两轮各13项通过；静止 Navfn 对照11项通过；随后实际下侧改道17项通过。巡航 .713683m/s，最小中心AABB净距 .634036m，无保护锁停。

本轮是静止时感知新障碍、重新规划、随后行走；不声称运动中突发封路重规划或独立冷启动复测已通过。场景是原仓储内增加的可控隔断，不是现有多货架跨通道通用验证。

## 场景与地图

独立 runtime：`/home/admin/go2-warehouse/motion_branch_obstacles_20261018`，由原 `manage_warehouse.sh` 单例启动；work/baseline/A副本不改。`prepare_branch_copy.py` 在目标存在时拒绝覆盖。

固定先验隔断中心(4.3,-1.3,.5)，尺寸(2,.4,1)m。`branch_obstacle_fixture.py` 总是创建该物体；新增障碍单独由 artifact/obstacle_request.json 控制。新障碍中心(4.3,1.4,.5)、尺寸(.6,5,1)m，接在隔断上侧。测试模块最大尺寸6m只是参数有效性边界，保护阈值不变。

`collect_branch_prior.py` 从(1,0)沿上侧采集，`collect_branch_lower.py` 从(1,-2.7)用原生 DeserializePoseGraph START_AT_GIVEN_POSE 接续首轮图，再采下侧。种子使用已知仿真配置下的 scan_ground 位置(1.2,-2.72,0)；建图输入仍是模拟真实 odom，不能声称独立LIO精度已验证。

最终保存地图 `nav_stage2/maps/slam_b_prior_20261018_v2/map_nav.yaml`，313×489、5cm，63655自由、2013占据、87389未知。原PGM像素不改；prepare_amcl_map.py 将free_thresh改为.196并保留精确原点，导航解析与实时SLAM逐格一致。正式导航只有map_server+AMCL，不运行SLAM或Bridge map→odom。

## 使用约束

- 管理器启动前查状态。所有副本共用唯一 go2-warehouse 会话，停止请求后确认退出才重启。
- 用GO2_SAVED_MAP显式选择B地图；GO2_INITIAL_POSE必须引用来源以SLAM_开头的一次性已知起点种子。标签不符会拒绝，勿绕过检查。
- GO2_PLAN_PARAMS=nav_obstacle_plan_safe.yaml；GO2_NAV_PARAMS=nav_obstacle_motion_arrival.yaml；GO2_NAV_BT=navigate_valid_path.xml。
- GO2_GUARD_BOXES必须使用高度裁剪1077静态对象+真实先验隔断+当前新增障碍。真值几何只作独立安全保护，不作为Nav2输入。原.55m/1.5s预测/scan/TTL/姿态限制不变。
- `check_branch_plan.py`不发速度，只调用原生ComputePathToPose；`check_branch_navigation.py`才发送真实NavigateToPose并显式启用已有保护。后者是实际运动命令，不可当只读检查。
- 本轮保存的go2_run_branch*.sh固定case目录，仅供复现参考；再次执行必须新建case并根据当前已存在runtime跳过创建步骤，不能覆盖原证据。

## 记录

原始目录 `artifacts/m4_global_b_20261018`：prior_01、prior_02、plan_01/planning、plan_01/navigation_01。

先验上侧路径5.567m，封路后下侧9.131m；两次action均SUCCEEDED，地图不变。原检查ROI与旧隔断相交，32→114致命格；离线补充验证增长82格通过。后续观察器已将该条件强化为增加至少20格；实际执行旧源码保留在executed_source。

实际路径在x4–4.6m段的y范围[-2.69251,-2.59711]，动作SUCCEEDED；巡航540样本、AMCL83更新。观测进入.197625m目标范围，成功时yaw .495207rad，沿用Nav2 stateful xy=.20/yaw=.50。动作返回XY误差 .266815m、5秒后 .263496m/yaw .701357rad；严格停车精度仍未通过，信息项保留，不改写成精准到点。解除后零目标速度通过。
