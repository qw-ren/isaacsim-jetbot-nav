# 开阔带A类实验与高度保护投影

报告：docs/GO2_WAREHOUSE_OPEN_OBSTACLE_A.md。仅Isaac仿真；不训练、不上真机。

refine_guard_height.py从同一warehouse USD读取实际碰撞面，在原z=.05–.85m高度带裁剪后计算保守AABB；非Mesh/非原始三角碰撞近似保留旧边界。输出static_boxes.json、changes.json和report.json。该文件只供保护桥，不能作为SLAM地图。生成是一次性CPU操作，manager不会每次重算。

已验证数据在 `/home/admin/go2-warehouse/artifacts/m4_open_a_20261018/height_clipped`。保护距离仍.55m，预测1.5秒，所有TTL、scan和姿态门槛不变。1077对象全部保留，80处XY范围修正，旧危险箱体轨迹仍被拦住。

新独立运行副本motion_open_obstacles_20261018由prepare_open_copy.py生成，拒绝覆盖。open_obstacle_fixture.py从本轮仿真artifact目录读取box_config.json，主线程创建固定静态箱体；启用状态仍通过obstacle_request.json设置并写obstacle_state.json。先写配置再等待仿真就绪。原固定位置副本不变。

本轮配置：起点(1,-1.3)、箱体(4.3,-1.3,.5)、尺寸(.6,.6,1)m、目标约x6.5/y-1.3。map_server/AMCL/scan和单例manager沿用。Nav2使用nav_obstacle_plan_safe.yaml、nav_obstacle_motion_arrival.yaml及navigate_valid_path.xml。保护桥通过GO2_GUARD_BOXES显式使用“高度裁剪静态几何＋本轮实体箱体”文件；**不能用旧整块高构件AABB替代本轮数据**。

check_open_obstacle_navigation.py保留旧严格停车后精度验收。check_open_mission.py按既有Nav2 stateful xy=.20m/yaw=.50rad判定任务，停车5秒后的XY/yaw及驻足精度作为信息项保留；巡航实测>=.6m/s、实际净距、保护不锁停、地图不变、单一定位源和解除后的零目标速度仍是必要条件。

首次生命周期服务响应丢失与观察器缺省fixture错误、case02观察器对sim=None的汇总错误均保留。修正后先保存原始telemetry及action_result_before_metrics，再算结果，防止统计异常丢失证据。GetState最多3次只读重试，原生ManageLifecycleNodes启动操作不重复发送。

完整启动脚本go2_run_open_a*.sh保存在证据目录，固定输出目录不可直接重跑覆盖。生成新case目录后再复现；停止用各manager，只停止本轮会话，不操作电源。旧原始默认配置未被这些实验变体替换。
