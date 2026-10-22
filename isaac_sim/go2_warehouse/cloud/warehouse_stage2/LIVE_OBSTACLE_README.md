# 实时障碍主线（2026-10-19）

用户最新要求：先跑通导航主线，不扩展FAST-LIO精度支线，不以额外速度/终点精度统计门槛阻塞流程。保留碰撞、数据失效停车、唯一TF发布者。暂缓运动中突然封路。

`prepare_live_obstacle_copy.py` 从已通过的GPU物理仓储副本派生新的 `motion_live_*`，拒绝覆盖。默认从 `motion_obstacles_20261018` 派生；开阔区导航使用 `--source motion_open_obstacles_20261018 --target motion_live_open_20261019`。原work/baseline和原扫描配置保持。

`live_rtx_points.py` 每个渲染帧读取原生ReadRTXLidarData，根据原生帧末变换累积最近五个连续20ms包，再投影到当前雷达帧。间断时丢弃旧组；不保留历史物体点。用于障碍导航，不宣称严格逐射线去畸变，也不改变FAST-LIO输入。日志同时记录旧ScanBuffer与新点云箱体回波数，用于解释残留来源。

`scan_converter_live.yaml` 沿用原高度带、量程、坐标与时间配置，将LaserScan角分辨率从1度改为0.25度，以减少栅格清除射线空隙。没有调用clear_costmap服务，没有添加假障碍回波或改先验地图。

`check_live_obstacle_flow.py` 使用已有单例仓储/scan/AMCL/planner，静止验证：自由路径、实体障碍标记/绕行、目标落入障碍时规划ABORTED且无路径、移除后地图清除与同一目标重新可达。它不发速度，不等于双通道全封的完整NavigateToPose结束验证。

`check_live_navigation.py` 复用已有开阔区实际绕行观察器，NavigateToPose原生成功与运动安全作为功能依据，巡航、驻足、额外终点误差检查只记信息项。保留现有Nav2原生goal checker配置；不为通过试验调宽到点范围。保护几何使用height_clipped/static_boxes.json加本轮实际箱体。

云端使用已有manage_warehouse.sh、manage_scan.sh、manage_localization.sh、manage_obstacle_plan.sh、manage_velocity_guard.sh与manage_navigation.sh，禁止重复实例。每轮artifact必须全新；历史实际启动脚本归档在artifacts/live_obstacles_20261019/source，固定case不可原样重跑。
