# M3：先验地图上的实时障碍

仅仿真。`nav_obstacles.yaml` 在原 StaticLayer、InflationLayer 之间增加现有 Nav2 ObstacleLayer，观测 `/scan`；不改变先验PGM/YAML、robot_radius=.45m、膨胀=.70m、unknown不可通行或原DWB速度配置。

`prepare_obstacle_copy.py` 从已验证的 world-LiDAR 运行副本派生 `motion_obstacles_20261018`，拒绝覆盖。添加的 `m3_obstacle_fixture.py` 只在仿真主线程响应本次 artifact 目录下的 JSON 文件，控制固定箱体：中心(4,-1,.5)、尺寸(.6,.6,1)m。新版移除操作实际RemovePrim；第一版隐藏后残留回波的证据保留。

启动前确认无其他仿真、SLAM、AMCL或planner。继续使用已有manager单例保护；先启仿真（spawn=(2,-1)、freq=50、`++publish_map_to_odom=False`、限时900秒、全新artifact目录），再启scan_converter_50、AMCL（`localization_start2.json`），最后：

```bash
bash /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_stage2/manage_obstacle_plan.sh start
```

不得同时启动原manage_plan或manage_localized_plan；不得为本轮检查启动controller、BT或速度桥。`check_obstacle_costmap.py` 只请求 ComputePathToPose，不请求 NavigateToPose。

在云端admin已source nav_env.sh的环境中：

```bash
/home/admin/isaacsim/python.sh "$GO2_CLOUD_ROOT/nav_stage2/cloud/warehouse_stage2/check_obstacle_costmap.py" \
  --output "$GO2_CLOUD_ROOT/artifacts/NEW_RUN/check" \
  --fixture "$GO2_CLOUD_ROOT/artifacts/NEW_RUN/simulation" \
  --map "$GO2_CLOUD_ROOT/nav_stage2/maps/slam_m2_20261015_v1/map_nav.yaml"
```

output必须为新目录；fixture必须等于本轮运行仿真的artifact目录。检查顺序：初始自由/直线路径、实体箱体出现/致命表面与膨胀/绕行路径、实际移除/清除/规划。每阶段保留实测网格、scan、路径NPZ；未使用clear_costmap服务帮助通过。几何路径净距以箱体外接圆测量，是保守下界。

收尾依次正常停止obstacle_plan、localization、scan和本轮warehouse；确认会话、进程消失，归档校验，不操作云电脑电源。

规划绕行成功仅表示M3感知与规划链路；A/B/C真实行走另在M4验证。

10月18日结果：标记/规划链路已验证；移除回波残留未解决。keepOnlyPositiveDistance=False诊断失败，未采用。三轮均有失败项，不宣称完整M3通过。
