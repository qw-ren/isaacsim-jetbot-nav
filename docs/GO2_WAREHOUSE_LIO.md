# FAST-LIO2：独立部署与导航基线

LIO已部署，输入只有点云与原生IMU加速度/角速度；Bridge位置/姿态真值隔离用于离线评价。配置保持common.time_sync_en=false，保留真实测量时间戳。20ms点时间仍为近似，没有宣称严格逐射线校准。

2026-10-21历史有噪实验：原仓储无新增障碍，两圈六目标全部原生SUCCEEDED，实际116.645m；在线XY P95=3.10cm，最大4.34cm，终点1.63cm。同数据原生/有噪IMU回放P95=3.00/3.02cm。单次初始SE3，不做整轨迹/尺度拟合。完整原始结果见[独立LIO实验记录](../experiments/go2_warehouse_lio_20261021/README.md)。

后续实际部署统一原生IMU，历史人工噪声仅用于已保存记录的比较。原生不等于零误差：物理采样、点云时间近似、外参及估计误差仍存在；不将精度外推真机。

[LIO代码及入口](../isaac_sim/go2_warehouse/cloud/warehouse_lio/README.md)与[LIVO代码](../isaac_sim/go2_warehouse/cloud/warehouse_livo/README.md)分开保留。LIO改动包括可靠ROS2点云传输和导航适配，不修改原LIO数学更新。依赖、权重、完整CDR/点数组留外部存储，不要求重装Isaac/CUDA/PyTorch。
