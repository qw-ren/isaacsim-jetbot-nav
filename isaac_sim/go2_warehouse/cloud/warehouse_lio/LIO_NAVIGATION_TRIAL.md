> 阶段说明：下文为2026-10-21的LIO历史记录。“尚不接入LIVO”是当时决定；2026-10-22已在相邻warehouse_livo独立接入。实际部署统一原生IMU，历史有噪案例不覆盖。最新分别见docs/GO2_WAREHOUSE_LIO.md与docs/GO2_WAREHOUSE_LIVO.md。

# LIO 导航与有噪 IMU 评估

2026-10-21实测：`lio_nav_20261021_noise_long_05` 原仓储约24×39m、无新增障碍，两圈六目标全部原生SUCCEEDED；实际评估路程116.645m，在线有噪LIO水平误差P95 3.10cm/最大4.34cm/终点1.63cm。单次初始SE3对齐，不做全轨迹拟合。同点云与IMU采样时间配对回放P95：理想3.00cm、有噪3.02cm；不能声称比Bridge真值odom更准。导航及回放已停止。单次固定种子结论不外推到任意少特征仓库，当前没有触发LIVO2接入条件；JetBot准备暂停。

完整报告在原工程 `docs/CLOUD_WAREHOUSE_LIO_NAV_NOISE_M3.md`，云端完成记录 `artifacts/lio_nav_20261021_handoff_notes`。既有case不可覆盖；下面命令用于后续新case，不需要重跑本轮成果。

此入口在独立运行副本中将 `/lio/odometry` 接到 Nav2/AMCL/保护桥，旧稳定默认入口保留。传感器仍是原生 IMU 与 Hesai XT32，粗粒度20ms点时间近似仍须在实验报告中声明。仅仿真，不准备 JetBot，不训练。

云端，admin/root均可；每轮必须用新case ID，启动前拒绝已有仿真实例：

```bash
bash /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_lio/run_nav_trial.sh status
bash /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_lio/run_nav_trial.sh start NEW_CASE noise long
bash /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_lio/run_nav_trial.sh stop
```

监督在 `go2-lio-nav-task` tmux，原有manager负责单例组件，正常结束自动收尾。结果在 `artifacts/lio_nav_NEW_CASE`，保留源码快照、配置、传感器日志、原生动作结果和失败。退出SSH不会关闭任务或云电脑，不操作电源。

先等待实际传感器静止就绪，再启动FAST-LIO IMU初始化；初始化期间没有速度源解锁。接入层使用静止IMU与LIO姿态建立固定重力对齐、从连续LIO位姿计算速度，保留测量时间。Bridge真值输出隔离到 `/truth/*`，仅供独立评估记录器保存；LIO、导航驱动和保护桥不订阅真值。

AMCL初始猜测来自配置，具有显式不确定度；以估计odom时间戳发送，并取得AMCL姿态确认后才启动规划/控制。CAD碰撞盒位于map，通过AMCL的map←odom变换计算保护净距；不能假定任意LIO odom原点和方向等于地图。原.55m净距、1.5秒预测、前方扫描、输入/数据壁钟超时和失联停车均保留。

噪声在 `/lio/imu` 的加速度与角速度上加入；理想读数只送评估话题，姿态仍无效。白噪声量级参考BMI088，初始偏置和随机游走为明确的场景假设，种子/实际偏置写入 `noise_contract.json`。`imu_noise_model.py` 记录单位及离散化，不冒充硬件标定。

约1.5MB点云需要可靠传输。独立 `fastlio_reliable_20261021` 程序只应用 `fastlio_reliable_transport.patch` 的ROS2 QoS改动，LIO计算与预处理未改，旧库复用。保存构建源码、二进制SHA与部署校验，禁止用未说明的另一个算法版本替代。

评估记录器保存zlib压缩的ROS CDR SQLite。完成后，在独立domain47回放同一趟点云与理想/有噪IMU；锁防止两次回放并发，不发布运动指令。仅离线使用真值，单次初始SE3对齐，不拟合整条轨迹或尺度。比较绝对/相对误差和终点漂移；纯惯性积分只是无外部约束基线，不能把很大的漂移倍数当成超过原真值odom的“优化”。

现有地图为从USD碰撞盒生成的辅助CAD先验，不能称为SLAM成果。长程终点留出预测停车空间，路线长度是观察范围而非精度通过门槛。用户要求先比较LIO：若实际长程精度不足，再考虑LIVO2；当前不做JetBot迁移。
