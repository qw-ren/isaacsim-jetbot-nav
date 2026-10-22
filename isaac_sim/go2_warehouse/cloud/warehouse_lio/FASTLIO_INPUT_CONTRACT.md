> 历史传感器接入记录：本文件旧“当前/下一步”以其日期为准。LIO已于2026-10-21完成独立116.645m导航，最新状态见[独立LIO说明](README.md)；LIVO另存相邻目录，不覆盖本记录。

# FAST-LIO2 输入接入检查点（2026-10-18）

最新2026-10-19：算法已在[20ms分片近似隔离试验](FRAME20MS_TRIAL.md)中实际运行，155条里程计、15项接口/9项短直行通过，生产odom/TF未切换；严格逐点时间仍未验证。下文是早期历史记录。

2026-10-19最新进度见[RTX_TIME_AUDIT.md](RTX_TIME_AUDIT.md)：两次冷启动的时钟/帧末位姿核查通过，第二轮覆盖.779m/s巡航；帧内采样几何尚无独立证明，LIO输入与算法仍未运行。以下为10月18日初始检查点历史记录。

当前只完成原生IMU/RTX原始数据探针与运行库预检，**FAST-LIO2尚未启动，也未替换导航odom**。

阶段二已验证快照ae2e2dc、PR #2；本目录属于后续独立接入工作，不包含在该快照107文件清单中。行进中突然封路按用户最新安排暂缓。

## 已验证的IMU方案

- 原生IMU需在机器人USD生成、首次物理初始化前创建；动态挂载首轮3804次读取全部无效。
- GPU物理对照虽有200Hz消息，但出现getVelocities expected device 0/received -1，gyro与无重力加速度全0，因此判失败。仅检查频率不足以证明IMU有效。
- CPU物理独立副本motion_lio_cpu_sensors_20261018，使用相同rough权重（推理设备CPU），RTX仍用GPU。8220个原生样本、200.000002Hz，真实角速/加速度非零且变化；ROS观察2408消息、12.035秒、11项流检查通过。
- 原生含重力读数减去不含重力读数与传感器坐标重力向量的最大差1.37e-6m/s²；gyro z与真值odom均差.00406rad/s、P95 .01915rad/s。独立物理核查5项通过。
- 实际短直行.77921m/s、9项运动检查通过。不同物理设备可能改变动力学，不将本轮短直行外推为CPU Nav2 A/B复现。

IMU位置为Go2/base本体原点，无旋转；TF unitree_go2/base_link→unitree_go2/imu_link恒等。/lio/imu只发原生物理时间新样本，不给LIO传真实姿态（orientation_covariance[0]=-1）。每步读取后一个已完成物理步的原生时间，平均比sim.current_time落后5ms；/clock按渲染更新，ROS消息最多暂时超前最近clock15ms。协方差0表示未知，理想仿真读数不等于标定噪声模型。

## 尚未获准作为LIO输入的雷达时间

保留Hesai_XT32_SD10及原XYZ topic。额外RtxSensorCpuIsaacReadRTXLidarData探针能得到对应长度的azimuths/elevations/distances/channels/deltaTimes/intensities；channels确为0–31，deltaTimes约368–19997240ns。

原始绝对timestampNs与仿真时钟有本轮固定、跨启动不同的偏移：GPU两轮约4.60536s/4.62462s，CPU约4.90817s。不能直接把原始绝对时间当/clock，不能任意填time/ring，也不能简单把当前XYZ字段改名当作原始去畸变输入。

下一步捕捉每个原始render frame的起止世界变换、frameId、物理/渲染/时间节点同帧值，证明原始点时间与实际采样位姿关系。时间域转换须有可重放的依据；相对delta/tick规律也应核查。再生成Velodyne兼容PointXYZIRT（实际传感器仍为XT32）并验证唯一顺序、10Hz整圈、逐点时间、外参与运动几何。通过前不发/lio/points。

## 候选FAST-LIO2配置与隔离方式

上游hku-mars/FAST_LIO，ROS2分支，revision a4743b095409588842a5b30ddfa27e29d2f99164。只读预检本地既有Humble二进制，未修改算法源码。云端缺livox_ros_driver2类型支持、PCL common/io/filters直接库；未安装/重装系统包，也没触碰Isaac/CUDA/PyTorch。

输入候选/lio/points、/lio/imu；preprocess.lidar_type=2用于兼容字段，scan_line=32、scan_rate=10、timestamp_unit=0（秒），feature_extract_enable=false。由原生数据生成XYZ/intensity/time/ring后才采用；现有XYZ-only输入不能用。

IMU→LiDAR安装平移(.2,0,.2)m、旋转单位阵（FAST-LIO extrinsic将LiDAR坐标点变为IMU坐标）；首次禁用在线外参估计。common.time_sync_en=false、time_offset_lidar_to_imu=0，不用软件自动校时掩盖不明时间域。

先并行输出，/Odometry remap为/lio/odometry，/tf remap为/lio/evaluation_tf；不接入生产odom→base_link，不与AMCL/Bridge重复发布TF。关闭PCD累积保存、路径和无关可视化输出，避免占满磁盘。待传感器契约和运行库就绪后，再做独立初始化、静止/直线/转弯及真值误差评估。

## 文件与复现边界

prepare_lio_sensor_copy.py创建GPU诊断副本并在spawn时创建IMU；prepare_lio_cpu_copy.py派生CPU兼容副本。目标存在时拒绝覆盖；先查单例manager状态，再执行新case，绝不可并行启动。check_imu_stream.py只读，不发送速度；已有check_motion.py会真实启用保护后的运动。

完整证据artifacts/lio_sensors_20261018及负责人本地data/cloud_warehouse/lio_sensors_20261018；analyze_sensors.py离线核查物理读数与RTX原始时间。首轮观察器括号语法失败日志、GPU无效读数与CPU成功记录都保留。精确驻足继续作为历史问题，不因单次短直行停车通过就宣布解决。

接口参考：[Isaac 4.5 IMU节点](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/py/source/extensions/isaacsim.sensors.physics/docs/ogn/OgnIsaacReadIMU.html)、[RTX原始读出节点](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/sensors/isaacsim_sensors_rtx_lidar/node_descriptions.html)、[FAST-LIO上游](https://github.com/hku-mars/FAST_LIO/tree/ROS2)。旧ScanBuffer辅助字段失效仍以本项目实测为依据，不用论坛升级建议替换现有4.5镜像。
