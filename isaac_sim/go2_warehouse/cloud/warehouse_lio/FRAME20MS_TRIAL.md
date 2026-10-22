> 历史传感器接入记录：本文件旧“当前/下一步”以其日期为准。LIO已于2026-10-21完成独立116.645m导航，最新状态见[独立LIO说明](README.md)；LIVO另存相邻目录，不覆盖本记录。

# FAST-LIO2 隔离接入：20ms 分片采集时间近似

2026-10-19 首次实际运行成功。正式结果见负责人交接报告 `docs/CLOUD_WAREHOUSE_FASTLIO_INTEGRATION_M2.md` 与 `artifacts/lio_integration_20261019`。不是生产导航里程计切换，也不代表严格逐射线时间校准通过。

## 输入与误差边界

保留原生Hesai_XT32_SD10、Go2基座原点的原生200Hz IMU、原rough权重。因GPU物理原生IMU有设备读数冲突，独立副本用CPU物理/CPU推理，RTX仍GPU；不改Isaac/CUDA/PyTorch。相机未用于LIO。

独立已知平面标定中，连续逐点插值/帧末模型未达到严格区分判据。为尝试算法，另开 `/lio/points_frame20ms`：每个原生render packet的点使用对应历史渲染帧的采集末时刻，五个连续片段合成10Hz PointXYZIRT，偏移为0/.02/.04/.06/.08秒。**20ms是名义渲染片段宽度，不是已证明的逐射线时间误差物理上界。** 初次gate/log中bound字样仅指量化档宽；后续源码改用packet interval/assumed uncertainty并明确strict_uncertainty_bound_validated=false。

XYZ由同一原始azimuth/elevation/distance生成，ring使用真实channels，intensity保留原值；不使用真值位姿去变换输入点，不伪造连续发射时间。native deltaTimes仍保存在标定证据中，但不作为本试验测量时间。不发正式 `/lio/points`。Velodyne仅是FAST-LIO2字段解析格式，不是替换传感器型号。

make_frame_trial_gate.py要求显式 `--accept-frame-quantization-ms 20`、解析靶范围误差/静止运动覆盖及所有时钟位姿核查通过。strict_per_ray_calibration_passed仍false，与frame20ms_trial_allowed分开保存。输入流只给隔离算法，production_odom_switch_allowed=false。

## 运行与隔离

prepare_plane_audit_copy.py/prepare_frame_trial_copy.py仅创建新副本，目标存在时拒绝覆盖。先查单例manager和tmux，使用现有manage_warehouse.sh。靶标在启动前固定并加入guard_boxes.json，原.55m保护、预测、TTL和watchdog不变。

运行脚本保存在完整证据case目录。先启动只读check_lio_trial.py；sensors_ready后才启动算法，lio_ready后check_motion.py才通过原速度保护桥行走。正常巡航目标.8m/s。算法必须保留下面的隔离remap：

```bash
# 云端admin：已有独立评估前缀；使用nav_env，不能改全局LD_LIBRARY_PATH。
source /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_stage2/nav_env.sh
export LD_LIBRARY_PATH=/home/admin/go2-warehouse/nav_stage2/deps/fastlio_eval_20261019/lib:$LD_LIBRARY_PATH
/home/admin/go2-warehouse/nav_stage2/deps/fastlio_eval_20261019/bin/fastlio_mapping \
  --ros-args --params-file /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_lio/fastlio_frame20ms_trial.yaml \
  -r __node:=fastlio_frame20ms_eval -r /Odometry:=/lio/odometry \
  -r /tf:=/lio/evaluation_tf -r /tf_static:=/lio/evaluation_tf_static
```

上面是算法入口参考，不能未检查实例就重复启动。common.time_sync_en=false，offset=0，外参T(.2,0,.2)/R单位阵，禁用在线外参估计。噪声参数沿用上游默认，未做IMU噪声标定；PCD/路径/无关点云输出关闭。

## 软件与复现

FAST_LIO ROS2 revision a4743b095409588842a5b30ddfa27e29d2f99164，清洁git archive编译；只修改CMake使ROOT_DIR可指向云端可写日志目录，C++算法未改。本地已有Livox类型支持和PCL/VTK等49个运行库复制到独立数据盘前缀，没有apt/pip安装或系统库替换。20,404,031字节最小运行包的SHA为8d18c0e8de0f165847954c1c167c8e13b7bcfdb9639a31ddd91e1196014f8a42，79项文件校验通过；首次缺livox CMake prefix的配置失败日志保留。

24字节点步长由显式结构体数组保持，不能用会剥离尾部填充的np.concatenate结果直接序列化。真实ROS消息同时验证字段、字节长度、有限值、5档偏移、ring、频率和TF隔离。首轮22/24字节不匹配失败完整保留。

analyze_lio_trial.py仅离线读取真值，使用首次位姿单次SE3对齐，不拟合整条轨迹或尺度。下一步独立冷启动、转弯、较长路线和时间近似敏感性评估；通过前不切换Nav2/AMCL/保护桥的odom/TF输入。
