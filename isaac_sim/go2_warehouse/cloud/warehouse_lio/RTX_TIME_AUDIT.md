> 历史传感器接入记录：本文件旧“当前/下一步”以其日期为准。LIO已于2026-10-21完成独立116.645m导航，最新状态见[独立LIO说明](README.md)；LIVO另存相邻目录，不覆盖本记录。

# FAST-LIO2 RTX 时钟与运动几何核查（2026-10-19）

本文件为时钟核查M1历史记录；后续FAST-LIO2实际隔离运行见[FRAME20MS_TRIAL.md](FRAME20MS_TRIAL.md)。

本次继续纯仿真 FAST-LIO2 接入。GitHub 阶段二快照仍为 ae2e2dc / PR #2，本次诊断代码未提交或推送；未开始行进中封路。**FAST-LIO2算法仍未启动，/lio/points未发布，导航odom/TF未切换。**

## 本次实时结果

SSH50796在认证阶段可达；用户交互恢复共享连接后，检查无旧实验会话，/home约48GiB空闲。新副本motion_lio_time_20261019继承已通过物理核查的CPU物理/CPU推理配置，同一rough权重、Hesai XT32、GPU RTX渲染和原生200Hz IMU。稳定work、baseline不改；使用既有单例manage_warehouse.sh，运动经过原保护桥，.55m净距、TTL、预测和watchdog不放松。

TimeAuditAdapter直接记录原始RTX frameId/timestampNs/transform/transformStart、每帧真实本体姿态（仅评估）、SimulationContext时间、timeline时间、core原生时间和dispatcher referenceTime对应的历史渲染时间。历史帧查询取法来自云端Isaac 4.5自带lidar_rtx.py。另存有界原始点数组和同render product世界ScanBuffer，诊断节点不发布点云、里程计或速度。

case_01初次分析600帧有1个启动哨兵：timestamp=0、points=0、单位矩阵，原始失败time_analysis.json与执行源码保留。修正分析器后仅明确排除此哨兵，不修改原始数据；599有效帧6项核查通过。采样窗口0.07–12.03s，正式运动13.97s后才开始，**没有覆盖巡航，不用作运动时序证据**。独立运动实测.777077m/s、9项通过，分别保存。

case_02完整冷启动，将采样上限延长至1800帧、点数组降采样为每25帧存一份。去除1个同类启动哨兵后1799有效帧，0.07–36.03s，覆盖139个速度>.6m/s的巡航帧。独立实际运动巡航.779209m/s、9项通过；最小中心静态净距2.186m，watchdog清零延迟.05464墙钟秒。此为短直行诊断，不是CPU Nav2复现或LIO精度验收。

| 检查 | case_01 | case_02 |
|---|---:|---:|
| 有效帧 | 599 | 1799 |
| 原始timestamp减env时间 | 6.284699991–6.284699992s | 4.583350021–4.583350022s |
| 单轮偏移极差 | <1ns | <1ns |
| core时间/历史render时间对env差 | 0 | 0 |
| 雷达帧末位置对真值最大误差 | 1.34e-7m | 2.46e-7m |
| 雷达帧末旋转最大误差 | 7.80e-8rad | 2.20e-7rad |
| 帧首位置对上一帧末差 | 0 | 0 |
| 顺序、原始时间增量、物理时钟、历史查询、帧末位姿、首尾连续 | 6/6 | 6/6 |

两轮证明原始时钟增量与渲染时间同步、首尾变换对应明确。跨启动偏移不同，再次排除固定常数软件补时方案。原始timestamp文档定义为帧首，transform为帧末，不能把两者直接同刻处理。timeline_time还与物理时间不同，保留其实际值，不混用。

## 尚未解决的帧内采样语义

case_02保存72个原始点数组快照。按原生azimuth/elevation/distance构造传感器XYZ，全部用帧末变换投到世界，与现有ScanBuffer最近邻P95的跨帧中位数约3.44e-6m；按deltaTimes在首末姿态之间插值后，该中位数约.00355m。启动姿态变化较大时，插值差异还更大。

**这仅是同一RTX输出链之间的几何对照，不是独立场景真值。** 它证明ScanBuffer表现为帧末变换，尚不能证明RTX射线实际上是逐点运动采样还是每个20ms渲染片段的同一时刻快照；也不能据此宣告原始deltaTimes错误。不能把ScanBuffer的最近邻吻合当成逐射线对应或LIO去畸变时间已通过。

因此本轮lio_input_approved仍false，不输出伪造的time/ring，也不使用自动time_sync掩盖问题。common.time_sync_en继续false。保留XT32与原生IMU；无需更换传感器型号或重装环境。

下一最小实验是在新诊断副本放置远离行走路径的已知平面标定靶，保证安全盒也显式包含该靶。短直行时直接以解析平面距离核查原始射线：比较帧末一次采样与逐点插值假设，并检查是否存在可配置的原生运动补偿选项。先确定实际采样时刻，再决定逐点时间换算；若证实为20ms瞬时片段，应忠实保留片段时间，不能假装成连续逐点扫描。通过后才聚合PointXYZIRT并隔离运行FAST-LIO2。

FAST-LIO2缺失的Livox类型支持/PCL运行库本轮未处理，未安装系统包，也未触碰Isaac/CUDA/PyTorch。后续算法输出仍先remap到/lio/odometry和/lio/evaluation_tf。

## 证据与状态

云端artifacts/lio_time_20261019保存完整原始数据与SHA清单，初次冻结307文件、219,921,494字节（约210MiB，之后另加实时verification.json）。本地data/cloud_warehouse/lio_time_20261019已续传完成并通过归档SHA和85项逐文件SHA校验，精选归档包含：85项、18,020,695未压缩字节，包括全部时间帧记录、IMU、运动、报告源码与7个代表性原始点数组。压缩约11MB；cloud_files.sha256记录完整云清单，selected_files.sha256明确本地子集。此前中断的逐文件备份只算部分文件，不能宣称完整210MiB已取回。

17:54实时收尾验证：baseline12、稳定work3、冻结stage2的107源码SHA全部通过；tmux为空、无实验/导航/LIO残留进程，云电脑仍开机，未操作电源。原复杂工作树暂存区未执行add/reset/commit/push。

参考：[Isaac 4.5 RTX节点定义](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/sensors/isaacsim_sensors_rtx_lidar/node_descriptions.html)、[IsaacReadTimes](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/py/source/extensions/isaacsim.core.nodes/docs/ogn/OgnIsaacReadTimes.html)；实测依据为本机云镜像随附的节点rst和lidar_rtx.py，不混用5.x接口。

续传完成：归档SHA256为468130aff4e02c94819cac1ffc68e6a0d36b78ded8dad359927ffa281332a83d，85项全部通过，实时verification.json已取回通过；case02的1799帧时钟/位姿检查已在笔记本独立重算通过。早期中断递归传输额外文件移至_interrupted_transfer并标记未验证，不混入精选证据。

运行库进一步只读盘点：本地二进制ldd传递闭包与云端loader cache/nav_env搜索路径对照，候选缺43个SONAME、75,571,184字节，包含PCL→VTK/OpenNI依赖；不是只有先前4个直接缺库。fastlio_runtime_closure.json保存各候选库路径、大小和SHA。尚未补库，dlopen类型支持依赖仍需另检，未替换云端系统库。
