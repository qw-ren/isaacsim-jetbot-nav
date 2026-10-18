# RTX 点云运动坐标诊断与世界坐标缓冲对照（2026-10-15）

## 本轮范围

接续用户“继续”，优先验证动态LiDAR，而非继续调驻足门槛。baseline/work、Isaac/CUDA/PyTorch、权重、执行器参数均保留；新副本只由既有受保护运行副本派生。未进入SLAM/AMCL/JetBot/训练。

## FULL 与完整索引对照：辅助字段仍不可信

读取本机isaacsim.sensors.rtx/config/extension.toml，默认app.sensors.nv.lidar.auxDataOutput=EXTRA。参考NVIDIA对4.5的[辅助输出建议](https://forums.developer.nvidia.com/t/missing-object-id-information-from-rtx-lidar-omnigraph-node-rtxsensorcpuisaacreadrtxlidardata/327961)，在独立motion_lidar_full_20261015的进程启动参数中设FULL，不改安装目录。静止、无外部速度源，1.01/1.03/1.05秒各保存一帧。

FULL使非零单位法线比例从0变为100%，但timestamp仍出现2^64-1，按objectId解析为ground的点只有44.4%在地面±5cm内。不能用单位法线比例证明数组与点正确对应。

再派生motion_lidar_unfiltered_20261015，keepOnlyPositiveDistance=False保留128000条完整记录，Bridge显式distance>0才发布，尝试排除压缩索引影响。仍出现异常时间戳和ground ID几何不一致，故此假设没有解决问题。两组辅助输出均拒绝用于物理命中结论。具体审计：data/cloud_warehouse/m1_lidar_aux_full_20261015/audit_full_vs_unfiltered.py和full_aux_audit.json。

NVIDIA也有[ScanBuffer时间戳数量问题记录](https://forums.developer.nvidia.com/t/rtx-lidar-scan-buffer-timestamp-count-is-half-of-the-point-cloud-size/354993)，但其版本/症状不完全相同，不能直接断定本机遇到同一缺陷。未升级或替换引擎。

## 世界坐标缓冲方案

原实现transformPoints=False，整圈缓冲的局部坐标被Bridge标为发布时LiDAR坐标，之后scan转换用发布时TF。它没有证明运动中各点与该TF一致。

独立motion_lidar_world_20261015从motion_work_20261015派生：

- 仅将RTX ScanBuffer设transformPoints=True，先保留世界坐标点。
- 根据同一仿真循环的实际机器人位姿与已验证(0.2,0,0.2)外参，将点转回当前unitree_go2/lidar_frame再发布；topic、TF所有者及scan过滤线均不变。
- 不使用异常的per-point timestamp/objectId/normal；不把传感器坐标转换冒充SLAM定位。
- 该方案对静态仓储环境有效性单独验证，不声称修复所有动态物体/每点去畸变问题。

新增world_lidar_adapter.py、prepare_world_lidar_copy.py；实验记录world与sensor两种数组、当前root状态及RTX给出的transform。后续独立重放由prepare_world_stance_copy.py建立，复用旧calibrated_reference_v3控制器，单独改变点云链路。该控制器此前严格驻足失败，不能作为已通过控制器。

## 首轮实际结果

固定起点0.8指令直行3仿真秒、零指令5秒，check_motion旧接口9项通过：实际均速0.772166m/s、位移2.202308m、零反馈延迟0.085805墙钟秒；但身体触发后净移动0.147786m。这是接口与有限停车检查，不是12秒/5cm严格驻足通过。

并行复用原站立scan观察器取得370组同stamp数据：投影、单scan发布者、预期TF、有效范围、地面排除和低处点保留等10项通过。它的“无外部速度源”“静止2cm内”两项按预期失败，整体passed=false保留；未将其伪装成移动验收。新增check_scan_motion.py明确要求--allow-motion、唯一速度发布者为go2_sim_velocity_guard、真实位移>=0.5m且至少10条实际速度>=0.6m/s，再检查原投影/TF/有效范围等条件，供后续独立重放使用。

世界点静止/运动/刹停18帧已归档。初始静止地面ROI误差约毫米，运动3.03秒对应地面ROI最高约2.6cm，未越10cm过滤线。ROI并非全为地面：4/5秒的高点经USD只读查询落在真实货架网格包围盒中，不能当作地面误投影计数。

## 旧锁停点的静态几何排除

inspect_lidar_geometry.py加载同一warehouse USD，检查1878个Mesh及原额外ground平面。旧触发点(3.716116,-1.366841,0.112832)离最近网格AABB为ground上方0.112732m，距仓储地面0.112832m，其余近邻更远，不在任何静态mesh包围盒±5cm内。它亦离当时机器人中心约1.06m，远于机器人本体。

这证明该点与当前静态几何不符，是坐标/缓冲链路异常的有力证据；尚不直接证明它的原始命中面就是地面。AABB包含也不代表精确三角面命中，几何检查报告保留此限制。

## 后续重放实时结果

待本轮受保护重放完成后补记，不提前宣布修复。

### 原锁停流程重放（19:00–19:03）

motion_lidar_world_stance_20261015保留原calibrated_reference_v3控制、权重和物理。check_stance结果中起点、5.07秒运动末端、17.09秒停车末端与旧失败流程逐项一致，严格驻足仍是相同0.145640m/0.146390rad，整体passed=false。恢复段现在成功移动1.387564m，实际均速0.799120m/s，没有FRONT_SCAN_CLEARANCE；未修改保护距离。

运动观察器check_scan_motion：902组同stamp配对、13项全部通过，实际位移覆盖3.324069m，单速度来源与预期TF通过。该观察器不发布速度；运动由已有受保护探针执行。

18.03秒原故障帧的body position三分量与quaternion四分量完全一致。旧front最小范围0.831944m，新world点独立重投影前方最小4.964920m；原故障XY±0.15m内562点高度[-0.001143,0.027838]m，中位0.020620m，全部低于0.10m，未被当作障碍。compare_fault_frame.py按float64归一化四元数作角差，避免float32归一化制造伪小角差。图fault_frame_comparison.png保留同位姿前后对照。

CONFIRMED：在已复现的相同运动状态下，更改缓冲坐标链路消除了原无几何对应的近地面锁停，恢复行走成功。结论限定此轨迹/静态仓储及当前帧转换；不是全运动范围、动态物体或SLAM定位已通过。另做完整停止/重启复测，结果在收尾补记。

### 完整重启复测与收尾（19:04–19:08）

全部停止后重启相同world/stance副本；manager曾因旧会话尚未退出而拒绝启动，随后确认旧会话消失再启动，无重复实例。第二轮运动scan 715组同stamp、13项全部通过；观察窗因启动等待没有覆盖整个恢复段，不能声称715组都覆盖恢复。完整check_stance单独记录恢复1.387564m、均速0.799120m/s，仍无前方锁停，12秒停车结果仍是相同0.145640m/0.146390rad失败。

两轮证明的是同一故障流程下的点云坐标修复及完整重启后可复现。没有放宽0.10–1.20m高度、0.30m近距或0.55+0.5vx保护距离，没有修改Nav2外部验收。尚不能将其计作通用转弯导航或严格驻足通过。

world_lidar_adapter.py当前源码使用每次--artifact-dir的sensor子目录，避免重启覆盖证据；首轮rough原始运行代码使用固定诊断路径，已作为runtime_original单独保存，后两轮runtime_stance使用新路径。收尾时已在停止状态将首轮rough副本的入口和适配器同步为每次artifact子目录方式，避免后续覆盖；仅日志路径与额外采样窗更新，数学链路不变，语法检查通过，原首轮代码在runtime_original，更新后在runtime_current_rough。该日志设置已随world_stance两轮实测，rough副本此次同步后未再执行运动。

19:08:21确认所有tmux为空，本轮仿真/scan/guard/观察器已退出，ROS CLI daemon未运行。未关机/重启、未改sshd或安装环境。baseline12项和稳定work三项哈希通过，当前59项源码manifest云端全通过（另加manifest自身共60文件）。原始数据、实际运行源文件、元数据和日志存于云artifacts与本地data/cloud_warehouse的m1_lidar_aux_full_20261015、m1_lidar_aux_unfiltered_20261015、m1_lidar_world_20261015目录；失败辅助字段和旧站立检查误用的失败JSON均保留。

阶段二未提交/推送。下一步在保留该点云修复的前提下继续Nav2转向/停车验证；独立驻足候选仍失败，不继续将其当作已通过控制器。动态物体的时间一致性、SLAM/AMCL和新增障碍任务尚未验收。
