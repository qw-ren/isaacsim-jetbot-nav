# 0.40m预测锁停诊断，2026-11-01

历史诊断记录；当前采用的研发冻结版见[JETBOT_NAV_PARAMETERS_20261101.md](JETBOT_NAV_PARAMETERS_20261101.md)。本文件不覆盖后续障碍软权重0.40的独立验收，也不把06根因未知改写成已解决。

结论：未复现06的预测间距锁停，不能确定0.40m是根因，也没有实际碰撞证据。新增诊断后08完整B→A成功；既有06失败继续保留，不以本次成功覆盖历史，也不宣称候选已通过全部回归。没有修改半径、速度、定位、容差或安全门判据，没有发布新的冻结版。

用户授权继续核对“控制器为何接受、安全门为何拒绝”。独立云代码`radius_20261101/diagnostic_code_01`来自未改动的`release_20261030/code`。新增内容仅为诊断：

- `velocity_gate.py`可选记录每次原始和限速后候选指令；第一次预测拒绝时保存scan点集、预测路径、时刻及门限到内存，停车退出后写`GATE_REJECTED_PREDICTION.npz`。本例没有拒绝，因此没有该文件。
- `controller_probe.py`记录每次收到的选中轨迹与评分（32MiB有界），以及约5Hz全候选评分（64MiB有界）；原有short-circuit保持true，因此部分落选候选评分可能不完整。接收记录不保证捕获每一个控制周期。
- `supervise.py`增加可选探针；基础`nav2.yaml`声明两个调试参数false，诊断JSON将`debug_trajectory_details`、`publish_evaluation`设true。保留原评分、速度和安全规则。调试发布仍可能影响运行负载。
- 外部有界`local_costmap_probe.py`只调用GetCostmap服务，约每0.5壁钟秒保存原始uint8局部地图、坐标系、分辨率、原点及时间，检测本例结束后退出；不发布指令、TF或truth。08运行至约42s时附加，不覆盖前半程。

门控验证：本地AST提取新旧tick，在安全通行、预测拒绝、轮速耦合限速、scan超时、非有限指令五类情形，分别启用/关闭诊断，发布输出/故障/clearance完全一致；拒绝快照可重新计算同一最小距离。语法、diff空白检查通过。诊断行为未改变门控公式或保护阈值。

07启动前因两个调试参数尚未在基础YAML声明，被`prepare_fixture_prior`严格校验拒绝。无PROGRESS、无运动、无RUNNER_RESULT；runner.log及LAUNCH_FAILURE_DIAGNOSIS.json保留。只在独立副本补声明后以新目录08重跑，未覆盖07源码。

08：原生4，输入审计通过、门控无fault且退出归零、runner和15个受管理子进程均0；外部地图探针也正常退出。实际路线45.684m，B/A两个事件完整且独立SHOWCASE几何审计通过。真实停车6.522cm/1.742°，原始AMCL样本XY RMSE1.908cm/P953.369cm。静态/A/B减实体保守圆的AABB近似最小净距47.750/41.981/239.383cm；这不是接触力认证或5cm窄隙验收。

探针结果：2461条收到的选中轨迹、384组采样全候选评分、299张局部地图，两个评分日志均未截断、探针无error。全程安全门最小预测中心间距0.730310m，未接近06记录的0.352873m；本轮没有触发同一故障，无法锁定具体critic。

运行参数读取确认`dwb_plugins::StandardTrajectoryGenerator`、`sim_time=1.5`、`acc_lim_x=.8`、`limit_vel_cmd_in_traj=false`。按[Nav2 Humble源码](https://api.nav2.org/nav2-humble/html/standard__traj__generator_8cpp_source.html)，标准生成器由当前估计速度按加减速限制推进；现有安全门由候选速度立即生效的恒定指令圆弧预测1.5s。两者同一预测时长不意味着同一轨迹。

离线将每条DWB选中轨迹转到其首姿态局部坐标系，并与相同选中速度1.5s恒定模型的终点比较：2461条的终点差中位数0.177cm、P951.919cm、最大31.460cm。最大样本stamp25.090s、候选约0.7667m/s和-0.6rad/s：DWB终点约[.6939,-.4149]m，恒定模型[1.0009,-.4835]m。这是两个模型的预测差异，不是真实定位误差；没有将该样本与06的被拒绝指令画等号，也不证明任一碰撞检查错误。不同障碍表示、时间基准等因素仍可能参与，尚无单因素因果验证。

按此前“未复现先不扩展”的范围，本轮不继续追加复现，不擅自放松门限、自动恢复、改变critic权重或把半径加大来掩盖问题。0.40保持候选；0.65旧冻结版仍可复现。诊断代码保留，后续若重现可记录被拒绝指令与点集，而不是再凭停车后的距离猜根因。

小型结果已下载：`experiments/jetbot_radius040_20261101/VALIDATION08.json`、`GEOMETRY08.json`、`MODEL_COMPARISON08.json`。后续用户恢复Master，已完成完整诊断归档`radius040_diagnostic_selected_20261101_01.tar.gz`，14,656,919字节，SHA256 `01abeb51ddf25010fb89cdc753e443da99620b4b9f5c0614fd74599c290c659e`，本地备份目录`data/cloud_warehouse/jetbot_radius040_20261101`。696文件逐摘要验证通过；包含07启动失败、08成功、诊断源码和局部地图，未覆盖前六例归档。97个原冻结文件（包括priors子目录两文件）与清单全部匹配；实时检查无本轮仿真/控制/定位残留。没有操作电源或关闭用户Master。

后续继续任务单独开展`limit_vel_cmd_in_traj=true`单项候选，见`JETBOT_CMDSTEP040_VALIDATION_20261101.md`；不改变本报告关于06根因未知的结论。
