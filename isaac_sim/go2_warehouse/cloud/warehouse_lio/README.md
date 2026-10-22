# FAST-LIO2 独立基线

本目录只负责LIO传感器适配、独立回放和导航接入，不含LIVO视觉融合。已部署与116.645m历史验证见[LIO报告](../../../../docs/GO2_WAREHOUSE_LIO.md)及experiments/go2_warehouse_lio_20261021。2026-10-22后实际部署策略统一原生IMU，历史noise案例不可改成无噪结果；此目录旧文档中的“不接入LIVO”是当时阶段决策。新LIVO放在相邻warehouse_livo，不覆盖LIO。

`run_nav_trial.sh start UNIQUE_ID clean long`指定原生IMU，`noise`仅复现历史试验。保留common.time_sync_en=false、真实消息时间、唯一odom/TF和现有保护桥。云端依赖/权重复用现有环境，不要求重装Isaac/CUDA/PyTorch。完整数据与权重不入Git。

## 正常停止与初始化确认补充（2026-10-22）

`run_nav_trial.sh stop`通过`request_trial_stop.py`识别唯一监督案例与同进程组的前台Python worker，发送TERM后等待原监督器正常收尾。记录器/估计器的独立进程组不作为前台目标；无前台worker时只等已有收尾，不再次给supervisor发信号。目标选择4项单元检查通过，LIVO录像预检`moving_20261022_A_fastvideo_preflight_01`在控制未启用阶段实际正常停止，RUNNER exit1、remaining_sessions=[]；该退出是人为预检结束，不是算法成功。B08旧helper重叠收尾造成原RUNNER缺失，保留POST_STOP_AUDIT，不补造成功。

AMCL初始GetState确认采用总30秒、每次最多5秒的有界查询重试，始终在控制未启用阶段。实际B_native_long_09完成初始化和84.286m导航；不放宽运行中的传感器时间/看门狗/姿态/几何保护。
