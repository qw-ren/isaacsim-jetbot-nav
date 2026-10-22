# FAST-LIO2 独立基线

本目录只负责LIO传感器适配、独立回放和导航接入，不含LIVO视觉融合。已部署与116.645m历史验证见[LIO报告](../../../../docs/GO2_WAREHOUSE_LIO.md)及experiments/go2_warehouse_lio_20261021。2026-10-22后实际部署策略统一原生IMU，历史noise案例不可改成无噪结果；此目录旧文档中的“不接入LIVO”是当时阶段决策。新LIVO放在相邻warehouse_livo，不覆盖LIO。

`run_nav_trial.sh start UNIQUE_ID clean long`指定原生IMU，`noise`仅复现历史试验。保留common.time_sync_en=false、真实消息时间、唯一odom/TF和现有保护桥。云端依赖/权重复用现有环境，不要求重装Isaac/CUDA/PyTorch。完整数据与权重不入Git。
