# 3倍模型对接证据

69：有头、无录像、原生RGB-D+FAST-LIVO+AMCL+Nav2+仲裁nav2，通过到点。299对RGB/深度、249只读样本、同header/光学frame和有效米制32FC1；原始module_observations在Release interfaces_selected.tar.gz。不是视觉识别测试，不证明录像与深度并发稳定。

policy_3x_contract_01：独立ROS52合成通信，真实仲裁和安全门代码；验证正负限速、轮速耦合、选定策略源失联归零锁停、非选定Nav2不接管、消息恢复不自动运动。没有实际PPO模型/仿真连接。

camera_contract.json等从3倍模型真实场景导出；当次CASE.json优先。详见docs/JETBOT_SCALED_MODULE_HANDOFF_20261029.md。
