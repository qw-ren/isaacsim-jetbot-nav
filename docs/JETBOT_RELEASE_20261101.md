# 2026-11-01交付：JetBot 3倍0.40m研发基线v2

当前入口：[参数与运行](JETBOT_NAV_PARAMETERS_20261101.md)、[冻结清单](../experiments/jetbot_soft040_20261101/FROZEN_PROFILE.json)、[本轮全部结果](JETBOT_SOFT040_VALIDATION_20261101.md)、[视觉/PPO接口](JETBOT_SCALED_MODULE_HANDOFF_20261029.md)。

规划圆0.40m、BaseObstacle.scale=0.40，其余速度、定位、容差、膨胀梯度和安全门保持。固定用例五例完整通过；启动失败11和收尾异常15保留。真实停车3.6–5.5cm、角度约2.7–3.0°，未达到3cm/1–2°最终目标；不承诺任意布局、高负载或无人值守训练。

通过团队feature分支→[PR #6](https://github.com/qw-ren/isaacsim-jetbot-nav/pull/6)→review流程发布，main不直接合并。[Release](https://github.com/qw-ren/isaacsim-jetbot-nav/releases/tag/jetbot-nav-r040-20261101)标为prerelease，旧10-29视频与10-30基线继续保留。

Release附件：

- `JetBot_R040_Baseline_20261101.zip`：源码、参数、小型证据、接口和报告，不含Isaac/ROS/模型安装环境。
- `radius040_selected_20261101_01.tar.gz`：01–06，包含数值类型启动错误、scan超时、预测间距锁停和成功记录。
- `radius040_diagnostic_selected_20261101_01.tar.gz`：诊断启动失败07、成功08、原始评分和局部地图。
- `cmdstep040_failed_selected_20261101_01.tar.gz`：失败09、被拒绝的原始指令和scan/预测轨迹快照；首步发布方案未采用。
- `soft040_selected_20261101_01.tar.gz`：10–16，包含最终参数回归、启动失败11和收尾异常15。
- `SHA256SUMS`、`RELEASE_MANIFEST_20261101.json`：文件大小、摘要和范围。原始归档均在上传前作包内逐文件核验，Git只收小型JSON/配置/源码。

truth只用于离线评分和展示；语义/视觉模型、PPO模型训练及场景种子不在本交付范围。当前参数和接口按版本固定，后续观测/动作或定位改变需同步版本并复测。
