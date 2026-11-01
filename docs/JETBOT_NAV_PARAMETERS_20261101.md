# 3倍JetBot研发基线v2：0.40m，2026-11-01

冻结ID：`jetbot_3x_nav_v2_r040_20261101`。用于当前仓储仿真的视觉/PPO接口和算法研发；固定用例回归通过，尚未达到真实停车3cm/角度1–2°，不作为最终工程验收或无人值守训练稳定性承诺。源码与配置摘要见[冻结清单](../experiments/jetbot_soft040_20261101/FROZEN_PROFILE.json)，逐次结果及异常见[验收报告](JETBOT_SOFT040_VALIDATION_20261101.md)。

## 固定参数

|项目|冻结值|
|---|---|
|机器人|独立3倍JetBot；实体保守圆半径0.34m，原1倍模型保留|
|全局/局部规划圆|robot_radius=0.40m，footprint_padding=0.01m|
|障碍软代价|FollowPath.BaseObstacle.scale=0.40，sum_scores=false|
|膨胀梯度|inflation_radius=1.0m，cost_scaling_factor=5.0|
|速度|最大0.8m/s、0.6rad/s；原轮关节上限与耦合限速保留|
|最低速度|线速度/角速度原最低速度均0|
|到达容差|XY 0.05m、yaw 0.05rad，stateful=false；进度预算保持10s|
|安全门|scan点到预测中心轨迹约0.40m、预测1.5s；原超时/锁停、不自动恢复保持|
|局部轨迹|DWB、sim_time=1.5、short-circuit=true、limit_vel_cmd_in_traj=false|
|定位|原生IMU+FAST-LIVO2+AMCL；差速平面twist、完整原始速度检查保留|
|地图|5cm保守CAD规划先验；独立2cm可见Mesh表面定位图|
|定位模型|原生likelihood_field_prob、beam skipping、sigma_hit=0.05，详见任务JSON|

相对v1，规划半径从.65改为.40，障碍权重从.20改为.40。相对已试验的0.40候选，只提高这一项权重；没有同时调膨胀衰减。原生IMU无额外噪声、orientation无效标记不影响加速度/角速度输入；不用truth或轮式里程计替代定位。

规划半径、实体尺寸和安全门含义不同。0.40不是要求日常贴边运行的距离，1m是障碍起算的总梯度影响范围；0.40的Nav2权重也不是PPO奖励权重。[接口说明](JETBOT_SCALED_MODULE_HANDOFF_20261029.md)中的话题、单位和控制源独占约束继续适用。

## 回归证据与限制

完整通过10/12/13/14/16：两次B改道后A绕行、两次固定长路线、一次C目标空闲但不连通的预期NoPath/ABORTED停车。原生状态、输入、安全门、runner与各15个子进程退出审计通过。C通过不计为到达。

完整到达例真实停车约3.71–5.50cm，角度2.73–2.96°；AMCL采样XY RMSE约1.49–1.90cm。C的连续导航全局位姿P95约5.55cm、max10.06cm，不保证每个时刻3cm。估计到达容差不是真实停车精度。此前历史0.40失败和本轮11视口启动失败、15后端收尾-6均随证据发布，不证明历史根因或所有场景成功率。

深度+录像高负载、任意A/B排列、5cm窄隙、绑架恢复、视觉/PPO真实模型联调和长时间训练尚未验收。保留调试发布开关的当前配置用于复现，不能假设其对负载无影响。冻结表示参数、接口和证据可复现；后续修复需版本化并复测。

## 云端复现

保留现有Isaac Sim4.5/CUDA/PyTorch和固定定位后端，不重装或重编。既有资产/基础规划图仍依赖原云端目录；源码包不是完整云镜像。先从本版提交/标签把`isaac_sim/jetbot_fastlivo`放到新的独立代码目录，并安装已打包定位图：

```bash
# 云端admin环境；CODE指向本版源码，不能指向旧release_20261030/code。
CODE=/home/admin/jetbot-warehouse/code_r040_20261101
python3 "$CODE/install_localization_prior.py"
bash "$CODE/manage_jetbot.sh" status
# 确认无已有仿真后，换成从未使用过的新case ID；不要并行仿真。
bash "$CODE/manage_jetbot.sh" start UNIQUE_NEW_ID 140 nav2 stationary "$CODE/warehouse_soft040_precision_20261101.json"
# B→A用warehouse_soft040_BA_20261101.json；C用warehouse_soft040_C_20261101.json。
```

本轮原始运行代码在`/home/admin/jetbot-warehouse/radius_20261101/diagnostic_code_01`，配置在同父目录`configs`；回归后发布的源码仅额外统一接口元数据、README及调试参数默认声明，功能参数以CASE和运行服务审计为准。原始case不可覆盖，源码前后差异及本地门控对照检查见实验目录。

源码/小型证据走Git，四份完整原始精选归档走[本版预发布](https://github.com/qw-ren/isaacsim-jetbot-nav/releases/tag/jetbot-nav-r040-20261101)，下载后按SHA256SUMS及包内清单核验。旧0.65m使用[2026-10-30标签](https://github.com/qw-ren/isaacsim-jetbot-nav/releases/tag/jetbot-nav-baseline-20261030)复现，不用当前HEAD覆盖历史源码。PR #6待review，main不直接更新。
