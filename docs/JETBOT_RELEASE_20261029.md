# JetBot 仓储导航：2026-10-29汇报素材与交付入口

任权伟负责导航和集成。当前为3倍JetBot、原生IMU、FAST-LIVO2+AMCL+Nav2。已完成A绕过、B封路换道、C无路停车与终点精度改进；本轮新增同一行程同时处理A/B的视频、RGB-D接口和PPO仲裁的放大版验证。不包含视觉模型/PPO训练或真机部署。

## 视频（可直接用于PPT）

[GitHub Release：视频与实验包](https://github.com/qw-ren/isaacsim-jetbot-nav/releases/tag/jetbot-warehouse-20261029)。文件 JetBot_AB_Arrow_Goal_4x_final.mp4，1280×720、25fps、按仿真时间四倍速约17.6秒。连续行程45.56m、最高指令0.8m/s。片中约3.21秒出现B类完整封路，随后换货架通道；约10.15秒出现A类单箱障碍并绕过；终点红色信标。后方平滑跟随镜头，宽橙色地面箭头来自原生局部轨迹，统一前移1.1m仅为展示，不能用作碰撞/精度测量。原地转向或局部轨迹很短时不显示长箭头。信标随可见性出现，不透过货架；视频裁去启动与初始镜头调整。

箭头和信标为离线观察画面叠加，没有加入USD、RGB/点云或控制链。truth只用于离线评分和展示投影。连续试验 showcase_AB_20261029_68 的Nav2原生状态4、全部子进程退出0、安全门无故障。实际终点11.63cm，估计终点2.60cm，不把后者冒充真实误差。

## PPT图表、CSV和原始依据

| 用途 | 仓库目录 / 文件 |
|---|---|
| 同一行程A/B实际路线与更新规划 | experiments/jetbot_showcase_20261029/SHOWCASE_ROUTE.png、pdf；trajectory_5hz.csv；SHOWCASE_AUDIT.json |
| B封路前后真实全局规划/实际绕行 | experiments/jetbot_assets_abc_precision_20261029/B_REPLANNING.png、pdf |
| 终点精度与AMCL改进 | 同目录 ENDPOINT_COMPARISON.png、pdf、各case指标；完整解释见JETBOT_ASSET_ABC_PRECISION_20261029.md |
| 同输入LIO/LIVO精度对比 | experiments/jetbot_lio_livo_20261029/LIO_LIVO_COMPARISON.png、pdf；accuracy_summary.csv；trajectory_and_error.csv |
| 视觉/PPO放大版参数、标定和验证 | experiments/jetbot_scaled_interfaces_20261029/；docs/JETBOT_SCALED_MODULE_HANDOFF_20261029.md |

Release的 JetBot_20261029_PPT_and_Interfaces.zip 汇总上述图表、CSV、JSON及交接文档。精选原始数据另附 showcase_selected.tar.gz（含失败66/67与成功68完整轨迹/路径）、interfaces_selected.tar.gz（69接口原始观测）、jetbot_assets_abc_precision_20261029_01.tar.gz（此前ABC/精度日志），避免把完整日志/视频/资产塞入Git。每个附件有SHA256SUMS；脚本与小型结果在Git可复核。完整Isaac资产库及708MB原始传感器数据库不随仓库分发，LIO/LIVO输入数据库SHA及复现脚本单独保留。

## 可引用的实验结论及限制

同一43.66m记录、完全相同点云/原生IMU/RGB输入、0.5倍离线回放：FAST-LIO2 XY RMSE11.23cm，FAST-LIVO2为1.49cm，LIVO关闭视觉为3.98cm；最终相对位置误差分别13.40、1.64、4.36cm。本次视觉融合更好，但只有一条轨迹；不是实时吞吐量测试，不代表各种路线或真机精度。姿态并非所有指标都更好。初始标定只使用静止段，不做全轨迹对齐拟合。详见LIO/LIVO对比报告。

终点改进：同路线基线17.81cm→控制参数9.56cm→加差速AMCL模型6.96cm；带B障碍组合测试12.55cm，新A/B连续视频11.63cm。单次到点3cm的配置容差不等于全局真实3cm精度。C为目标空闲但路径不连通的预期NoPath(native6)，不是到达成功。

## 仍存在的问题

1. 全局定位偏差仍约5–11cm量级且与路线有关。先验是5cm栅格的保守CAD包围盒，scan只旋转去畸变、未补偿平移。未做绑架/全局重定位鲁棒性证明；不能把误差简单归为AMCL重定位失败。
2. A后紧接B的狭窄掉头组合在66/67未通过：66有SCAN_TIMEOUT，去掉深度后67仍触发Failed to make progress。成功68采用B后在相邻通道出现A，不声称所有障碍排列都可靠。安全门/10秒停滞保护没有放宽。
3. RGB-D加录像的并行负载尚未验收；69无录像的短程RGB-D+Nav2仲裁通过（299对图像深度、249观测样本），不能推断长时重负载稳定。
4. 历史个别FAST-LIVO退出时有rclcpp上下文竞态/子进程-6，另有历史启动服务超时；本轮68子进程全0，但不声称历史退出问题已根治。
5. PPO只验证入口、限速和失联锁停；视觉只提供输入/标定及预留输出，语义导航只有人工名称目录。模型、训练、检测器与完整自然语言语义导航尚未实现。

## 对接同学优先阅读

[JETBOT_SCALED_MODULE_HANDOFF_20261029.md](JETBOT_SCALED_MODULE_HANDOFF_20261029.md) 更新3倍轮径/轮距、0.8m/s限速、质量尺度、0.65m规划裕量、传感器安装/内参、时钟、PPO提案话题与失联策略。机器可读 extension_contract.json v2。原有1倍0.37m/s参数和checkpoint不能直接套用。

本轮在隔离worktree提交，未操作原工程含安装包的暂存区；云端所有实验结束，不关机。按团队约定更新PR #6来源分支，main等待review，未直接推main。
