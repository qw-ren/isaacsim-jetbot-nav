# 学习对象关联接入 Nav2：独立仿真测试

用户授权将“到第二排货架旁的托盘”接入实际仿真导航。模型沿用 grounding_20261119_09/run01，置信度阈值 0.8、差距 0.2，不重新训练。固定地图、定位、控制器及安全门参数沿用已冻结版本。

默认学习后端仍为预览。新增 `object_grounding.simulation_execution`：仅在 simulation_only、operator_enabled 均为 true、后端为 pretrained_chinese_encoder、显式 enabled 时，允许配置中列出的原句。推理后再次检查选中 ID 是否等于该原句允许的 ID；这不是将指令规则映射到目标，实际对象仍由模型推理，匹配条件仅允许或拒绝执行。其他指令/ID 保持 MODEL_NOT_QUALIFIED。

执行继续经过当前对象有效性、地图/场景注册、固定绑定几何、接近点代价、原生 ComputePathToPose、完整路径与已登记物体交叉检查、NavigateToPose、速度策略和安全门；没有用对象中心替代可通行接近点，也没有用机器人真值定位。物体来自 USD oracle，机器人真值仅离线评分。

推理依赖目录仅加入 semantic manager 子进程 PYTHONPATH，不修改 Isaac 安装、定位后端或全局环境。33 项单元检查通过。

独立场景含三只原生托盘（第二排目标、第一排和第四排干扰）及第二排旁的纸箱。目标 ID `/World/SemanticOracle/pallet_target_row2`；固定接近位姿 `[-8.4, 10.388850762750563, 1.5707963267948966]`，中心到目标托盘表面的规划距离 1.21m。沿用语义接近余量 0.8m，不表示物理机器人半径或障碍膨胀参数发生变化。

云端独立源码：`/home/admin/jetbot-warehouse/pallet_nav_20261119_01/code`；配置 `../NAVIGATION.json`。唯一实验：`/home/admin/jetbot-warehouse/artifacts/pallet_grounding_nav_20261119_01`。先只读预览确认目标和路径，然后执行同一句原话；原始失败也保留。运行结果以本目录对应 experiments 记录为准，准备文档不构成成功声明。

## 实仿结果

`pallet_preview` 为 DRY_RUN_COMPLETE；`pallet_nav` 为 SUCCEEDED/native4，passed=true，disarmed=true，安全门无故障。模型正确选择目标，置信度 0.99085885，原生预览路径 10.216m。完成仿真时刻 45.18999899s；离线最近真值样本时间差约 7.52e-11s，无轨迹拟合或位姿修正。真实终点对绑定接近位姿为 3.1743cm / 2.7918°；估计位置误差 1.8627cm，不将估计误差、到物体的距离或全轨迹定位指标混为一谈。

本例仅证明在该静态布局、当前出生位置与这一原句上的单次闭环成功，不证明任意表达、布局或连续多任务可靠。真实角度误差仍大于导师提出的1–2°目标。启动和行驶期间未调整模型、固定导航或安全参数。

成功后请求正常 stop。RUNNER_RESULT exit_code=130、error=null，部分节点收到SIGINT后返回-2/254，未发生已记录的-6，不称全部子进程退出0。管理会话已结束，未操作电源或用户SSH Master。

本地证据 `data/cloud_warehouse/jetbot_grounding_nav_20261119`：4倍仿真时间视频、带SHA256的精选归档、配置、基线比较及备份核验。视频仅观察画面离线文字，不改变传感器或USD。新代码和报告尚未推GitHub。
