# 对象级中文关联研发记录，2026-11-19

主说明：../../docs/JETBOT_OBJECT_GROUNDING_FINETUNE_20261119.md。

- `PROTOTYPE01_*`：从零训练原型，未见表达失败，保留。
- `PRETRAINED02/03/05/06/07/08/09_*`：七轮预训练微调原始报告，包括退步、错误接受与低覆盖情况。09为最终Mini候选，8771673参数，15998训练/800开发/1000测试；测试原始目标判断98.9%。
- `BENCHMARK_RESULTS.csv` / `Grounding_Benchmark.png`：所有候选指标，含默认0.9筛选覆盖率；该基准已用于多轮开发，非全新外部验收。
- `FINAL_RUNTIME_BENCHMARK*`：最终0.8阈值及数值一致性检查后的1000条逐样本记录与摘要。905条确定判断中904正确，1条错误为不导航判断；337个对象建议在该基准上无错误，其余含低置信和一致性拒绝。确定判断包含NOT_FOUND/AMBIGUOUS，不代表发起导航。
- `FINAL_POST_CONTRACT_*`：24条既有指令、12条否定回归、12条训练前新留出指令的最终记录。三组分别19/10/11条正确确定判断，其余拒绝；均未发ROS导航指令。
- `FINAL_ROS_PREVIEW_RESULT.json` / `FINAL_LEGACY_ROS_REGRESSION.json`：最终源码域52四项学习预览与十一项旧后端夹具回归。fake Nav2，不是Isaac实仿；学习预览任务Nav2目标0、gate使能0。
- `PREVIEW_PROFILE.json`：只读可审查配置，模型checkpoint摘要、阈值、绑定表。manager硬性拒绝学习后端非dry_run任务。
- `CASE09_SUCCESS_BINDINGS.json`：仅历史真实到达的box_far接近位姿，不是所有对象已绑定。
- `*ARCHIVES.json` / `*BACKUP_VERIFICATION.json`：本地与云端逐SHA备份核验。完整模型、合成数据和源码归档在原工程data/cloud_warehouse/jetbot_grounding_20261119，不将模型权重塞入Git。

语义一致性检查只验证神经网络选择是否符合它自己提取的数值条件，不解析文字、不替选物体，也不能保证语言理解正确。仍限只读预览，未启用真实视觉或实际导航，定位与Nav2冻结参数保持原样。
