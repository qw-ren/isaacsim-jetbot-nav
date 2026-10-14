# Go2 仓储外部资源清单

团队约定策略权重、Isaac 安装、资产与依赖不进入 Git。
现有可运行云数据盘 `/home/admin/go2-warehouse` 已有下列文件；
新机器请向任权伟索取实际快照，不能以公开仓库同名文件代替而省略哈希检查。

| 文件 | SHA256 |
| --- | --- |
| ckpts/unitree_go2/flat_model_6800.pt | d358d4a99a327f9c34f8538755655affd50cecf9a194a0ce0d374ba3228df8b0 |
| ckpts/unitree_go2/rough_model_7850.pt | d435f46dcf2d614cad28e21e033b34d6036d079924e1746530a36f2fba2315bd |

放置到 isaac_sim/go2_warehouse/ckpts/unitree_go2（部署后为work/ckpts/unitree_go2）。
仓储：镜像已有 `/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd`。
Go2 USD、Isaac Lab源码/apps与ROS Humble runtime由现有运行环境提供。
补充Python依赖清单在源码cloud/warehouse_stage1/python_dependencies.txt。
负责人本地46MB备份仅含源码/权重/补充Python依赖/证据，非完整镜像或112GB资产库备份。
本文件未提供未经验证的权重下载URL或保存凭据。
