# 来源与修改说明

- 上游：Zhefan-Xu/isaac-go2-ros2，isaacsim-4.5 分支。
  https://github.com/Zhefan-Xu/isaac-go2-ros2
- 本地迁移基线 HEAD：f201692a5519e6e6ed058637ce93a2f603f016f2。
- 本地实际修改包含 ROS2 Bridge、控制与传感器接口；
  cloud/warehouse_stage1/local_sim_changes.diff 记录相对基线的差异。
- 云端适配见 cloud/warehouse_stage1/cloud_runtime.patch；导出的源码已应用这些改动，
  不要对它再次应用补丁。
- 上游 README 保留为 UPSTREAM_README.md，版本说明以本目录当前 README 为准。
- Go2 控制与策略沿用上游及其 go2_omniverse/Isaac Lab 来源，机器人 USD 来自现有
  NVIDIA Isaac Sim/Isaac Lab 资产；不是团队新训练或原创机器人模型。
- 此快照未发现上游独立 LICENSE 文件，不为上游源码或策略另行声明新许可证；
  使用与再分发需遵守原作者及 NVIDIA/Isaac Lab 的相关条款。
- 本目录不包含 NVIDIA 资产、Isaac/CUDA/torch 安装包、依赖运行库或策略权重。
