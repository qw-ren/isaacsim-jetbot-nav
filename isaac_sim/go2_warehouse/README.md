2026-10-22更新：[LIO独立基线](../../docs/GO2_WAREHOUSE_LIO.md)、[LIVO对比与动态封路](../../docs/GO2_WAREHOUSE_LIVO.md)分别保留。原生IMU短程A/B/C及长程A已通过，长程B也已通过（84.286m），本轮跳过长程C，高速实测1.67m/s录像及约1.3m/s完整绕行录像分别发布，高速完整绕行未通过；JetBot未迁移。以下阶段一/二文字保留历史。

# Go2 仓储云仿真（导航迁移前置验证）

负责人：任权伟（导航与系统集成）。本目录是实际云端可运行源码副本，
不是 JetBot 实现，也不是仅从公开上游克隆的工程。

2026-10-13：阶段一 M0–M3 通过（仓储、运动、RGB/Depth/点云/odom/TF/clock、独立启动）。
2026-10-14：云端 baseline 12 项哈希再次通过，当前源码与13日本地备份一致；
用户报告已完成 GUI 键盘验收。SSH 恢复原因由用户确认为笔记本代理问题。
没有将本次手动键盘验收记作自动化测试。另有14日独立低速复测报告12项通过，
本地/云端哈希已核对一致，见 experiments/go2_warehouse_stage1_20261014。
以上是阶段一历史状态。2026-10-18已完成阶段二SLAM/AMCL、受保护Nav2与A/B障碍实验；见[最新进度](../../docs/GO2_WAREHOUSE_STAGE2.md)。

## 运行

现有云实例根目录为 `/home/admin/go2-warehouse`，Isaac 与依赖继续复用。
把本目录内容作为 work 的源码；切勿覆盖唯一稳定副本，迁移时先使用新目录。
`cloud/warehouse_stage1/cloud_env.sh` 支持 GO2_CLOUD_ROOT、GO2_ISAAC_ROOT、GO2_ASSET_ROOT。

依赖目录：deps/IsaacLab（v2.3.0 source/apps）、deps/ros_humble（Humble runtime）、
deps/python（隔离补充依赖，清单见 python_dependencies.txt）。现有镜像 Isaac Sim 4.5、
PyTorch 2.5.1+cu118 与 CUDA 保持原样；112GB 资产库不在 Git 中。
四个补充系统库为 libconsole-bridge1.0、libfmt8、libspdlog1、libtinyxml2-9。
这些是现有环境记录，不是要求重新安装全部环境。

策略文件不提交 Git，放在本目录下 ckpts/unitree_go2；详见
[资源清单](../../assets/go2_warehouse_resources.md)。当前使用 rough 策略。

```bash
# 云端终端；root 会自动切换为 admin。先检查，已有会话时不要重复 start。
bash /home/admin/go2-warehouse/work/cloud/warehouse_stage1/manage_warehouse.sh status
bash /home/admin/go2-warehouse/work/cloud/warehouse_stage1/manage_warehouse.sh start
bash /home/admin/go2-warehouse/work/cloud/warehouse_stage1/ros2_cloud.sh topic list --no-daemon
bash /home/admin/go2-warehouse/work/cloud/warehouse_stage1/manage_warehouse.sh stop
```

详细操作见 [运行说明](cloud/warehouse_stage1/README.md)，
验收结果见 [阶段一报告](../../docs/GO2_WAREHOUSE_STAGE1.md)。
默认 warehouse-shelves、出生(0,-1,0.4)、yaw=0，ROS Domain 41、localhost、FastDDS、仿真时间。
当前使用已有 X11 桌面，不声称 headless 已验证。

## 保留的旧 Nav2 文件

nav2/nav2_params.yaml 与 scripts/start_nav2_servers.sh 是迁移前旧场景配置，
保留用于阶段二审查。脚本含旧笔记本路径且引用未迁移旧地图，**不要直接运行**。
这些旧配置未用于当前仓储验收；当前/scan、地图和AMCL/SLAM使用cloud/warehouse_stage2中的独立入口。旧高速参数仍不应使用。

后续定位节点发布 map→odom 时须关闭 Bridge 的临时静态变换
（`+publish_map_to_odom=false`），避免重复 TF。

## 来源

见 [SOURCE_NOTICE.md](SOURCE_NOTICE.md)。完整本地备份及独立依赖并未全部上传；
本仓库交付源码、配置、脚本及精选验收证据，依赖与权重需由负责人提供。
