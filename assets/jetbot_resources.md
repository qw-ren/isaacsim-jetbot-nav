# JetBot运行资源与备份边界

本仓库发布导航源码、接口协议、精选验收摘要。它不是完整云镜像；仅克隆仓库不等于已具备当前实际工程的全部运行依赖。不要盲目重装Isaac/CUDA/PyTorch。

现有云端：`/home/admin/jetbot-warehouse/code`、`source_scene/jetbot_sim.usd`、`map_prior_20261023_01`；Isaac Sim4.5在`/home/admin/isaacsim`，资产在`/home/admin/isaacsim_assets/Assets/Isaac/4.5/Isaac`。原队友场景固定来源为PR4提交`35fee2debcfb528e2aef86c91f82b06c2fd42869`，在副本中禁用其重复ROS图，源USD保留不覆写。

需要的NVIDIA资产（通过合法Isaac资产库获取，不随Git复制）：`Robots/Jetbot/jetbot.usd`、`Sensors/HESAI/XT-32.usd`、`Environments/Simple_Warehouse/full_warehouse.usd`。A/B临时障碍由dynamic_fixture.py建立USD几何体组合并加碰撞，不是下载的箱子/推车资产；不改先验静态地图，按新鲜估计位姿及非零前进状态触发。

固定FAST-LIVO后端：`/home/admin/go2-warehouse/nav_stage2/deps/fastlivo_trial_20261022/bin/fastlivo_mapping`，SHA256 `8a8e61dd543befa5f9da26bdf2674af804f7d2d5a81542d9f1fb7a9cf11e958f`。复用现有ROS2及补充库与nav_env.sh，不包含训练权重或新训练流程。

原迁移受控备份`jetbot_migration_20261023_01.tar.gz`：85,636,883B，SHA256 `6a6db65a147b95e508364f599fccc520dcd116c7f6e0ee2a32763fe58b71dfbd`，940文件核验，包含固定后端/补充依赖/原始场景与证据，不包含完整Isaac/ROS/CUDA镜像和资产库。后续各轮增量备份清单见实验文档。备份在项目维护者本地及云端受控存储，不把不存在的公网下载地址写成可用链接。

视频发布为GitHub Release资产，不进Git对象。最新放大版与验收范围见docs/JETBOT_SCALED_20261025.md；PPO和视觉同学的对接见docs/MODULE_INTERFACES.md。
