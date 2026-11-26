# 导航视觉与随机场景集成

本分支合并任权伟当前导航与二维语义地图代码、PR #8 随机场景模块和 PR #9 视觉模块。合并保留三条 Git 历史，原开发工作树及 main 未改动。随机场景生成、视觉识别和导航使用各自已有的验证入口；源码合并不等于三套运行环境已经完成联合实仿。

## 合并来源

| 来源 | 固定提交 | 内容 |
| --- | --- | --- |
| 当前已提交导航 | c7a017ed771636451d91cfa12626d1e929b61d59 | FAST-LIVO2、AMCL、Nav2、对象模型与导航协议 |
| 当前未提交成果快照 | 7ab05ff | 单次托盘导航接入、视觉地图适配、已引入视觉源码与记录，共98个文件 |
| PR #8 | 13e42258689dc3b28c8e8f5f8d506f3513249f6e | 仓储生成器、场景资产、机器人独立集成及测试 |
| PR #9 | a3bbe944a99b72e5b93e1ef6024125af4e586b73 | 视觉检测、跟踪、平面定位、ROS封装、数据生成与观测证据 |

合并提交分别为 `6711238`（PR #8）、`63f9562`（PR #9）。此前手工引入的视觉文件与 PR #9 一致，Git 自动合并补齐了遗漏文档和实验记录，没有文本冲突。原始98文件的逐项备份保存在本地 `data/cloud_warehouse/jetbot_merge_pr8_pr9_20261126`，不把安装包、权重或传感器大文件加入Git。

## 目录与入口

| 功能 | 目录 | 运行范围 |
| --- | --- | --- |
| 冻结导航和仿真监督 | `isaac_sim/jetbot_fastlivo` | 已有3倍JetBot、原生IMU、FAST-LIVO2、AMCL、Nav2、安全门 |
| 对象地图和语言任务 | `ros2_ws/src/jetbot_semantic_nav` | 明确区分 oracle 与 vision 两种来源 |
| 视觉算法 | `perception` | 图像检测、跟踪、RGB-D/雷达几何，无控制输出 |
| 视觉ROS封装 | `ros2_ws/src/atongmu_perception_ros` | 生产检测节点；独立缓存另有自身入口 |
| 随机场景 | `isaac_sim/warehouse` | 布局生成、拓扑检查、资产和独立机器人集成示例 |
| 视觉数据生成 | `isaac_sim/replicator` | 训练数据与真实资产小样本工具，不自动运行 |
| 队友部署和观测工具 | `deployment/server_demo` | 保留历史部署路径；不是本分支的通用启动器 |
| 统一离线验证 | `tools/validate_team_merge.py` | 分进程检查所有模块，不启动ROS或Isaac |

模块保留原目录和导入结构，避免移动代码破坏脚本相对路径。README提供当前总入口；各模块原文档和历史实验保持原样，不能把他人云端部署记录当作当前机器的状态。

## 接口冲突处理

### 两种二维地图分开发布

| 话题 | 发布方 | 协议和作用 |
| --- | --- | --- |
| `/perception/objects_2d` | PR #9 PerceptionNode | `atongmu.planar_observations`，原始可见表面观测 |
| `/semantic/objects/map_2d` | PR #9 PlanarMapNode | `atongmu.planar_map`，5秒短期缓存 |
| `/semantic/objects/visual_map` | 导航 VisionMapper | `jetbot.visual_object_map`，关联后的可查询对象 |

原导航适配也用了 map_2d 名称，本次将新配置与ROS验证夹具改为 visual_map，防止不同协议同名发布。任务管理器只接受配置中对应的协议。集成 supervisor 只启动 PerceptionNode 和 VisionMapper，不同时启动队友独立缓存节点；独立视觉调试仍可使用原 `jetbot_planar.launch.py`。

旧云端03报告中的话题名称属于旧接口记录，未改写。新配置通过 `make_visual_map_config.py` 生成。点测量不虚填成三维盒子，不沿用 oracle 的对象ID、尺寸、排号或固定目标绑定。

### 随机场景与导航底座分别启动

PR #8 独立示例使用自己的出生点 `(-9.97,-7.37,0.1009)`、机器人几何和 `/rgb /scan /odom /cmd_vel` 图。当前导航使用3倍JetBot和 `/jetbot/...` 传感器/控制接口、FAST-LIVO2估计里程计。两者不能在同一控制域中各启动一套机器人、TF或速度输出；PR #8 的仿真里程计没有接到当前定位链。

保留随机生成器及其场景测试，但不自动把随机布局套入冻结导航先验。真正联合运行前，需核对机器人尺寸、出生位姿、场景和规划/定位图摘要及reset生命周期，再生成独立实验配置；现有地图哈希校验不能绕过。生成器的资产类别与实例ID是场景元数据，不是视觉识别结果，也不直接写入视觉语义地图。拓扑通道编号不冒充视觉货架排号。

### 文件保留范围

PR #8、PR #9 的新增文件内容保持原样。仅将合并后 `.gitignore` 的 USD 放行规则收窄到两个已交付文件，防止后续生成的大场景文件意外进入Git。当前98文件快照中仅四个文件有明确后续变更：视觉配置生成器、对应配置测试、ROS夹具与视觉地图说明，均为话题重名的处理；其余文件逐摘要核对。

七个底层导航/安全文件和冻结配置没有被场景或视觉分支覆盖。原始1倍/3倍配置、原生IMU、FAST-LIVO2定位来源、安全门和已有到达容差保持不变。

## 验证与复现

在笔记本集成工作树中运行，不在Go2真机执行：

```bash
/usr/bin/python3 tools/validate_team_merge.py --output /tmp/jetbot_team_check_NEW_UNIQUE_NAME
```

输出目录必须不存在；每个组件的完整输出和汇总 JSON 都保存在该目录。组件单独运行，避免同名 `test_config` 或顶层 `generator` 导入互相影响。

本地209项组件检查通过：导航/视觉地图42、视觉75、采集变换2、场景配置70、场景确定性12、场景负例8。本地缺少云端官方资产目录，场景配置额外3项文件检查跳过；云端已执行该脚本，73项全部通过。seed1001/medium在独立进程中生成两次结果一致，A20/B6、封闭通道[1]，布局摘要与PR #8示例相同。仓库Python语法、shell语法及ROS包名唯一性也已检查。

云端独立 `team_merge_20261126_01` 验证合并后对象地图话题，仅使用domain52的合成视觉协议夹具、真实mapper/manager/语言模型，不启动domain51仿真。详细结果见 `experiments/jetbot_team_merge_20261126`。

八类别检测权重尚未提供，不能称为真实相机识别、随机场景视觉导航或PPO接入成功。原main和PR #8/#9的GitHub状态不作修改，本次结果保存在本地集成分支，尚未推送。
