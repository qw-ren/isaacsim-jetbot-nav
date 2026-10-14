# Go2 仓储云仿真阶段一验收记录

日期：2026-10-13（Asia/Shanghai）。范围仅为 Go2 仓储仿真与 ROS2 接口。
操作命令与迁移结构见 [运行说明](../isaac_sim/go2_warehouse/cloud/warehouse_stage1/README.md)。

## M0：环境与实际工程审计（通过）

- CONFIRMED：云镜像 Ubuntu 22.04.5；Isaac Sim 4.5.0-rc.36；驱动 570.172.18。
- CONFIRMED：GPU RTX5880-Ada-24Q，24,576 MiB；32 vCPU；约 62 GiB RAM。
- CONFIRMED：系统盘 100 GB，数据盘 200 GB，数据盘挂载 `/home`。
- CONFIRMED：镜像已有仓储与 Go2 USD，不需下载 112 GB 资产库或重新安装 Isaac Sim。
- CONFIRMED：有效推理运行时保留镜像 torch 2.5.1+cu118；使用本地 Isaac Lab v2.3.0 source/apps。
- CONFIRMED：ROS Humble 在独立目录迁移；Python 依赖安装在独立 target。
- CONFIRMED：只额外安装四个系统共享库：libconsole-bridge1.0、libfmt8、libspdlog1、libtinyxml2-9，约 846 kB。
- CONFIRMED：迁移源包含本地 modified Bridge、仿真入口、控制/传感器代码及实际权重，不以公开仓库替代。
- CONFIRMED：12 项 baseline 源码、配置与权重哈希逐项一致；原本地核心文件没有被改写。
- UNKNOWN：镜像控制台中的计费产品、自动关机、释放与长期数据保留策略；没有操作云电脑电源。

## M1：仓储与站立（通过）

实际资源：`/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd`。
场景 prim `/World/Warehouse`：8,126 个子 prim，1,878 个碰撞 prim。
单位米，Z-up；世界范围约 x=[-12,12]、y=[-18,20.818]、z=[0,9.300] m。
最终起点 `(0,-1,0.4)`，yaw=0。实际稳定 base 高度约 0.417 m。

原点不适合作为任意朝向的测试起点：USD 中货架支架位于原点前方约 0.24 m。
几何检查在保留碰撞的情况下选出沿 X 的空通道，测试保守余量 0.6 m。
此检查仅选择静态测试起点，不是 SLAM 或运行中的导航算法。

## M2：运动与 ROS 接口（通过）

成功记录：云端 `artifacts/m3-canonical-v3/interfaces.json`。
通过 `/unitree_go2/cmd_vel` 在仿真中实际发送速度并测量结果：

| 检查 | 实际值 |
| --- | --- |
| 前进指令 | 0.25 m/s，4 秒仿真时间 |
| 前进距离 | 0.837283 m |
| 转向指令 | 0.35 rad/s，3 秒仿真时间 |
| 转角 | 1.033954 rad |
| 零速度后线速度 | 0.002652 m/s |
| 零速度后角速度 | -0.044923 rad/s |
| 起始倾角 | 0.010984 rad |

RGB 非空、Depth 有效、点云含有效 XYZ、内参与图像 frame 一致、
仿真时间推进、消息时间戳单调、四段 TF 存在，检查均通过。
动态 `/tf` 经 ROS CLI 确认只有 `robot_data_manager` 一个发布者。
`use_sim_time` 经实际参数服务确认 true。

当前约 0.3 实时因子，实际消息约 10–12 Hz（以各次 JSON 为准），不是已实现 40 Hz 实时运行。
语义分割、逐点 LiDAR 去畸变及独立键盘自动验收不在通过清单中。

## M3：独立重新启动（通过）

独立重新启动后，`artifacts/m3-nav2-alias/interfaces.json` 通过全部 12 项检查。
本轮使用 `/cmd_vel`，前进 0.837283 m、转向 1.033954 rad，
零指令后线速度 0.002652 m/s、角速度 -0.044923 rad/s。
这一固定起点/固定命令试验的数值与 canonical 话题试验一致；不据此声称任意工况可靠性。
最终截图为 `artifacts/m3-reproduction/warehouse_viewport.png`，已目视确认机器人与货架可见。

云端成功运行日志：`logs/warehouse_20261013_201832.log` 与
`logs/warehouse_20261013_202647.log`。首次旧执行器失败与不合适起点的记录全部保留。
两次成功运行均已退出。session manager 的 stop 在 canonical 试验后实际验收；
最终复现运行在有界窗口到期后正常退出。收尾确认仿真进程与 tmux 会话均已退出，
两份成功日志没有 Traceback、getArticulationData 或执行器错误。
云电脑电源保持开机，未进入阶段二。

本地证据目录：`data/cloud_warehouse/stage1_20261013/`。
其中保存全部 artifacts、日志、云端工作源码、独立 Python 依赖及环境记录；
原本地核心仿真源码与 cloud baseline 哈希一致。没有 commit 或 push。
本地备份约 46 MB；补丁 dry-run、脚本语法和 Python 编译检查通过。
最终系统盘剩余约 64 GiB、数据盘约 49 GiB。云端新增文件没有保存登录密码。

## 保留的失败与修复依据

- 首次镜像缺少 h5py/HID 等模块；补齐轻量依赖，未重装 torch/CUDA。
- 原 Bridge 后台读取 PhysX 状态产生 getArticulationData 错误；读取移至仿真线程。
- 原外部 CLI 设置仿真时间失败；改用节点参数 API。
- 原 CameraInfo 的短 frame 不匹配图像；改为完整 `unitree_go2/front_cam`。
- 一次 180 秒窗口过早结束，运动已生效但停车检查超时；这轮不算通过。
- 原点 yaw=90° 时前进仅 0.059 m；保留失败 JSON，并据 USD 几何更换起点。
- 原退出路径执行器与销毁节点互等；独立 executor 加顺序关闭后已正常退出。
- 早期高跟随视角被货架遮挡；最终改为较低的机器人跟随视角。

所有历史失败 artifact 与日志均保留，验收仅引用相应通过记录。

## 下一阶段边界

CONFIRMED：里程计来自仿真真值；`map → odom` 是临时静态变换。
未运行在线 SLAM、Nav2 或导航目标验收。旧仿真地图不适用于仓储。
以后接入 SLAM 时禁用 Bridge 的静态 map TF，避免重复发布。
当前结果不能证明碰撞自动避让、导航可靠率、真机迁移或 PPO 训练兼容性。
尚未发布 `/scan` 或 IMU；下一阶段根据选定 SLAM 的输入契约处理，不能直接假定适用于 Fast-LIO。
Go2 无显示服务器的 headless 模式尚未验收；当前验证使用镜像现有 X11 桌面。

## 2026-10-14 补充复核

云端baseline 12项SHA256重新通过，当前运行源码与13日备份一致。用户报告GUI键盘验收通过；这不是新增自动化运动测试。SSH现已恢复，用户确认为笔记本代理问题；关机操作者仍未知。云端仓储SLAM/Nav2尚未验收。精选历史证据见 `experiments/go2_warehouse_stage1_20261013/`。

另已核对14日独立低速复测报告：`experiments/go2_warehouse_stage1_20261014/interfaces.json`，12项通过；本地与云端SHA256一致（3f1edfe1…7d99b0b）。该报告为已有独立复测，本次归档没有再次发送运动命令。
