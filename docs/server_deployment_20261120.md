# 云端部署与二维视觉联调（2026-11-20）

部署目录：`/home/admin/atongmu_deploy_20261120`；导航运行目录：
`/home/admin/jetbot-warehouse/code_deploy_20261120`。文件归 admin 用户所有，
安装依赖时使用 root，仿真、定位与视觉运行使用 admin。没有保存 SSH 密码。

## 已部署

- PR #4 原始 JetBot USD；PR #6 v2 导航代码、参数、控制保护和定位地图。
- PR #5 的环境/集成构建材料；按固定版本源码与原补丁重建 FAST-LIVO。
- PR #8 随机仓储代码、资产引用、底图和种子示例，单独验证配置及布局复现。
- PR #7 源码包保存在独立目录，未启用对象真值 oracle 或旧三维盒语义任务管理器。
- 本地二维视觉代码和 ROS 包，新增二维观测缓存。

原 Isaac Sim4.5、资产库和 YOLO 环境保留。安装 ROS2 Humble/Nav2 及编译依赖，
使用独立 `/home/admin/venvs/atongmu_ros` 运行视觉：numpy1.26.4、OpenCV4.11.0、
SciPy1.11.4，复用既有 YOLO 环境的 torch2.8.0+cu128 和 ultralytics8.4.211。
原训练环境的 numpy/OpenCV/torch 没有被卸载或替换，驱动仍为570.172.18。

## 版本边界

本次为 `20261120_pinned_source_rebuild_system_ros` 部署变体，导航参数沿用
`jetbot_3x_nav_v2_r040_20261101`，**不是原冻结运行环境的逐字节恢复**。
系统 ROS2 取代原机器缺失的独立 ROS runtime；Livox 仅生成官方消息定义供编译，
没有启动硬件驱动。估计器使用团队固定 FAST-LIVO/Vikit/Sophus 版本和原补丁，
保留代码中的二进制校验，并在独立部署副本中将其固定为新二进制摘要：

`4bad0bccf5c5b0ec8a14447a72c23146a59447063b1f91b75ed2c2c7d3d1f00c`

原发布包及其校验值保留，新变体单独记录在导航目录的 DEPLOYMENT_PROVENANCE.json。
本次重新生成规划图，场景及规划图摘要均与交付定位地图要求一致，定位地图安装校验通过。

## 已验证

本轮 case：`/home/admin/jetbot-warehouse/artifacts/vision_deploy_20261120_01`。

| 检查 | 实际结果 |
|---|---|
| 视觉测试 | 本地及云端各71项通过 |
| 仓储配置测试 | 云端73项通过 |
| 随机种子1001 / medium | A20、B6、封闭通道[1]，hash前16位3d87ce3ecfffa3c1 |
| 真实输入与TF | 审计通过；IMU、RGB、点云、估计odom、原生定位更新均收到 |
| 二维视觉数据 | 30墙钟秒，63帧、62帧非空、零协议/时龄错误；1帧等待初始clock |
| 二维地图缓存 | 收到map系shelf/pallet/box观测，无尺寸或三维盒 |
| Nav2与视觉并行 | (1.5,6,0)任务原生SUCCEEDED；结束后disarm成功，无保护门故障 |
| 停止检查 | SIM无错误，保护门发出零速，仿真和视觉tmux会话均退出 |

导航末端估计误差约0.0494m，**不是已测真实停车误差或全程定位精度**。
停止由操作者请求，runner为130，部分子进程为-2/254；保留原记录，不当作全部exit0。

首轮视觉检查为60帧/58非空，初始clock未收到导致1次检查错误；第二轮工具显式等待
首次clock后仍执行原时龄阈值，63帧/62非空/零错误。完整保留两轮结果。
还修复了探针对32FC1大小写和16UC1通道数的解析问题，并增加行填充/大小端回归测试。

## 使用

从SSH终端启动一个从未使用过的case ID（首次加载仓库约需两分钟）：

```bash
bash /home/admin/atongmu_deploy_20261120/start_demo.sh demo_NEW_UNIQUE_ID
```

启动后为待命状态，不自动发送导航目标。查看视觉日志：

```bash
tail -f /home/admin/atongmu_deploy_20261120/logs/vision.log
```

另开终端检查话题时，每次设置环境：

```bash
source /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_stage2/nav_env.sh
export ROS_DOMAIN_ID=51
ros2 topic list
```

当前数据流：

```text
Isaac RGB + 对齐深度 + scan + TF
  -> YOLO / 跟踪 / 定位与融合
  -> /perception/detections        像素框
  -> /perception/objects_2d        map系XY观测
  -> /semantic/objects/map_2d      最长5秒的观测缓存
```

现有Nav2继续使用FAST-LIVO2+AMCL定位和雷达避障；视觉节点不发布速度或TF。
平面缓存保留测量表面点，并非物体中心、完整轮廓或导航目标。

结束：

```bash
bash /home/admin/atongmu_deploy_20261120/stop_demo.sh
```

## 尚未验收

- **检测准确率**：训练数据是几何近似物体，真实仓库外观存在域差距。保存的相机帧中
  `pallet`框可能落在叉车轮胎上，已有明显误检迹象，不能把可运行等同于识别正确。
  后续需使用实际仓储资产生成/标注数据并评估、微调。
- **视觉寻物决策**：PR #7 仍要求三维盒子，本次不虚构盒子去兼容它。
  二维观测已进入独立缓存，尚未驱动旧语义任务管理器选择Nav2目标。
- **随机场景联合导航**：种子生成器已部署并验证，未在这轮导航case中插入随机布局。
- 未做全场景导航对比、PPO策略接入、完整定位精度或长时间稳定性验收。

小型证据保存在仓库 `experiments/server_deploy_20261120/evidence/`；
服务器完整报告位于部署目录的 DEPLOYMENT_REPORT.json，完整运行记录保留在上述case。

## 有窗口演示与录像入口（同日新增）

原 `start_demo.sh NEW_ID` 默认仍为无窗口待命；可附加 `gui` 或 `record` 参数。
自动导航与录制的新入口、已生成的54秒MP4和验证记录见
[gui_recording_20261120.md](gui_recording_20261120.md)。新case `gui_record_20261120_01`
已验证有窗口+深度+视觉+录像同时运行，两次短程导航成功；不等于长时间负载验收。
