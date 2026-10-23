# 独立 FAST-LIVO2 仓储导航试验

本目录用于 Go2 / Isaac Sim 4.5，未迁移 JetBot。保留原稳定入口，所有测试生成新 case 与冻结源；不覆盖历史记录。云根目录为 `/home/admin/go2-warehouse`。

## 运行策略

实际运行统一使用 **原生 IMU 加速度、角速度**：`prepare_rgb_trial.py` 与 `prepare_dynamic_trial.py` 显式设置 `imu_noise_enabled=false`。IMU 姿态无效标志保持，不向算法输入姿态/位置真值。历史噪声模型的 seed/profile 可以保留在快照中，但关闭时不叠加偏置或白噪声；离线分析逐样本检查 `/lio/imu` 与 `/evaluation/imu_ideal` 的加速度/角速度完全相同。

LiDAR、原生 IMU、320×240 原生 RGB 输入 LIVO；导航采用上游自身的 IMU 传播状态及激光/视觉校正，保留测量时间戳。唯一适配节点发布导航 odom 与 odom→base，内部估计 `world` 标签不表示读取仿真真值。真值只进独立记录/离线评价。激光/视觉校正超过0.35仿真秒未更新则停止估计输出，原有保护桥超时/净距限值不放松。

实时域41，历史回放域47；实时仿真与历史回放顺序执行。启动器拒绝重复仿真，使用既有 tmux 管理与自动收尾；退出 SSH 不等于关机，不操作云电脑电源。

## 启动示例（云端）

```bash
bash nav_stage2/cloud/warehouse_livo/run_moving_trial.sh start UNIQUE_A_ID A short artifacts/livo_selection_20261022_v2.json
# C采用有已知侧墙/后墙的受控入口；前墙仅在实际运动后插入，先验不写入前墙。
bash nav_stage2/cloud/warehouse_livo/run_moving_trial.sh start UNIQUE_C_ID C short artifacts/livo_selection_20261022_v2.json /home/admin/go2-warehouse/nav_stage2/maps/moving_C_observable_20261022
```

选择 JSON 应传绝对路径，或先 `cd /home/admin/go2-warehouse`。`short` 可改 `long`：原生NavigateThroughPoses将完整仓储路线、回程入口与终点作为同一个任务，剩余途经点<=2且进入安全窗口、实际估计运动时触发障碍。旧案例的分段做法保留在冻结源中。A/B原生成功、C原生ABORTED分别验收，同时检查实际USD碰撞回执、地图不变、唯一估计源、解除控制零输出；实际插入速度与净距在结束后离线核验。准备地图的离线连通性不等于运行通过。

## 历史比较

`run_native_history_comparison.sh start CAPTURE_CASE NOISY_TAG native_01` 只补跑已保存原生 IMU 的 LIO/LIVO，不重新加噪、不重采集。短程旧记录 `short_03` 的 noisy tag 为02，较长 `long_02` 为01。`analyze_imu_pair.py` 在相同输出时间网格比较四条件，仅一次初始SE3对齐，不拟合整轨迹或尺度。旧无图消融与失败结果不覆盖。

FAST-LIVO2 为社区 ROS2 Humble 移植，固定来源/补丁见运行包 PROVENANCE/SOURCE_PINS；不声称官方 ROS2。此处有原生相机和近似点时间等仿真限制，精度不能外推真机。精选备份排除完整 CDR/NPZ/重复权重及 Isaac 资产库，另存完整云端 SHA 清单。

长程首次原生树创建以保护桥未启用的预热动作填充其TF缓存，确认该动作结束后才发送实际目标。预热结果不计导航通过。RemovePassedGoals显式使用map/unitree_go2/base_link，不创建base_link别名；每例冻结prelude_route.json，并检查原生途经点进度。

## 实验性高速跟随录像：全程绕行未通过

`run_moving_trial.sh start UNIQUE_ID A long SELECTION_JSON '' fast-video` 使用独立运行副本，将DWB、保护桥及底层Bridge的前向上限统一为1.7m/s，扩大局部代价窗口到8m。仅fast-video使用DWB前瞻1.5秒（与保护预测一致）、GoalDist48以产生实际较高巡航；标准评分不变。稳定standard仍0.8m/s。姿态、碰撞净距、1.5秒预测、传感器新鲜度和失联超时不变；动态插入窗口前移，保留反应距离。允许转弯和绕障自然减速。

`follow_video_capture.py`只控制展示viewport，以机器人姿态跟随，绝不向估计器/导航反馈真值；算法前向相机标定不变。1280×720、目标12.5帧/仿真秒，保存请求/完成仿真时间，离线按1倍仿真时间编码，速度由独立记录离线核对。不是墙钟录屏加速后冒充高速。长程C按本轮用户安排跳过，不宣称已验证。

高速路线v4沿宽通道避开中央货架紧角，入口(-2,-1.3)，插入窗口在(-3.6,-2.2)的回程通畅段。标准B保留v3路线/入口(-2,0)，不以视频路线替换原验证。先前两轮高速未完成案例与完整速度核验保留。记录器/演示相机可以记录仿真状态；估计器和Nav2不读真值。保护桥使用已声明CAD及插入物AABB作额外独立几何监督，并非已经证明只靠传感器实现所有安全功能。

高速04采用与保护一致的1.5秒DWB前瞻，局部/全局膨胀1.2m、BaseObstacle0.5，属于更保守的障碍间距评分；标准配置不改。高速03在A物理插入后重规划，但预测中心净距0.5485m低于.55m保护而停，原始失败保留，不作为完整绕行通过。

高速录像配置只支持本轮选定的FAST-LIVO2；独立FAST-LIO2标准入口保留。高速转弯使用1.2m中间途经区域（标准仍.7m），以容纳1.7m/s、最大.6rad/s的转弯弧线；这不是碰撞或终点容差。最终XY到达区域及保护阈值不变。`forward_prune_distance=4.0`确保前方参考路径覆盖1.5秒巡航，而非默认2m导致控制器提前减速。

本轮B标准主线84.286m已通过。高速03提供真实82.484m录像，75.323m后运动中插入A，随后预测保护停车，没有越过障碍；不能当成高速完整绕行通过。04完整绕行成功但实际最高1.325m/s，不满足用户高速要求。05/06的4m前向裁剪虽支持高速巡航，仍在途经转角出现原生进度失败，完整记录保留。`fast-video`保留为实验配置，标准入口是已通过的运行基线；本轮不继续为演示调转弯。
