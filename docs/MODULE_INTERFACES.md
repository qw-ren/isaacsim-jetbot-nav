> **2026-10-29更新：当前3倍模型请先读 [放大版接口交接](JETBOT_SCALED_MODULE_HANDOFF_20261029.md)。以下保留1倍历史验收参数，不能直接用于当前模型。**

# JetBot 模块对接说明（导航侧，2026-10-25）

本轮只提供导航与系统集成接口，不实现同学负责的 PPO、目标检测或视觉定位。依据用户确认以及10.16汇报的分工：任权伟负责导航/SLAM/集成及控制仲裁；代晟伯负责视觉；殷皓程负责 PPO；刘航硕负责仿真与传感器。原生 IMU、FAST-LIVO+AMCL、Nav2 全局规划和 DWB 基线保留。此处是仿真接口，不用于真机。

## 提供给视觉同学

| 输入 | 类型 | 坐标/含义 |
| --- | --- | --- |
| `/jetbot/image_raw` | `sensor_msgs/Image` | RGB8，320×240，`jetbot/camera_optical` |
| `/jetbot/depth/image_rect_raw` | `sensor_msgs/Image` | 可选开启；32FC1，小端 float32，单位米，光轴 Z 深度，不是欧氏射线距离；NaN 无效 |
| `/jetbot/camera_info` | `sensor_msgs/CameraInfo` | 与 RGB 同分辨率、同 header；K/P 来自实际算法相机 |
| `/tf`、`/tf_static` | TF2 | `map→odom→chassis→jetbot/camera_optical`；按采集时间查询 |
| `/jetbot/livo/points` | `sensor_msgs/PointCloud2` | 真正 LiDAR 观测，见 SENSOR_CONTRACT；不得把相机光学轴当车体轴 |
| `/clock` | `rosgraph_msgs/Clock` | 仿真时间，模块启用 use_sim_time |

深度与RGB来自同一相机 render product 的同一 current_frame，使用同一采集时间/光学 frame/标定；额外深度开关不改变算法相机位姿或 RGB 标定。开启 `navigation.extensions.depth=true`。固定观察相机是视频用的，不能作为视觉算法输入。像素回投：X=(u-cx)Z/fx，Y=(v-cy)Z/fy，Z=深度；用对应时间的 TF 转入 map，不用截图尺寸/观察镜头内参。SDK 原始无效深度转换为 NaN，不填成零距离或自由空间。

按汇报保留输出名称 `/perception/detections`、`/perception/object_poses`。字段约定：类别、置信度、检测框/三维位置、frame_id、采集时间、可选实例ID和位置不确定度。建议标准消息 `vision_msgs/Detection2DArray`、`vision_msgs/Detection3DArray`，但当前云端未装 vision_msgs、尚无检测发布者/消费节点：这两项是待双方确认的输出约定，不冒充已运行接口。正式联调前确认消息版本及依赖；本轮不为此重装环境。

只给二维框不能直接生成地图导航目标。视觉同学需提供有效深度回投/或激光关联后的空间位置，以及坐标系和时间。货架/目标本体位置也不等于机器人目标位姿：导航侧后续选择可达的接近点、距离及朝向，处理歧义、置信度和过时结果。

## 提供给 PPO 同学

- `/jetbot/scan`：`sensor_msgs/LaserScan`，frame `chassis`，1440格/0.25度，量程0.15–20米。无有效高度带回波为NaN，即未知，不是20米自由空间。正角绕+Z，车体+X前、+Y左。当前启用基于原生 IMU 角速度的旋转去畸变，未补偿平移。
- `/jetbot/odom`：`nav_msgs/Odometry`，parent `odom`、child `chassis`，估计位置和车体速度；不是轮式里程计/真值。全局目标和规划使用 `map`，必须通过 TF 转到车体，不能直接相减不同坐标系坐标。
- `/plan`：`nav_msgs/Path`，frame `map`，由 Nav2 全局规划提供。局部子目标、雷达扇区化、历史堆叠和归一化由 PPO 同学按模型定义实现；接口不固定36/72格，也不把汇报里参考模型的50D当成本项目既定维度。
- 全局任务使用现有 Nav2 `NavigateToPose` / `NavigateThroughPoses`；任务请求、反馈和结果保存在 case/missions。训练 reset、reward、碰撞反馈、域随机化和模型推理不由本轮实现。

输出预留为 `/jetbot/policy_proposal`，`geometry_msgs/TwistStamped`，header.frame_id=`chassis`，header.stamp为仿真采集/决策时间；linear.x单位m/s，angular.z单位rad/s。不要发送归一化网络动作/左右轮速。策略侧先恢复物理量；本轮底盘线速度上限0.37m/s、角速度0.6rad/s，最终还受原始轮速耦合限制（旋转时可用线速度下降）。初期约定前进速度；其他Twist分量置零。速度上限不等于需要始终输出上限。

`navigation.extensions.command_mux=true` 启用导航侧仲裁；`control_source=nav2` 或 `ppo` 在启动时冻结。Nav2原生控制器改发 `/jetbot/nav2_proposal`，仲裁唯一输出 `/jetbot/nav_cmd_vel`，已有速度门唯一发布 `/jetbot/cmd_vel`。PPO同学不能发布后两个话题，不能启动额外TF或改地图。默认旧配置不启用仲裁，保留历史 DWB 路径；对接使用明确的 module_interfaces 配置。

选定源过时/时钟异常/非有限动作/重复发布者会使仲裁归零并锁停；不会自动切到另一个源，也不会在消息恢复后自动启动。最终速度门及底盘仍保持各自的限速、扫描净距、消息新鲜度和失联停车检查。上线PPO时另开新case，不在正在行驶的case热切控制权。当前 `ppo` 模式仅用隔离域中的合成通信消息验证，未接真实策略，不能称PPO导航通过。

## 留给语义导航的边界

当前 `semantic_places.json` 是人工根据CAD标注的名称/别名→空闲区域接近位姿，供导航任务层验证。不做语言模型/目标识别，不假设箱子身份已知；未知或歧义名称明确拒绝，不猜坐标。`semantic_goal.py` 解析后复用已有不可变任务提交器，必须有活跃且IDLE的operator会话，并保留词条ID、目录SHA及目标坐标审计。

后续语义导航接法：视觉输出带时间戳的目标实例/位置 → 语义目录或对象关联 → 选择明确目标 → 导航侧求可达接近点 → Nav2规划 → DWB或PPO控制 → 完成/失败反馈。自然语言解析可以以后接在任务层；不用改底层传感器或让模型直接绕过速度门。没有可靠空间位置或多目标有歧义时不发运动请求。

## 对接样例与验证

所有命令在云端执行；先 `manage_jetbot.sh status`，使用新ID，保持有头模式。`module_probe.py` 是只读接收样例，记录实际RGB-D配对、标定、scan、odom和全局路径；不执行模型，也不发速度或订阅truth。

```bash
bash /home/admin/jetbot-warehouse/code/manage_jetbot.sh start UNIQUE_ID 180 operator stationary /home/admin/jetbot-warehouse/code/module_interfaces47.json
# 等待OPERATOR_READY且IDLE，再按实际case路径提交：
python3 /home/admin/jetbot-warehouse/code/semantic_goal.py --catalog /home/admin/jetbot-warehouse/code/semantic_places.json --place 东侧入口 --case /home/admin/jetbot-warehouse/artifacts/UNIQUE_ID --id entry1 --timeout 180
# 等待missions/entry1/RESULT.json以及IDLE，再提交第二个：
python3 /home/admin/jetbot-warehouse/code/semantic_goal.py --catalog /home/admin/jetbot-warehouse/code/semantic_places.json --place 交接点 --case /home/admin/jetbot-warehouse/artifacts/UNIQUE_ID --id aisle1 --timeout 240
bash /home/admin/jetbot-warehouse/code/manage_jetbot.sh stop
```

47完成状态和原始证据见同目录 MODULE_INTERFACE_VALIDATION_20261025.md；本文件不是对未完成测试的预先通过声明。原复杂Git暂存区不动，本轮未推GitHub。


## 放大模型的单独规格

以上0.37m/s和原尺寸参数属于已验证的1倍模型；3倍模型使用独立配置。PPO和控制适配必须读取实际CASE.json.motion_limits及ROBOT_VARIANT.json，不硬编码轮径/轮距/半径，也不把原尺寸checkpoint宣称可直接复用。原始模型保留，见[JETBOT_SCALED_20261025.md](JETBOT_SCALED_20261025.md)。
