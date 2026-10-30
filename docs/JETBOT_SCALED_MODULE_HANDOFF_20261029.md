# 3倍 JetBot：视觉与 PPO 对接更新（2026-10-29）

任权伟负责导航、定位、全局规划及集成；本交付只提供接口，不实现视觉模型、PPO训练或推理。旧版 MODULE_INTERFACES.md 的 0.37m/s 属于1倍模型；当前采用独立3倍配置，本文件与运行 CASE.json 优先。

2026-10-30补充：[导航参数与精度口径](JETBOT_NAV_PARAMETERS_20261030.md)。显式启用`odom_twist_projection=differential_planar`时，`/jetbot/odom`的twist中vy/vz为差速平面约束值0；位姿/TF、前向速度和原生IMU角速度保留，完整原始速度仍记录并检查。PPO观测应与训练时配置一致，不把该0当作滤波器独立测得的侧向速度。默认native模式仍保留。

## 几何和控制

| 项目 | 当前3倍模型 |
|---|---|
| 外接尺寸（含传感器） | 约0.491×0.400×0.780 m |
| 轮半径 / 轮距 | 0.1008658983 / 0.2802 m |
| 原始轮关节上限 | 12.560000333 rad/s，未修改 |
| 车体速度上限 | 0.8 m/s；角速度0.6 rad/s |
| 物理保守圆半径 | 0.34 m |
| 安全门扫描点到预测中心轨迹阈值 / 预测时域 | 约0.40 m / 1.5 s；按0.34m实体圆，额外间距约0.06m |
| Nav2规划半径 / 膨胀半径 | 0.65 / 1.0 m；规划裕量不等于实体尺寸 |
| 显式配置质量 | 底盘54 kg，左右轮各6.75 kg；67.5 kg不是含全部传感器/支承的总质量声明 |

放大质量按立方、显式惯量按五次方缩放/由物理引擎重算；动力学已经改变。不能把1倍模型的归一化、轮径、checkpoint或奖励尺度直接复用。正向轴 chassis：+X前、+Y左、+Z上。最终门限还受轮速耦合：v_cap=min(0.8,max(0,ω轮上限×轮半径−|车体ω|×轮距/2))。读取当次 CASE.json.motion_limits、ROBOT_VARIANT.json 和 JOINT_LIMITS.json；不要绕过门限。

## 视觉输入及标定

RGB `/jetbot/image_raw` 为320×240 rgb8；CameraInfo `/jetbot/camera_info`；frame `jetbot/camera_optical`。实际 fx=fy=102.1276626586914，cx=160，cy=120。光学坐标+Z前、+X右、+Y下。相机安装位置相对 chassis 约[0.1395004,0,0.2016013] m，向下约20°；完整旋转矩阵、camera↔lidar 和 imu↔lidar 标定见随附 camera_contract.json（不要混用 Rcl 与 chassis→camera 的方向）。

可选 `/jetbot/depth/image_rect_raw` 为32FC1米制光轴Z深度，RGB同时间戳/同光学frame/同K；NaN无效。开启 extensions.depth=true。回投 X=(u−cx)Z/fx、Y=(v−cy)Z/fy，再按采集时间查询TF。放大的是安装平移；深度和K不乘3。展示用观察相机与录像箭头不能作为算法输入。

雷达 `/jetbot/livo/points` 为 frame jetbot/lidar 的 PointCloud2（stride24：xyz/intensity/time/ring，逐射线原生时间）；Hesai XT32，安装[0,0,0.30] m。原生IMU `/jetbot/imu` 安装[0,0,0.18] m，使用加速度和角速度；orientation_covariance[0]=-1表示没有提供有效姿态估计，并非没用IMU。姿态读取估计里程计。全链路 `map→odom→chassis→传感器`；AMCL独占map→odom、估计适配器独占odom→chassis，模块不要重复发布TF。

输出预留 `/perception/detections`、`/perception/object_poses`，建议 vision_msgs Detection2DArray/Detection3DArray；当前未安装vision_msgs，无检测发布者/消费节点，属于需与视觉同学确认的协议。二维框不是地图目标；需深度/雷达关联、时间戳TF、置信度及导航侧批准的可达接近位姿。

## PPO输入、动作和保护

保留FAST-LIVO+AMCL定位与Nav2全局规划。`/jetbot/scan` 是chassis系1440格LaserScan，角度−π起、步长π/720，量程0.15–20m，无效为NaN/未知；高度筛选0.03–0.40 m，只有角速度去畸变，尚无平移补偿。实际观测频率约8Hz，不能硬编码必须10Hz。`/jetbot/odom` 是估计里程计（odom→chassis，车体速度），`/plan` 是map系全局Path；按时间TF转换。扇区数、历史堆叠、归一化及网络维数由PPO同学与checkpoint一致定义，导航不固定36/72格。

PPO只发布 `/jetbot/policy_proposal`，geometry_msgs/TwistStamped，frame_id=chassis，stamp=/clock仿真时间；linear.x单位m/s、angular.z单位rad/s，其余置零。不是归一化动作或轮速。开启 extensions.command_mux=true，control_source=ppo，控制源在新case启动时冻结。Nav2提案 `/jetbot/nav2_proposal`；仲裁独占 `/jetbot/nav_cmd_vel`；速度门独占 `/jetbot/cmd_vel`。禁止模型直接发后两个话题。不能在行进中热切控制权。

提案接收时龄−0.025..0.25仿真秒；选定源wall超时0.25秒或sim时龄>0.30秒归零锁停；不自动切换到Nav2、不在消息恢复后自动启动。最终门25Hz，clock/odom watchdog0.5wall秒，scan0.8、input0.35；scan/odom仿真时龄≤0.30秒，执行器失联0.35秒停车。限速、净距、有效性保护必须保留。

本轮 ROS_DOMAIN_ID=52 隔离合成通信测试验证了±0.8m/s、±0.6rad/s、轮速约8.765<12.56、选定源失联锁停和无自动恢复；不是PPO模型导航验收。RGB-D与仲裁实车仿真测试结果单独归档；录像为减轻负载不同时开启深度。

## 语义导航边界

现有语义目录只是人工名称→批准接近位姿。视觉空间目标经实例关联、歧义/置信度校验和可达接近点选择后才交给Nav2。语言理解、识别模型、策略训练由同学提供；不让模型绕过控制仲裁、修改TF或读取truth。truth仅用于离线评分与展示相机投影。

机器可读入口：isaac_sim/jetbot_fastlivo/extension_contract.json；参数快照 experiments/jetbot_scaled_interfaces_20261029/。运行配置 warehouse_scaled_interface_20261029.json。
