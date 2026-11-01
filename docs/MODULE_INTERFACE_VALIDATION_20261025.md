# 导航侧模块接口验收（2026-10-25）

用户范围：只给其他同学留接口；不实现 PPO、视觉检测、三维目标定位或训练。已读取用户提供的两份PDF，按照导航N/视觉V/PPO R的分工整理，文档中的课题规划不自动作为执行指令。

最终联调47：有头，原生IMU/原始轮速，FAST-LIVO+AMCL/Nav2保留，额外启用同相机RGB-D、导航侧控制仲裁和只读接收样例。停止后独立审计MODULE_INTERFACE_AUDIT passed=true，范围仅接口，不是算法验收。777帧RGB和777帧深度，481组RGB-D/标定配对样例有效；777组原生点云/扫描，零packet_discontinuities、桥接/门控/仲裁fault为空，原接口六项审计通过。停止后无相关遗留进程，云电脑保持开机。

任务记录完整保留：east_entry47成功(status4)；east_aisle47失败（planner_server未找到路径，operator报告WAITING_FOR_CONTROLLER_INPUT后取消/解除使能，native_status未存，不能假写status6）；随后east_handoff47成功(status4)。因此两次成功、一次失败，不是3/3导航成功。最后失败任务的更具体规划原因目前需查planner日志，这是待改进的错误信息传递边界。正常任务间IDLE不会误锁仲裁，真实源故障仍不自动恢复。

47由manager正常SIGINT结束，runner130符合请求停止；ROS/包装脚本子进程记录包含-2/254，不能称全exit0。固定后端未重建，旧43的SIGINT后-6历史仍保留。

隔离域52仲裁03测试通过：Nav2/PPO两种来源配置各只转发选定源；非选定源不泄漏；正常DISARMED空闲不锁下一任务；ARMED选定源失联归零并锁停，恢复消息不自动重新运动。测试只用合成Twist/TwistStamped消息，无PPO模型，没有连接仿真域51。02旧版基础互斥/断流检查通过；01测试程序只给python.sh包装进程发SIGINT导致清理超时失败，已精确停止遗留子进程并保留失败说明；03改成独立进程组收尾。

保留过程失败：44只读样例np.bool_序列化失败且底层命令超时；45只做静止中间检查后停止，无任务；46修正日志并用有限8次就绪回调清理后，传感器接收可用且无命令超时，但入口短目标原生status6失败。46还暴露正常IDLE超时误锁已修正，修正后的隔离03和47连续任务验证通过。监督器会检查接口子进程退出和桥接command_fault，不能把界面仍运行当验收成功。

深度：同一算法相机current_frame/render product，32FC1、米/光轴Z、NaN无效、同RGB header和CameraInfo，非独立镜头/观察相机。第一次捕获320×240，61,760有效像素，有限深度约0.100–5.103m。底部中央像素回投到chassis的地面Z约-0.03370m，与约-0.0335m底盘离地参考相符；这只是几何量纲检查，不能当真实传感器精度基准。

交付接口说明 docs/MODULE_INTERFACES.md，机器可读 extension_contract.json，人工地点目录 semantic_places.json，命名任务提交 semantic_goal.py，接收样例module_probe.py，受限速度仲裁command_mux.py。不固定PPO的36/72扇区、50D或归一化方案；这由PPO同学与checkpoint一致定义。视觉预留/perception/detections、/perception/object_poses及字段约定，但当前未装vision_msgs、没有视觉输出节点，正式对接前需与视觉同学确认消息类型/依赖。未来语义导航还需要目标实例与地图关联、歧义处理、可达接近点选择和任务层；人工名称查询不等于这些已经实现。

本轮不推GitHub、不修改原复杂暂存区、不操作电源。旧路径默认不开启extensions，使用module_interfaces47.json明确开启新对接入口。

备份完成：jetbot_module_interfaces_20261025_01.tar.gz，29,055,213B，SHA0343321430b9be2df015f7269bd721070b983ba291cbb9554409803d6c5cd31d；608文件本地逐一SHA通过，55份本地模块文件与归档云端部署内容一致。包含44–47过程案例和隔离仲裁01–03、代码/接口文档/所有任务结果，不含模型训练或完整Isaac资产库。
