# A类固定新增障碍：开发实验，尚未整任务通过

报告：docs/GO2_WAREHOUSE_FIXED_OBSTACLE_A.md；原始证据为m4_fixed_a_20261018。

所有试验保留原SLAM地图与AMCL；新增箱体来自M3主线程fixture，导航通过scan感知。safety_boxes只给独立保护桥，不给Nav2。默认nav_motion.yaml及原BT不变，以下新配置均为可选实验，未推广为稳定默认：

- navigate_valid_path.xml：复用安装版Nav2原生GlobalUpdatedGoal+IsPathValid，路径有效则保留，避免等价路径左右切换。
- nav_obstacle_motion.yaml：仅增加上述BT插件；DWB原参数。
- nav_obstacle_plan_safe.yaml / nav_obstacle_motion_safe.yaml：Global/Local radius=.60m、inflation=1.00m。
- nav_obstacle_motion_track.yaml：在safe基础上提高障碍/路径代价，GoalAlign=0、预测2秒。
- nav_obstacle_motion_arrival.yaml：在track基础上允许零线速并恢复RotateToGoal；巡航仍必须用真实odom验收>=.6m/s。此配置也失败，不作稳定配置使用。

manager可选参数GO2_NAV_PARAMS、GO2_NAV_BT、GO2_PLAN_PARAMS、GO2_GUARD_BOXES分别指定本轮控制参数、行为树、全局规划参数和独立保护几何。未指定时保留旧默认。不要同时启动多个planner或controller。

check_obstacle_navigation.py只请求NavigateToPose及保护开关，不直接发布速度。记录AMCL轨迹、真实odom、全局路径、运行目标速度及保护状态。持续驻足仅作信息项，未丢弃其原始记录；完整到达、速度和安全检查仍必要。

每轮实际启动命令保存在原始证据的go2_run_fixed_a*.sh；它们包含固定输出目录，重跑前须改用全新目录并核对起点初始化，不能直接覆盖或重复执行。用manager正常停止各本轮会话，不操作电源。
