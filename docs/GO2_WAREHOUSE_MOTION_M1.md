# Go2 仓储阶段二 M1：受保护的直通道目标导航

2026-10-15 本次实测。阶段一历史验收及其GitHub PR保持独立；阶段二尚未提交/推送。

## 本次结论

独立运动副本完成速度/刹停/失联保护和两次真实 NavigateToPose 直通道导航，其中第二次完整停止并重启仿真、scan、规划、控制和保护节点。各轮导航9项检查全部通过。当前仅确认这一固定走廊基线，不把M1扩展解释为任意仓储目标、转弯或新增障碍已完成。

| 验收 | 实测结果 |
| --- | --- |
| 0.6m/s指令 | 末段平均0.558m/s，低于用户巡航要求；仅作为跟踪/刹停标定 |
| 0.8m/s指令 | 末段平均0.781m/s；零目标反馈延迟0.0725s，之后位移0.2184m |
| 输入断流 | 0.3634s后底层目标归零；之后位移0.1480m；恢复输入不自动重新启用 |
| 保护进程退出 | 仅终止本轮自建节点；底层主循环0.5346s独立清零；之后位移0.2000m |
| 导航首轮 navigation_v7_diagnostic | SUCCEEDED；巡航均速0.767783m/s；停车后误差0.157702m |
| 完整重启 navigation_reproduction | SUCCEEDED；巡航均速0.768677m/s；停车后误差0.118826m |

导航起点约(0,-1)，目标(6,-1)，朝向0；约8.2秒仿真时间、34秒墙钟到达，另观察5秒仿真停车。两轮最后2秒内线速均低于0.05m/s；最终线速分别0.000511/0.000818m/s。最小机器人中心至低处USD碰撞AABB距离1.1773/1.1768m；采用临时0.45m圆半径，不能当作全身净空认证。巡航均速取启动2秒之后、x>1且距离目标至少1m的实测机体系前向速度，不把起步、接近目标或停车算作巡航，也不保证步态中每一瞬时值都≥0.6。

## 代码与运行边界

稳定work与baseline不改。prepare_motion_copy.py校验三个源文件哈希，创建motion_work_20261015，应用motion_runtime.patch；不覆盖已有目录。副本关闭键盘和/cmd_vel旁路；Bridge限制0–0.8m/s、无横移、角速±0.6rad/s，主循环独立墙钟0.5s看门狗。runtime_cmd反馈控制目标张量，实测运动看odom。

外部velocity_guard默认DISARMED、墙钟20Hz；检查输入0.35s与odom/scan/clock各0.5s新鲜度、姿态、唯一发布者、1.5秒已知静态AABB前瞻及前方scan；锁停后必须显式启用。故障注入测试使用0.6m/s，不声称0.8m/s故障制动距离已标定。保护不是急停硬件、完整制动模型或真机认证。

Navfn全局规划+静态地图/膨胀层，DWB局部rolling costmap采用实时scan障碍/膨胀层；全局实时障碍层尚未实现。controller明确使用/unitree_go2/odom；输出显式重映射/nav_m1/cmd_vel，再经唯一保护节点送/unitree_go2/cmd_vel。使用现有Bridge真值map→odom，无AMCL/SLAM。

DWB前向范围0.6–0.8m/s，不在输出端强行提速。直通道参数去掉RotateToGoal评分，因为它的接近减速要求与最低前向速度冲突；SimpleGoalChecker位置0.20m、朝向0.5rad，到点后发布零。此配置尚不支持通用目标朝向原地调整或窄道低速策略；保留停车后误差≤0.25m独立验收。

启动scan前等非零/clock稳定，启动导航前等odom/clock与planner active，验收脚本再等真实新鲜scan及BT active，避免启动未完成就发目标。重启后的两轮使用相同配置及固定起点；唯一仿真由已有manage_warehouse.sh管理。

仅增加libboost-context1.74.0 1.74.0-14ubuntu3（223kB下载），解包至nav_stage2/deps/bt_runtime；不apt install，不改变Isaac/CUDA/PyTorch/系统ROS。

## 实时接口审计

首轮及完整重启后各7项审计通过：map_server/planner_server/controller_server/bt_navigator/local_costmap/global_costmap均active且use_sim_time=true；DWB min/max为0.6/0.8，odom路径正确；底层速度唯一发布者go2_sim_velocity_guard，输入唯一controller_server，/cmd_vel无发布者；保护DISARMED且输出零。

/tf两个发布者为robot_data_manager与go2_scan_ground_frame，观测边仅odom→base_link与odom→scan_ground，结合源码核查未引入重复TF；旧rclpy回调不能取得每条消息的发布者元信息，未声称逐消息归属已认证。

本地/云端40个阶段二文件SHA256一致；baseline12项和稳定work三个补丁源文件哈希通过；开发图四文件哈希未变。Python/shell/BT XML语法检查通过。

## 保留的失败与限制

- speed_0p6初轮验收脚本把服务成功前的旧DISARMED状态误判失败，修正等待启用反馈后speed_0p6_v2通过。
- 累积行走后的input_timeout及gateway_death初轮分别被静态前瞻/前方scan保护拒绝；未计故障测试通过。固定起点独立复测通过。
- 导航v1目标默认整数类型不满足ROS浮点消息；v2真正行走巡航0.7676m/s、停车误差0.1569m，但RotateToGoal拒绝全部接近轨迹，action ABORTED，严格计失败。
- v3发送目标早于BT active而被拒绝；v4/v5缺新鲜scan而未启用。修正启动和验收就绪检查。
- v6前行约0.4m后FRONT_SCAN_CLEARANCE锁停，最终停车稳定性检查也未通过；原因未闭环确认为自身回波或实际近点。不能把后续两次成功解释为该偶发故障已根治。增加只读原始点云故障抓取；v7未再触发，观察期未捕获故障快照。
- 初次生命周期配置回复超时，正常退出时新建manager卡住；核验完整命令/父PID/启动时刻后通过pidfd仅终止该manager，再在时钟稳定后重启成功。此操作不涉及云电源。
- 原始scan检查live_scan_check在运动期间运行，69组独立投影一致，但因有控制发布者和运动而不满足“站立检查”两项；未计为站立验收通过。
- 仅5秒停车观察、两轮固定直通道成功；未证明长时停车、转弯、不同起点/布局可靠率或一般导航速度下限。暂无SLAM、AMCL、新增障碍三类任务、JetBot本体或学习算法。

## 证据与当前状态

云端artifacts/m1_motion_20261015，本地data/cloud_warehouse/m1_motion_20261015，含成功/失败JSON、完整遥测、日志和源快照。团队仓库experiments/go2_warehouse_motion_m1_20261015仅放精选报告、审计和图；源码在isaac_sim/go2_warehouse/cloud/warehouse_stage2。

末次仿真11:25:46启动，--run-seconds 1800，约11:56到期；不是云电脑自动关机策略。交接时保留唯一仿真、scan、规划、导航、默认禁用运动的保护和原RViz会话。机器人无活跃目标，保护输出零；用户重用前必须先查manager与数据新鲜度。未操作电源、重装环境、改原暂存区或推送阶段二。

下一小阶段先验证转弯/不同起点与前方scan偶发锁停来源，再进行正式SLAM先验图和AMCL定位；新增障碍重规划仍属后续阶段。
