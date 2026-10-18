# Go2 仓储 M1b：转弯、低速响应与驻足诊断

2026-10-15，11:35–12:05云端实际试验。结论：**巡航速度满足要求，但稳定转弯/到点驻足未通过，不升级为正式导航配置。** 先前两次固定直通道通过的历史结论保留，不外推为通用导航成功。

## 实测结果

所有导航目标都由真正的NavigateToPose/Navfn/DWB链路执行，最终判定同时检查action、停车后位置/朝向、持续线角速度、姿态和几何。外部要求位置误差≤0.25m、朝向误差≤0.30rad；最后2秒仿真时间线速<0.05m/s、角速<0.10rad/s，另有5秒仿真停车观察。巡航统计远离起终点且|实测角速|<0.2rad/s的样本，允许转弯/刹停低速；短目标的巡航样本有限，不能当作长期速度保证。

| 试验 | action | 巡航均速m/s | 停车后误差m / rad | 结论 |
| --- | --- | --- | --- | --- |
| diagonal_01：40Hz原goal checker，(0,-1)→(4,1) | 成功 | 0.777 | 0.227 / 0.189 | 首轮10项通过；尚未加入持续角速度检查，不能代表后续严格标准 |
| turn_back_02：从前一实际终点→(6,-1) | 成功 | 0.765 | 0.233 / 0.493 | 朝向失败 |
| diagonal_stopped_v2：StoppedGoalChecker，40Hz | 成功 | 0.776 | 0.297 / 0.040 | 停车后位置失败 |
| turn_back_stopped_v2 | 成功 | 0.764 | 0.226 / 0.269 | 持续线/角速度失败 |
| diagonal_rough50：相同checker，仅50Hz对照 | 成功 | 0.773 | 0.277 / 0.027 | 位置失败；50Hz没有单独解决问题 |
| turn_back_rough50_tight：内部8cm窗口 | 中止 | 0.783 | 0.056 / 0.223 | 低速旋转无进展，巡航仅5个样本 |
| final_diagonal_01：8cm+较短旋转评分 | 中止 | 0.772 | 0.169 / 0.186 | 近目标低速混合命令未推进 |
| final015_diagonal：内部15cm窗口 | 中止 | 0.772 | 0.204 / 0.111 | 仍未进入窗口、无进展 |
| short_diagonal_01：DWB时域0.8s对照 | 成功 | 0.775 | 0.265 / 0.070 | 最后位置仍超过0.25m，计失败 |

末次短时域试验到点瞬间误差0.099556m，之后5.02秒仿真身体位移0.207628m，最终误差0.264502m；到点0.2秒墙钟后收到的Bridge目标反馈全部为零。不是命令未发送或导航节点未启动就可以解释的问题。

## 已定位的控制响应问题

独立原始命令/Bridge张量目标反馈/真值odom对照（response_diagnostics.json）：

- turn_back_rough50_tight末段7.48秒仿真：底层角速度目标平均0.0600rad/s，实际平均绝对角速仅0.000647rad/s，净朝向变化-0.000520rad；目标没有形成持续旋转，导致progress checker中止。
- final015_diagonal末段7.48秒：底层前向目标0.072727m/s、角速目标平均0.130588rad/s，实际平均前向速度0.000046m/s、净位移仅0.000029m。不能把DWB的理想低速预测直接等同于Go2当前策略的实际运动。
- 到点后零指令并不保证身体位置立即固定。停止阈值瞬时满足也不代表之后不会发生步态姿态调整。最后一轮即使持续停车检查通过，早期停车调整仍可把位置推离合格范围。

以上是在这些具体姿态/命令下的实测响应不足，不是完整的通用“死区模型”，尚未标定角速/线速全区间，也没有证明唯一根因。应先独立修复/标定驻足与刹停，不继续通过放宽最终目标标准来掩盖问题。

## 做过的对照与保留边界

1. nav_turn.yaml支持原地转向和接近目标减速，前向上限0.8、角速±0.6，使用StoppedGoalChecker。内部位置窗口经历0.20→0.08→0.15m；外部0.25m标准始终不变。恢复RotateToGoal，lookahead_time从-1改0.3s，作为独立实验，不替换已验收的nav_motion.yaml。
2. 40Hz与50Hz只通过启动参数freq=50对照，原cfg默认40不改。云端Isaac Lab常规Go2配置为dt=0.005、decimation=4，但没有取得旧权重训练元数据，不能断言50Hz就是其真实训练周期，更不能宣称50Hz已修复驻足。
3. 使用已有flat权重的独立motion_flat_20261015仅做0.8m/s速度/刹停对照，停车高度最低0.194635m，触发0.20m姿态下限，失败后立即停止，未用于Nav2。原rough副本、work、baseline、两份权重和环境均不改；没有训练、下载权重或重装。
4. 最后nav_turn_short.yaml将DWB sim_time从1.5减到0.8秒；独立速度保护的静态前瞻保持1.5秒，所有空间保护阈值不变。该候选仍未通过，未采用，也不声称与原碰撞能力等价。
5. 使用Humble官方源码核对RotateToGoal.lookahead_time仅影响旋转评分预测位置；不把这项参数修改解释成完整解决低速响应。[Nav2 Humble RotateToGoal源码](https://api.nav2.org/nav2-humble/html/rotate__to__goal_8cpp_source.html)。

## scan问题与新增诊断

本轮没有复现FRONT_SCAN_CLEARANCE，旧问题仍开放，不计为修复。新增capture_guard_fault_stamped.py按同一时间戳配对scan、点云、odom，记录首次锁停输入，避免用不同时刻的位姿误判自身回波；各观察窗口未捕获有效故障事件。fault_flat观察跨过一次实例重启，仅null结果，不用于源点归因。

velocity_guard的行为阈值和默认DISARMED保持，增加pose、输入、scan/odom/clock时间戳、前方最小点距离与角度的状态诊断。scan/导航manager支持显式GO2_SCAN_PARAMS/GO2_NAV_PARAMS，默认仍指向旧40Hz扫描和已验收直通道配置；50Hz另用scan_converter_50.yaml，名义scan_time=0.020。尚未验证运动去畸变、窄通道或新增障碍重规划。

另外发现旧check_motion.py的posture_bounded字段仅检查倾角，虽然高度异常会让no_error失败，字段本身仍可能为true。本轮补入0.20–0.85m高度检查，并以所有本轮原始odom离线复核；flat正确判为false。check_navigation_turn也同步补入高度条件。原始JSON不改；最终源码快照含此后续修正，未重新跑完整云端试验，已做语法与真实遥测离线复核。

## 核验、归档和当前状态

停止前实时审计7项通过：六个Nav2生命周期active/use_sim_time=true，唯一控制输入/底层输出、无/cmd_vel旁路发布、预期两条动态TF、保护DISARMED输出零。参数确认StoppedGoalChecker、0.15m/0.10rad、停止线角速各0.05、DWB sim_time0.8、旋转评分0.3。**接口审计通过不等于实际导航通过。**

最终47个阶段二源码文件本地/云端SHA256一致；baseline12项、稳定work三个原始补丁源文件SHA256通过。Python及shell语法通过；没有改原工作树暂存区、commit或push。阶段二仍未推送，阶段一PR状态未动。

完整数据：云端artifacts/m1_turn_20261015；本地data/cloud_warehouse/m1_turn_20261015，包含所有失败、原始遥测、配置对照、日志、最终源码、flat副本元数据，以及可重复运行的analyze_response.py与诊断图。精选证据仅在独立团队工作目录experiments/go2_warehouse_turn_m1_20261015。

2026-10-15 12:05:31核验：仿真、scan、planner、controller/BT、guard均已停止，无isaac_go2_ros2.py进程；只保留此前的go2-plan-rviz。没有关机、重启或操作云电源。两个受保护rough/flat副本都保留，实验参数没有设为默认。

下一步：先独立标定零指令驻足、刹停和低速命令响应，核对旧权重的实际观测/控制周期契约，再选择驻足或控制适配方案；用同一批目标及相同外部误差阈值复测。暂不进入SLAM/AMCL、新增障碍或JetBot迁移，不把这轮诊断写成M1通用导航完成。
