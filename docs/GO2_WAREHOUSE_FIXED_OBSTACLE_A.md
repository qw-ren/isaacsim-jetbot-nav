# 固定新增障碍的实际导航 A（2026-10-18）

## 任务与保护

使用现有SLAM走廊先验地图、AMCL、Navfn和DWB。机器人固定起点(2,-1)，箱体中心(4,-1,.5)、尺寸(.6,.6,1)m，目标约为map下起点前方4m。箱体不写入先验地图，导航通过实际/scan和Costmap感知。此次固定存在，不涉及已知的移除残留问题。

新增箱体AABB加入独立速度保护桥的safety_boxes.json，用于仿真安全兜底，不输入Nav2的地图或规划器。保持1.5秒预测、.55m预测净距、前向scan阈值和失联停车。DWB指令范围仍为.6–.8m/s、角速度±.6rad/s。

按用户要求，持续驻足精度作为informational_checks保留，不作为本次A类任务通过的前置；姿态、保护、几何净距、到达、巡航与地图不变仍检查。实际目标误差照常记录。

## 首轮实际问题与最小调整

navigation_01使用原周期重规划BT。三条全局路径中，前两条选箱体负y侧，第三条改选正y侧；DWB指令从负角速度切成正角速度。保护桥在真实中心距箱体仍约1.34m时，因预测净距.5029m低于.55m而锁停。动作取消，没有到达。

navigation_02采用安装包内Nav2官方“路径有效则保留，失效或目标更新再规划”逻辑（GlobalUpdatedGoal+IsPathValid），全程只发布一条全局路径，消除了上述左右切换。机器人已实际前进并转弯，巡航样本均值.76896m/s，但预测净距.54987m再次低于.55m而锁停；最后真实中心距箱体最小.85388m，未触及.45m净距判据。

由此发现原规划/控制robot_radius=.45m，而保护桥要求预测净距.55m，控制器认为可行的动作仍可能被保护桥拒绝。navigation_03仅针对本轮将Global/Local Costmap半径统一为更保守的.60m，膨胀半径从.70m增为1.00m；保护桥阈值、原地图、速度限制保持不变。历史参数文件保留，默认启动行为不变。

首轮失败检查器的巡航统计曾包含错误后的停车观察窗，不用于说明有效巡航；后续检查器用abort_wall截断，原始JSON保留不覆盖。

## 关键代码

团队导出目录 cloud/warehouse_stage2：check_obstacle_navigation.py；navigate_valid_path.xml；nav_obstacle_motion.yaml及safe版本；nav_obstacle_plan_safe.yaml。
manage/start_navigation新增可选GO2_NAV_BT，默认原BT；manage/start_obstacle_plan新增可选GO2_PLAN_PARAMS；manage/start_velocity_guard新增可选GO2_GUARD_BOXES。均保留单例拒绝逻辑。

未修改原work/baseline、控制策略、Isaac/CUDA/PyTorch，未训练。实际轨迹与状态保存在data/cloud_warehouse/m4_fixed_a_20261018，不将规划成功冒充实际到达。

第三轮现场核对：Global/Local Costmap参数robot_radius均为.6，进程命令行指向safe配置，排除未加载配置。实际巡航.77320m/s，但仍因预测净距锁停，最小真实中心距箱体.84123m、未到达。扩大半径本身不足以解决执行问题。

第四轮保留safe配置，仅调整DWB：BaseObstacle.scale .02→.20、PathDist.scale32→64、GoalAlign.scale24→0、sim_time1.5→2秒。目的是更早避开障碍并跟随绕行路径，减少直接朝最终目标切弯；结果见后续记录，不预先宣称通过。

第四轮实际结果：动作ABORTED（6），DWB明确报告264条候选轨迹均不可行（BaseObstacle/Trajectory Hits Obstacle）。未触发保护桥锁停，未到达；最后中心距箱体至少1.330m，3个巡航样本不足以做速度验收。

第五轮为单独的预警距离对照：只将配置起点x2→x1，目标区域仍约x6，箱体与第四轮DWB/Costmap/BT/保护参数不变。AMCL仍用已保存SLAM对齐关系和配置已知起点初始化，不连续注入真实位姿。此对照不得替代原x2起点的失败案例。

第五轮对照实际绕过了箱体：巡航均值.70962m/s、337个样本，中心距箱体最小.83160m，横向最大偏移1.223m、AMCL更新52次。但终点附近绕圈后因预测净距再次锁停，整体false；最终map误差.4397m、偏航误差1.4799rad，不能记作NavigateToPose通过。

第六轮保留x1起点与同一箱体/目标，处理原先硬性最低线速导致近目标不能减速/原地对正的约束冲突：min_vel_x/min_speed_xy设为0，恢复原生RotateToGoal（scale32、slowing_factor5、lookahead_time=-1）。这允许启停/转弯阶段零线速，巡航实测均值>=.6m/s仍为必要检查，最高线速.8、角速度上限.6及全部保护门槛保持不变。commands_bounded检查0<=vx<=.8，巡航要求另由真实odom统计，不把最低命令速度当作实测速度。

第六轮也未通过：接近阶段因预测净距锁停，巡航.76935m/s、77样本，未越过箱体。该到点参数没有得到完整任务验证，不作为稳定默认。

## 保护几何重放的新发现

按每次锁停的真实pose、输入v/w，逐点重建保护桥1.5秒预测轨迹，对全部1078个AABB重新计算净距：

- 第1/2轮最近障碍为新增箱体，预测净距.50293/.54987m。
- 第3/5/6轮最近障碍均为 `/World/Warehouse/SM_PillarPartA_9M6_1455/SM_PillarPartA_9M/Section0` 的整块AABB，预测净距.54013/.53844/.54567m。
- 该盒范围x约.412–9.512、y约-3.20–-2.80、z约0–9m。此处用的是整块高构件包围盒，尚未按机器人高度验证实际碰撞截面。因此**过度保守/误拦截只是待验证假设，不能宣布已证明**。

诊断文件为guard_geometry_diagnostic.json。下一步应审查该构件在机器人高度范围的真实碰撞几何与预测轨迹关系，优先解决保护几何与导航可行区域的一致性，再从已实际绕行的案例复测到点。不继续无依据叠加DWB调参，不降低.55m保护门槛。

## 本轮结论

已实测自动规划并实际绕过固定新增箱体，样例巡航.710m/s、最小中心净距.832m；完整NavigateToPose未通过，六轮原始失败记录全部保留。未验证原x2起点可完成任务、未验证独立成功复现、未进入B/C测试。地图文件与/map始终不变，不能将本轮标为完整A类验收通过。

## 收尾与备份

20:31所有本轮会话已正常退出，无仿真/AMCL/控制器/规划器/转换进程残留，ROS CLI daemon停止，云电脑仍开机。94项源码云/本地校验、baseline12项与稳定work3项均通过。501个原始归档文件约217MB已取回本地，并按cloud_files.sha256逐项校验通过。完整源码为本轮收尾版本；各轮配置/启动命令和原始JSON均保留。未改原暂存区、未commit/push，实验变体没有作为默认部署。

后续更新：20:55已由原USD核实高构件整块投影误拦，修正高度几何后在开阔带完成A类冷启动16项检查，见CLOUD_WAREHOUSE_OPEN_OBSTACLE_A.md。原六轮失败与旧位置未通过的事实不变。
