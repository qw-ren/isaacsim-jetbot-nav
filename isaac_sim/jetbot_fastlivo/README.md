> 2026-10-25最新：原尺寸和3倍放大副本分别保留。放大版完成43.66m货架A/B，直行实测0.797m/s；使用warehouse_scaled_video53.json显式选择。原尺寸默认仍为原始轮径/0.37m/s。下方10.23记录是历史，不把两种物理条件混成一组实验。详见[放大版验收](../../docs/JETBOT_SCALED_20261025.md)。

# JetBot FAST-LIVO 仓储导航与有头运行

2026-10-23，任权伟的导航集成分支。使用同学 PR4 的实际 JetBot/full_warehouse 场景（commit `35fee2debcfb528e2aef86c91f82b06c2fd42869`），在独立副本中接入已验证的 Go2 FAST-LIVO 后端。团队 main、同学原 USD、Go2 稳定运行目录均保留。

## 已验证范围

- `motion_20261023_07`：基于 FAST-LIVO 估计位姿完成约0.7m前进、0.5rad转向与停车，实际 ROS 独立检查六项通过。
- `nav2_20261023_13`：原生 `NavigateToPose` 状态4/SUCCEEDED；配置出生点(0,6,0)，目标(1.5,6,0)。实际参考路程1.397m，末次估计目标距离0.1997m；离线仿真参考目标距离0.2403m。是基础功能接通，不是精密停靠通过。
- 原生IMU约200Hz，RGB320×240和点云约8.3Hz；点云每点24字节，ring/time/步长实际ROS检查通过，原生激光与视觉校正持续输出。
- 静态TF、估计odom、AMCL与唯一速度来源六项检查通过。隔离域52实际ROS门控断流验证：命令停止后归零，恢复消息或再次启用不会清除故障。真实底盘看门狗方法另做源SHA绑定的启动零速/运动断流/锁停验证。

- `nav2_20261023_23`：原生长程成功，实际35.90m、实际终点误差0.115m。
- `nav2_20261023_24`：长程行进中A托盘箱/B横向货架均触发并绕过；实际38.69m、实际终点误差0.0876m，六项审计及12子进程退出均通过。冻结配置为0.2m/s、无头。
- `operator_20261023_26`：有头模式连续三个独立原生目标成功，空闲停车后可接续；忙碌请求拒绝。命令峰值0.37m/s，直行参考速度约0.41m/s。六项审计通过；主动manager stop后runner130、ROS子进程-2/254保留为正常请求停止，不是所有子进程0。

`nav2_20261023_27`已用拉远抬高的新视角和原始轮速上限完成长程A/B：实际38.47m、实际终点误差0.119m、直行参考中位0.413m/s，六项审计/全部12子进程退出通过。`operator_20261023_28`已验证连续任务与长程A/B组合：先10m目标到达停车，再执行长程并触发A/B，2/2原生动作成功，实际总路程38.89m、最终误差0.114m，六项审计通过。完成后主动正常停止，runner130；不将停止信号产生的子进程-2/254当作全部exit0。Go2历史结果不直接算JetBot通过；终点误差不是整段定位误差。详见[长程与连续任务记录](../../docs/JETBOT_LONG_NAVIGATION_20261023.md)。

## 输入和 TF

| 数据 | Topic | Frame |
|---|---|---|
| 原生 IMU 加速度、角速度 | `/jetbot/imu` | `jetbot/imu` |
| 原生 RGB / CameraInfo | `/jetbot/image_raw` / `/jetbot/camera_info` | `jetbot/camera_optical` |
| 带 ring/包采集时间的点云 | `/jetbot/livo/points` | `jetbot/lidar` |
| 同批点云投影的真实扫描 | `/jetbot/scan` | `chassis` |
| 原生传播/校正输出 | `/jetbot/livo/imu_propagated` / `/jetbot/livo/filter_updates` | 后端内部坐标 |
| 导航估计里程计 | `/jetbot/odom` | `odom`→`chassis` |
| Nav2 速度 / 门控速度 | `/jetbot/nav_cmd_vel` / `/jetbot/cmd_vel` | m/s、rad/s |
| 仅离线参考 | `/truth/jetbot_odom` | 不进入估计/导航 |

```text
map --AMCL--> odom --FAST-LIVO适配--> chassis
                                    ├── jetbot/imu
                                    ├── jetbot/lidar
                                    └── jetbot/camera_optical
```

IMU 是场景中显式挂载的 Isaac 原生传感器，不假设 JetBot/Hesai 资产自带IMU。只传加速度和角速度，姿态协方差首项-1；不额外加噪。Hesai模型为原资产的 `Hesai_XT32_SD10`。IMU位于chassis上方0.06m、LiDAR上方0.10m，标定与杠杆臂显式补偿。相机保持原安装下俯约20度，根据实际USD矩阵转ROS光学轴，内参来自原生Camera。静态TF一次发送完整列表，支持晚加入订阅者。

原USD的ROS图和键盘图只在迁移副本禁用，由新runtime唯一发布传感器、clock和静态TF；原生ComputeOdometry的真值不当作导航里程计。FAST-LIVO原生TF重映射到内部评价话题。算法不订阅轮式odom/仿真位姿；已知CAD出生坐标只用于一次AMCL初始化。

## 地图与运动配置

先验图由3473个仓库碰撞prim中536个低位AABB投影（高度0.035–0.30m、5cm栅格），属于开发用CAD先验，矩形内未被占用处假定自由，不是SLAM或完备可通行保证。scan按同一批真实点云的实际固定安装变换投影到chassis，筛选高度0.03–0.40m，保留采集时间，不依赖额外转换队列。

底盘和DWB仍为差速(v,w)，当前命令上限0.37m/s、0.6rad/s，转弯按左右轮速耦合降速，半径0.16m；不把Go2的0.55m保护半径或巡航速度原封照搬。AMCL采用`OmniMotionModel`描述LIVO校正可包含横向位移的里程计噪声，这不是将底盘改为全向控制。旧差分噪声模型在本次低速LIVO增量上导致过大的转向噪声，失败与敏感性诊断保留。

原始Isaac JetBot资产两轮上限719.635deg/s（12.56rad/s），属性栈无队友场景覆盖，运行时保持原值，不修改电机或关节上限。按控制器名义半径0.03m折算直线约0.3768m/s；原始轮子几何半径约0.0336m，本次实际直行约0.41m/s，命令和实测不能混用。

导航速度门检查实际scan、估计姿态/odom、clock、命令期限、唯一来源和预测圆形包络净距；过期归零并锁停，不自动清除故障。底盘内部0.35s失联期限保留，只有零速的启动等待不锁死运行；真正运动指令失联会锁停。所有配置用于本仿真。

## 云端操作

运行根 `/home/admin/jetbot-warehouse`，ROS域51；Go2域41及目录保留。先查看状态，启动器也会拒绝重复仿真。

```bash
bash /home/admin/jetbot-warehouse/code/manage_jetbot.sh status
# 只有所有既有任务结束后，以新ID启动：
bash /home/admin/jetbot-warehouse/code/manage_jetbot.sh start UNIQUE_ID 100 nav2
bash /home/admin/jetbot-warehouse/code/manage_jetbot.sh stop
```

启动会创建新case、解决已有资产路径、冻结源码/地图/配置并启动唯一tmux监督器。结果见`NAVIGATION_RESULT.json`、`INPUT_AUDIT.json`、`GATE_RESULT.json`、`RUNNER_RESULT.json`。`source_files.sha256`对应实际测试冻结源；源码包后续只改善模板携带/管理命令，不改变13的判定记录。

继续复用现有Isaac Sim4.5、ROS2、FAST-LIVO二进制和补充库，不安装CUDA/PyTorch。二进制SHA固定为`8a8e61dd543befa5f9da26bdf2674af804f7d2d5a81542d9f1fb7a9cf11e958f`；社区ROS2移植来源/补丁沿用Go2已有PROVENANCE。`livo_base.yaml`逐字节复制B09已验证模板，实际生成参数覆盖JetBot的topic、内外参和IMU传播设置。

原生逐点时间未验证：使用五个实际渲染包的采集时间，包间隔可能20/40ms，不冒充精确逐线去畸变或MID360。较长路线、动态障碍及误差统计见已完成案例23/24；原生逐点时序仍是限制。

## 保留的失败历史

01环境脚本nounset，02采样/组包未接通，03/04的NumPy点云填充被压成22字节（已明确标废），05静止LIVO输出正常但晚加入TF缺失，06消息相对clock的15ms采样差触发保护，07短运动通过但旧内置审计关闭时异常（独立外部审计通过），08默认BT依赖缺项，09转换scan断流，10启动零速超时误锁，11位姿校正差分误报速度跳变，12差分AMCL噪声模型下目标超时，13首次基础Nav2通过。原记录不覆盖、不改名为成功。

## 后续较长路线与动态场景

可选第六参数是路线 JSON 文件：
```bash
bash /home/admin/jetbot-warehouse/code/manage_jetbot.sh start UNIQUE_ID 280 nav2 stationary /absolute/route.json
```
`goals` 为地图坐标 `[x,y,yaw]` 列表；多个目标使用真正的 NavigateThroughPoses，一次动作到最终目标，原单目标默认行为保留。`timeout_wall_s` 最大1800秒；模拟时长最大600秒，runtime_wall_s=max(600,导航时限+100)。传感器/命令看门狗期限不随总时限放宽。配置内容进入新case的CASE.json，源码/地图仍冻结。

动态托盘箱(A)及横向货架(B)由独立`dynamic_fixture.py`在仿真线程建立实际碰撞体；根据导航反馈的估计位置和速度门非零前进状态触发，不读取真值、不修改先验地图、不移动机器人。事件与实际碰撞部件写入`obstacle_events.jsonl`。`navigation_paths.jsonl`保留实际全局/局部规划，必须结合原生动作结果判断，插入成功本身不是绕障通过。

`evaluate_case.py --case ...`仅结束后离线读取真值轨迹、计算路程与近似圆形底盘净距，绝不被导航/控制导入。仍需区分地图AABB预检、原生导航成功及离线碰撞参考。

扫描有效回波取每个角度最近距离；无有效高度带回波的角度为NaN，障碍层`inf_is_valid=false`。不能将高度过滤后空缺的角度当成穿透障碍的自由空间。全局代价地图定期发布完整网格用于只读审计，A/B出现后保存快照；这不向地图写入已知障碍坐标。

## 观察视角与连续任务

后续启动默认有头；动态读取现有admin桌面DISPLAY/XAUTHORITY路径，不保存认证内容。缺少有效桌面时明确报错，不静默改为无头。观察相机独立为`/World/JetbotObserverCamera`：车后3.5m、高2.2m，看向前方2m/高0.35m，24mm焦距；只使用位姿做显示跟随，FAST-LIVO车载相机及其标定不变。

所有新案例必须使用新ID，先检查已有实例。连续任务可在云桌面终端执行：

```bash
# /absolute/operator.json 例：{"goals":[[0,6,0]],"timeout_wall_s":900}
# goals仅用于统一配置校验；operator不自动执行该列表。
bash /home/admin/jetbot-warehouse/code/manage_jetbot.sh start UNIQUE_OPERATOR_ID 240 operator stationary /absolute/operator.json
# 等待该case OPERATOR_READY.json，并确认OPERATOR_STATUS.json为IDLE：
python3 /home/admin/jetbot-warehouse/code/submit_goal.py --case /home/admin/jetbot-warehouse/artifacts/UNIQUE_OPERATOR_ID --id goal1 --x -3 --y 6 --yaw 3.141592653589793 --timeout 240
# 下一任务必须使用新任务ID，等待前一RESULT.json及IDLE；提交端退出不会取消任务。
bash /home/admin/jetbot-warehouse/code/manage_jetbot.sh stop
```

每次原生动作结果位于`missions/任务ID/RESULT.json`，不是根目录NAVIGATION_RESULT；唯一监督器仍有总运行时限。`submit_goal.py`只提交不可变请求，不发布速度。门控故障后要求新运行，不自动清除锁停；正常任务完成后解除使能允许下个目标。结束后的逐任务精度诊断使用`diagnose_endpoint.py --case ... --mission goal1`，真值只用于离线评估。


## 2026-10-25 原生采集时间与平面扫描去畸变

新增路线参数 `point_time_mode: "native_frame_rays"`：同一 LiDAR render product 的 `IsaacReadTimes.simulationTime` 关联帧末时刻，SDK `deltaTimes` 给出帧内逐点偏移；检查帧号偏移恒定与时间新鲜度，不用世界位姿换算时间。帧关联验证证据是 timing_20261025_37。SDK timestampNs 的时钟域不同，不能直接当 ROS 仿真时间。

同一路线可加 `scan_deskew_gyro: true`，要求 native_frame_rays。只对二维 LaserScan 使用原生 IMU 的角速度积分，将不同曝光时刻回波旋转到扫描中点；不补偿平移，不读取真值、绝对姿态或轮式里程计。IMU orientation 仍标无效；有效的是实际发布的角速度、加速度。历史覆盖不全则跳过扫描并计数，不伪造有效回波。点云本身仍给 FAST-LIVO 使用其自身估计器处理。

这些参数为显式选择，兼容默认值未改。`warehouse_native_deskew_ab.json` 是同一路线且关闭录像的便捷配置；已完成39验证使用的是录像开启的jetbot_deskew39.json，40为其独立复测，不将关闭录像配置称为已经复测。先检查现存实例，再用新ID启动；参见 docs/JETBOT_VIDEO_PRECISION_20261025.md 的实时结果及历史失败。


## 货架通道录像（2026-10-25）

`warehouse_aisle_video43.json` 明确进入北侧既有货架区：入口横向通道、x=-8北行、y=27绕货架端部、x=-13南行，实测约44m。A箱体在y=16、B两米小货架在y=22，行进中按估计y向上越阈触发；是部分占道侧向绕行，非完全封路。41起步180度掉头未进展失败；42导航完成但镜头转角穿架；43为最终交付录像。失败记录保留。

观察相机新增observer_follow.py：固定世界高度、平滑平面位置/航向、限制转速和小角度死区；只为显示读取CAD及已插入props，检查镜头到车体视线，必要时收近再缓慢拉回。该几何不送给规划器或滤波器，车载算法相机和导航输入未改。evaluate_aisle_observer.py仅在完成后验证实走通道里程、障碍位置、视线/高度/转速，需结合成片抽帧检查。

43原生动作和接口通过；固定后端SIGINT清理时exit-6仍保留，不能称全子进程0。最新精度和备份边界见docs/JETBOT_VIDEO_PRECISION_20261025.md。没有GitHub推送。


## 给PPO与视觉同学留接口（2026-10-25）

采用显式module_interfaces47.json配置，可选同相机RGB-D、导航侧启动时二选一控制仲裁、只读接收样例。参见docs/MODULE_INTERFACES.md与MODULE_INTERFACE_VALIDATION_20261025.md。仅提供集成接口，不实现模型或训练；视觉输出消息/依赖待双方对齐。旧运行配置默认不开这些扩展。47接口联调与隔离域仲裁检查通过，任务2成功/1规划失败；正常SIGINT停止，不称全部子进程0。
