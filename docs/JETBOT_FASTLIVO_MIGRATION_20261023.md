# JetBot FAST-LIVO 迁移，2026-10-23

已推送当前Go2检查点：团队PR5的HEAD `46215a4a02a40a881bdc7a4afa6cb89482004c5f`；个人PR3已合并main `9cae44045e26b1c98d8fe47a9541d3aa27ba459b`。两段固定后方视频发布到[Go2新Release](https://github.com/qw-ren/go2-navigation-experiments/releases/tag/go2-showcase-20261023)，GitHub大小与SHA和本地一致。标准B09通过与展示12后段保护停车的范围保持分开。

同学[PR4](https://github.com/qw-ren/isaacsim-jetbot-nav/pull/4)当前head `35fee2debcfb528e2aef86c91f82b06c2fd42869`，原USD SHA `5e5bccec677ce49396cca73d14152afb220dbccbee14337d6aa4762a0427ba66`。LiDAR/IMU的消息frame已统一为lidar/imu，实际camera节点已加入静态TF，键盘图active=false。仍有Hesai绝对路径，README“全部相对路径”需修正；迁移器在副本中解决全部三项资产引用，云端现有资产均可解析。原生ComputeOdometry是真值而不是轮式估计，迁移后只保留独立评价，不参与导航。原场景与团队main未改，迁移在单独worktree本地合入PR4。

本地代码 `team_export/jetbot-fastlivo-20261023/isaac_sim/jetbot_fastlivo`；云端新根 `/home/admin/jetbot-warehouse`，ROS域51。复用原有FAST-LIVO二进制、原生IMU、Hesai XT32与原生RGB，显式标定相机光学轴和IMU杠杆臂；唯一估计TF `odom→chassis`，AMCL发布`map→odom`，三传感器静态TF整组发布。未使用轮式里程计或位置/姿态真值作为估计/导航输入。

实际验证：07估计闭环前进、转向、停车通过，独立ROS审计6项通过。13首次原生NavigateToPose SUCCEEDED，目标(1.5,6)，末次估计目标距离0.1997m；离线参考路程1.397m、参考目标距离0.2403m，属于功能验收，不是精密停靠。13传感器/TF六项通过，372条估计odom、159条原生校正、2044条原生IMU、84个图像和点云，无适配器/速度门/底层故障。监督器exit0后已停止；目标检查器在清理竞态中SIGINT(-2)，结果文件已完整写入，其余受管进程退出0，后续管理器增加了正常等待。

12差分AMCL模型下未完成目标，保留原始数据；离线模型敏感性显示LIVO横向校正可能产生0.69rad的差分转向噪声标准差代理量，而允许横向增量的模型对应0.019rad。13使用OmniMotionModel描述里程计噪声，DWB/底盘仍差速；目标容差、速度与避障限值未因此改变。这是单案例功能结果，不是统计比较结论。

旧失败01–12均保留，03/04 wire格式无效显式标记，不计为传感器/算法验收。五个渲染包使用实际采集时间，逐点时间尚未验证；地图是3473个碰撞prim中的536个低位AABB开发先验，不是SLAM地图。JetBot长程、A/B动态障碍、独立完整复测仍未通过，不外推Go2历史结果。

隔离ROS域52验证实际速度门断流停零、输入恢复/再次启用均不清除故障，源码SHA绑定。底层实际command方法验证启动零速等待不锁死、运动断流归零锁停、恢复消息不自动重启。环境/资产/Go2运行副本未重装、删除或改写，不操作电源。最终源、过程、归档核验见交接记录的最新补记。
