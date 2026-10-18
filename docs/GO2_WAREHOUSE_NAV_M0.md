# 阶段二 M0：Go2 仓储导航链路审查

日期：2026-10-14，Asia/Shanghai。负责人：任权伟（导航与系统集成）。
范围为已有仿真只读审查；本轮未启动Nav2/SLAM、发布速度、修改云运行源码或安装依赖。
阶段一已上传 feature/go2-warehouse-stage1，PR：
https://github.com/qw-ren/isaacsim-jetbot-nav/pull/1 ，提交 df7b678、098e8d0。另补充14日独立运动复测报告，本地/云端哈希一致、12项通过；本次M0没有重发运动命令。
本M0报告与实时观测暂存在本地独立 feature/go2-warehouse-nav-m0 分支，未推送团队仓库。

## 结论

阶段一仓储仿真底座已完成，用户已确认键盘操作；**当前云端仓储Nav2尚未运行，不能导航到目标**。
旧场景导航经验与参数可以参考，但旧地图、脚本路径和速度配置不能直接视为仓储可用。
GitHub阶段一是源码与精选证据归档；权重、依赖及资产按资源清单保留外部，不是完整镜像备份。

## 实时系统结构及验证

```text
warehouse-shelves + Go2 rough locomotion（已有单个tmux实例）
  → robot_data_manager
    → 真值odom、动态TF、点云、RGB/Depth
    ← /unitree_go2/cmd_vel 或 /cmd_vel
  → Isaac /clock
map→odom：Bridge静态单位变换，尚无SLAM/AMCL
Nav2/Costmap/map_server：未运行
```

12秒只读观察取得odom 132、点云130、clock 131、动态TF131条；未发现这些消息时间戳倒退。
首份点云62771个XYZ点全部有限，frame=unitree_go2/lidar_frame。
odom frame=odom、child=unitree_go2/base_link；当前机器人约(1.489,2.130,0.416)，
不是重新启动后的默认起点；来自用户已有键盘操作后的运行实例。
观测末尾平面速度约0.000039m/s，仅是该时刻数据，不作导航停车验收。
日志RTF约0.23–0.30，不能把配置freq=40理解为墙钟40Hz。

动态/tf只有robot_data_manager一个发布者。/tf_static有同一节点的三个DDS端点，
分别提供base→lidar、base→camera、map→odom；不是三节点重复发布同一TF。
base→lidar平移(0.2,0,0.2)，map→odom为零平移单位旋转。
/odom实际位姿与TF来自仿真真值，后续评测必须与SLAM/AMCL定位结果分开。

/scan和/map均无发布者。无planner_server/controller_server/bt_navigator/AMCL/SLAM导航节点。
两条cmd_vel均无外部ROS发布者，Bridge订阅正常。本轮observer没有创建速度发布者。
use_sim_time=true已由实际参数服务确认。

## 文件与配置审查

| 项目 | 当前事实 | 最小处理 |
| --- | --- | --- |
| 云运行副本 | baseline 12项SHA256通过；运行源码与10月13日备份一致 | 保留唯一稳定副本，阶段二在独立目录开发 |
| 仓储地图 | 云项目未找到地图PGM或map YAML；旧sim_map是旧场景 | 先验证仓储对应地图，不迁移旧障碍地图冒充仓储 |
| 旧启动脚本 | 引用/opt/ros/humble和/home/liswnux路径，云端不存在；旧清理逻辑会pkill | 新云启动入口复用隔离ROS环境，不执行旧脚本 |
| 全局规划器 | 配置Navfn/Dijkstra（use_astar=false） | 暂复用，无需新规划算法 |
| Local Costmap | ObstacleLayer直接订阅XYZ点云 + InflationLayer，odom frame | 待用真实点云验证地面过滤、TF和障碍标记 |
| Global Costmap | StaticLayer + InflationLayer，无实时障碍层 | 后续M3加入ObstacleLayer并实测，不提前修改 |
| 控制器 | DWB min_vel_x=0.5、max_vel_x=1.8、max_vel_theta=1.2 | 以阶段一0.25m/s/0.35rad/s量级开始，实测调参；旧值未经仓储验收 |
| footprint | local/global robot_radius=0.25 | 根据Go2身体/转身包络核查后设置，不沿用未经验证半径 |
| 控制里程计 | controller_server未显式配置odom_topic，只有bt_navigator配置了 | 设置controller的/unitree_go2/odom，避免订阅默认/odom |
| 控制输出 | YAML写cmd_vel_topic，但Humble控制器代码发布cmd_vel | 显式remap到独立导航速度入口；不能依赖这个YAML键改变输出 |
| 控制保护 | Bridge回调直接保存速度，未发现超时自动清零或限幅 | M1运动前加入仿真限幅/墙钟看门狗/停止流程，保留原策略 |
| 冗余odom工具 | pose_to_odom_republisher会再发布odom和同段TF，且有失效调用 | 不启动；继续复用Bridge已有odom/TF |
| 行为树 | 云ROS副本已有周期规划/恢复XML，但配置写/opt/ros路径 | 改为包share解析；M4先审查恢复动作再有限启用 |

控制器参数语义核对的是云包版本1.1.20的官方源码：
[ControllerServer](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_controller/src/controller_server.cpp)、
[OdomSubscriber](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/nav_2d_utils/include/nav_2d_utils/odom_subscriber.hpp)。
还未启动云控制器做运行参数/订阅检查，因此M1必须再次用实际节点graph验证。

## 依赖：包存在不代表可以启动

迁移ROS副本可列出Nav2、AMCL、SLAM Toolbox、pointcloud_to_laserscan。
在与现有Isaac/ROS相同的环境下ldd检查：

| 可执行文件 | 缺少的共享库 |
| --- | --- |
| controller_server、planner_server、amcl | liborocos-kdl.so.1.5 |
| map_server | 上项 + libyaml-cpp.so.0.7、libGraphicsMagick++-Q16.so.12 |
| async_slam_toolbox_node | liborocos-kdl.so.1.5、libboost_serialization.so.1.74.0、libtbb.so.12 |
| pointcloud_to_laserscan_node | 此项ldd未报告缺失库；未实际启动或检查动态插件 |

这些是实际环境中的链接缺失，不是要求重装ROS、Isaac、CUDA或torch。
M1先找现有可复用库，再仅补齐必要小型系统依赖；插件dlopen等仍需启动验证。

## 下一步最小实验（待确认M1）

先做M1第一个小实验：**站立时验证点云→二维scan及RViz障碍观测**。
不启动控制器、不发目标，确认高度裁剪排除地面且保留货架支架，frame、时间与TF一致。
点云高度是LiDAR坐标系中的高度，不能直接当世界离地高度；需要用实际外参及站立高度校验。
该实验复用已迁移pointcloud_to_laserscan，不添加视觉/PPO。

随后才补齐Nav2链接库、云路径、控制odom/remap与仿真速度保护，用对应仓储的开发辅助地图
加当前真值TF，先验证map/Costmap/Navfn短路径，再进行一个空走廊低速目标导航。
开发辅助地图须与USD碰撞几何对齐、保存版本，且明确不是SLAM成果；最终仍保留M2一次性SLAM
建图和正式定位。新增障碍A/B/C和JetBot迁移分别按M3以后推进。

## 核心原理

先验地图是保存下来的环境底图；TF/定位回答机器人当前在哪里；Costmap把底图与实时障碍转换
为规划代价。Navfn在全局代价地图上选路，DWB根据局部代价与实际速度输出短时控制。
因此有点云/odom不等于导航已打通，旧地图也不能代表当前仓储。
新增静态障碍应进入实时Costmap，保持保存的先验地图文件不变；不靠长期在线SLAM替代障碍更新。

原始只读观测见 ../experiments/go2_warehouse_nav_m0_20261014/runtime_observation.json；
observe_m0.py为本次实际执行的订阅脚本（不发速度）。本轮保留用户原有仿真运行，未关闭云电脑。
