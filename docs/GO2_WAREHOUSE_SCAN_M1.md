# 阶段二 M1：点云转scan与站立观测（已通过）

2026-10-14，Asia/Shanghai。负责人任权伟，导航与系统集成。
本次完成M1的传感器准备小实验，**完整先验地图Nav2目标点导航尚未完成**。
本地独立开发分支feature/go2-warehouse-scan-m1；本次阶段二内容未推送团队仓库。

## 实际完成

复用用户17:22启动的唯一Go2仓储仿真；未重复启动或重启它。
在独立nav_stage2目录运行两个节点：go2_scan_ground_frame和go2_cloud_to_scan。
后者复用已迁移的pointcloud_to_laserscan，不重写点云转换算法。
/scan单发布者，Best Effort/Volatile，360条射线，range 0.30–12m。
新增odom→unitree_go2/scan_ground TF，不改变原Bridge已有TF。
当前ROS_DOMAIN_ID=41、localhost/FastDDS/use_sim_time=true。

## 为什么增加虚拟扫描坐标系

同时间戳点云/odom采样：base高度0.416m、LiDAR离地0.613m，横滚约-1.58°。
按原LiDAR帧z裁剪[-0.4,0.8]m、半径12m，会混入597个世界地面点。
收紧原帧下限到-0.2m虽去掉地面，却只剩672个0.10–0.50m低处障碍点。

所以使用新的地面投影帧：原点在LiDAR的地面投影，z=0、仅保留yaw。
由实时odom与Bridge的base→LiDAR外参计算，TF时间戳保持与源odom一致。
先把点云变换到这个帧，再按离地0.10–1.20m裁剪；水平扫描原点仍在LiDAR位置，
不是以odom世界原点计算距离。仅适用当前平面仓储地面。

## 测试与证据

| 检查 | 实际结果 |
| --- | --- |
| 首轮15秒 | 163组同时间戳点云/scan/odom配对，12项检查通过 |
| 独立停止后重启15秒 | 155组配对，12项通过；原仿真始终保持运行 |
| 每份scan有效返回 | 首轮277–279 / 360条射线；无NaN，有限值在range限制内 |
| 独立投影比对 | SciPy坐标变换重算源点，首轮所有有限返回匹配；最大距离误差约1.61e-6m |
| 时间 | 未见时间戳倒退；scan仿真消息周期中位数约0.025s，墙钟约10–11Hz |
| 地面过滤 | 原始每份约22696–22816个地面点（世界z绝对值<0.05）均不落入障碍带 |
| 低处障碍保留 | 每份5915–5978个0.10–0.50m障碍点保留 |
| 货架佐证 | 首份5942个低处点中1011个落在货架支架碰撞AABB的3cm邻域，涉及23个AABB |
| 机器人静止 | 两轮最大平面位置变化约2.67e-7m；无外部cmd_vel发布者，本程序不发布速度 |
| TF | 原动态TF由Bridge发布，新子帧由scan节点发布；graph只有这两个动态TF发布节点，源码无同段重复 |
| 启停 | 重复start被拒绝；stop后无转换/虚拟帧残留进程，原仿真会话保留；再start通过 |
| RViz | 实际打开、已目视截图，Global Status和Scan Status均Ok；订阅端Best Effort与scan发布端匹配 |

TF检查限度：现有Humble rclpy回调不提供publisher metadata；没有把每条TF消息来源认证当成已完成测试。
当前节点graph与独占子帧源码审查用于确认没有已知重复发布。
货架AABB对照只是离线几何佐证，不是精确网格表面、全身碰撞或导航地图验收。

## 改动与运行环境

新增源码位于isaac_sim/go2_warehouse/cloud/warehouse_stage2：
scan_ground_frame.py、scan_converter.yaml、start_scan.sh、manage_scan.sh、check_scan.py、
capture_cloud.py、scan_view.rviz、start_scan_rviz.sh、prepare_rviz_runtime.sh、README.md。
部署到/home/admin/go2-warehouse/nav_stage2；未修改work/baseline或现有策略。

RViz首次尝试借用Isaac同名Assimp库产生undefined symbol，不能使用。
最终在nav_stage2/deps下载并解包6个Ubuntu22.04 amd64库包，下载3474kB；
仅RViz进程设置独立LD_LIBRARY_PATH，不安装系统包、不替换Isaac/CUDA/PyTorch。
库包名和版本：libassimp5 5.2.2~ds0-1、libdraco4 1.5.2+dfsg-2、libminizip1 1.1-8build1、
libyaml-cpp0.7 0.7.0+dfsg-8build1、libignition-math6-6 6.10.0+ds3-7、liborocos-kdl1.5 1.5.1-2build1。
源码prepare脚本固定版本，packages.sha256保留下载文件哈希。

失败记录保留：首次检查器回调API不兼容、RViz Assimp符号缺失；均不算通过。
RViz初始加载订阅时曾产生一次QoS警告；配置载入后两端Best Effort匹配，截图Status Ok。
当前保留go2-warehouse、go2-scan-m1、go2-scan-rviz会话，供用户云桌面查看。

## 边界与下一步

尚无SLAM地图、AMCL定位或Nav2目标到达验证。当前输入是仿真真值odom，不能作为SLAM成果。
0.10m以下或1.20m以上障碍不在此scan内；完整碰撞覆盖、移动时TF、多帧点云累积与去畸变未验收。
scan_time=0.025记录消息周期，time_increment=0不提供真实逐射线采样时间。

下一步最小任务：补齐Nav2独立运行库和云路径，建立仓储对应的开发辅助地图，
先只运行map_server/Costmap/Navfn核查地图和短路径，不发送运动；
运动前再完成低速限幅、墙钟看门狗与停止流程，然后验证一个走廊目标。
开发辅助几何图不能代替M2最终一次性SLAM建图与正式定位成果。

完整原始数据：云端artifacts/m1_scan_20261014；本地data/cloud_warehouse/m1_scan_20261014。
精选证据：experiments/go2_warehouse_scan_m1_20261014，包含两份JSON、RViz截图和投影图。
可复用云命令见command.txt和阶段二脚本README。
