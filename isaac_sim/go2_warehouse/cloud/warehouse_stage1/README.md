# Go2 仓储云仿真：阶段一运行说明

本目录服务于 Go2 Isaac Sim 仿真迁移。它不启动 SLAM、Nav2、训练或真机接口。
技术验收见 [阶段记录](../../../../docs/GO2_WAREHOUSE_STAGE1.md)。

## 已迁移的实际工程

云端数据盘挂载在 `/home`。本次独立目录为 `/home/admin/go2-warehouse`：

```text
baseline/       本地实际代码、配置与策略权重的未修改快照
work/           应用云端适配后的运行副本
deps/IsaacLab/  本地 Isaac Lab v2.3.0 source/apps 与许可证
deps/ros_humble/ 本地 ROS Humble 运行文件的独立副本
deps/python/    只补齐缺失的小型 Python 模块
cache/          本项目临时文件与缓存
logs/           启动及检查日志
artifacts/      场景元数据、截图、ROS 检查 JSON；失败记录也保留
```

复用了镜像的 `/home/admin/isaacsim` 与
`/home/admin/isaacsim_assets/Assets/Isaac/4.5`。
没有迁移本地 7.2 GB 安装包、真实场地地图、bag、PCD、SDK、Fast-LIO 工作空间、
SCAN/VLA 工程或旧日志。Go2 模型与策略来源仍属于原工程及 Isaac Lab，不是新增原创模型。

## 在云电脑执行

通过原有 SSH 连接进入云端，使用以下命令；root 会由脚本切换到桌面用户 admin。
密码由 SSH 交互输入，不保存在本目录。

```bash
# 云端 root：在 tmux 中启动；SSH 断开后进程仍继续运行。
bash /home/admin/go2-warehouse/work/cloud/warehouse_stage1/manage_warehouse.sh start

# 云端 root：状态与停止。stop 只停止仿真，不关云电脑。
bash /home/admin/go2-warehouse/work/cloud/warehouse_stage1/manage_warehouse.sh status
bash /home/admin/go2-warehouse/work/cloud/warehouse_stage1/manage_warehouse.sh stop

# 云端 root：查看 tmux 会话；按 Ctrl-B 然后 D 退出查看，保留运行。
runuser -u admin -- tmux attach -t go2-warehouse

# 云端 root：ROS 命令使用与 Isaac 相同的 Python 与 ROS 环境。
bash /home/admin/go2-warehouse/work/cloud/warehouse_stage1/ros2_cloud.sh topic list --no-daemon
bash /home/admin/go2-warehouse/work/cloud/warehouse_stage1/ros2_cloud.sh param get /robot_data_manager use_sim_time
```

桌面启动使用已验证的 `DISPLAY=:1` 与 admin 的 Xauthority。
在无影客户端打开现有 Linux 桌面即可看到 Isaac 窗口；没有重新配置图形服务器。

默认场景 `warehouse-shelves`，资产为
`/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd`。
使用图像自带的本地资源，不要求远程 Nucleus 或重新下载场景。
默认位置 `(0, -1, 0.4)` m，yaw=0；根据真实 USD 碰撞包围盒选取，而不是假设原点为空。
策略加载 `ckpts/unitree_go2/rough_model_7850.pt`，只运行推理。

可指定一个有界的运行窗口或独立证据目录，例如：

```bash
# 云端 root：活跃仿真阶段运行 300 秒后退出；启动预热不计入这 300 秒。
bash /home/admin/go2-warehouse/work/cloud/warehouse_stage1/manage_warehouse.sh start \
  --run-seconds 300 --artifact-dir /home/admin/go2-warehouse/artifacts/my_run
```

## 在云端检查消息与运动

以下检查程序会发送短时速度命令，最后持续发送零速度。
只用于这台云电脑上的仿真，使用 ROS Domain 41 与 localhost DDS。
运动阶段按仿真时间计算；不能同时启动其他速度发布者。

```bash
# 云端 root：完整检查，速度为前进 0.25 m/s、转向 0.35 rad/s。
runuser -u admin -- bash -c '
  source /home/admin/go2-warehouse/work/cloud/warehouse_stage1/cloud_env.sh
  /home/admin/isaacsim/python.sh \
    /home/admin/go2-warehouse/work/cloud/warehouse_stage1/check_ros_interfaces.py \
    --motion --output /home/admin/go2-warehouse/artifacts/manual_check
'
```

去掉 `--motion` 可以只检查站立与接口，但检查程序仍发送零速度。
增加 `--cmd-topic /cmd_vel` 可以检查 Nav2 常用速度接口。
新的检查请使用独立 output 目录，避免覆盖已有证据。

## 云端副本的适配

`baseline.sha256` 记录原本地文件哈希；云端 baseline 已逐项校验。
`local_sim_changes.diff` 保存相对原 Git HEAD 的仿真修改。
`cloud_runtime.patch` 保存云端副本相对本地快照的修改：

- 仿真入口增加启动位置、有限运行时间、截图和场景元数据输出。
- 状态与 RTX 数据在仿真线程读取；ROS 执行器只处理命令与时间消息。
- 直接设置仿真时间，修正 CameraInfo frame，按需导入 cv_bridge。
- 显式使用 LiDAR 传感器坐标，与 PointCloud2 frame 对应。
- 独立执行器顺序退出，日志每两秒输出一次状态。
- 更新已弃用的 height scanner 参数，并使用较低的跟随视角。

在另一个匹配的 baseline 副本中，可以通过 `patch -p1 < cloud_runtime.patch` 复现这些适配。
不要对原本地稳定版本直接应用这个补丁。独立依赖目录需一并迁移或按记录恢复。
本次没有重装 Isaac Sim、CUDA、PyTorch 或替换镜像的软件环境。

## 下一阶段的接口

| 用途 | Topic / TF | 类型 / 说明 |
| --- | --- | --- |
| 控制 | `/unitree_go2/cmd_vel`、`/cmd_vel` | `geometry_msgs/msg/Twist` |
| 里程计 | `/unitree_go2/odom` | `nav_msgs/msg/Odometry`，仿真真值 |
| 位姿 | `/unitree_go2/pose` | `geometry_msgs/msg/PoseStamped` |
| LiDAR | `/unitree_go2/lidar/point_cloud` | `sensor_msgs/msg/PointCloud2`，XYZ |
| RGB | `/unitree_go2/front_cam/color_image` | `sensor_msgs/msg/Image`，rgb8，640×480 |
| 深度 | `/unitree_go2/front_cam/depth_image` | `sensor_msgs/msg/Image`，32FC1，640×480 |
| 内参 | `/unitree_go2/front_cam/info` | `sensor_msgs/msg/CameraInfo` |
| 仿真时间 | `/clock` | `rosgraph_msgs/msg/Clock`；ROS 节点使用 `use_sim_time=true` |
| 动态 TF | `odom → unitree_go2/base_link` | Bridge 单节点发布 |
| 静态 TF | `base_link → lidar_frame / front_cam` | 前缀均为 `unitree_go2/` |
| 临时全局关系 | `map → odom` | 真值环境中的静态单位变换，不是 SLAM 定位 |

后续如果 SLAM/localization 发布 `map → odom`，启动时添加
`+publish_map_to_odom=false`，让这一段 TF 只有一个发布者。
旧障碍场的地图不能用于新仓储；本阶段没有生成仓储导航地图或验证 Nav2。
语义分割暂时关闭，虽然旧 Bridge 的订阅/可视化发布器会让相关名称出现在 topic list 中。

## 数据保存与关机

工程、资产、输出及 Isaac 默认日志都位于 `/home` 数据盘；系统盘没有被大型安装包占用。
本阶段证据另备份至笔记本 `data/cloud_warehouse/stage1_20261013/`。
SSH 断开、停止 tmux 和云电脑关机是不同操作。
先 stop 仿真并同步代码、配置和实验成果，再由用户在云电脑控制台决定关机。

当前账号的计费产品、余额、自动关机及实例释放策略不能从 SSH 确认。
请在控制台核对。阿里云的规则区分关机与资源释放，不能把数据盘视为永久备份：
[开关机说明](https://help.aliyun.com/zh/wuying-workspace/user-guide/start-stop-or-release-cloud-computers)、
[按量付费说明](https://help.aliyun.com/zh/wuying-workspace/product-overview/pay-as-you-go)。

NVIDIA 的 [RTX LiDAR 节点说明](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/sensors/isaacsim_sensors_rtx_lidar/node_descriptions.html)
解释了坐标变换与多帧 scan buffer。当前未验证逐点时间戳、去畸变、在线 SLAM 或动态避障。
