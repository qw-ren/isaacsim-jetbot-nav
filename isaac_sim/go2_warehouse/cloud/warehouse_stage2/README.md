# 阶段二：scan 传感器准备

2026-10-14 在已有 Go2 仓储仿真上实测，负责人任权伟。
这是 M1 的第一个小实验；完整 Nav2 目标导航尚未完成。
新增文件为本次导航适配，不属于上游 Go2 源码。

## 数据流与裁剪原理

```text
Bridge 真值 odom + base→LiDAR 静态外参
  → scan_ground_frame.py
  → odom→unitree_go2/scan_ground（LiDAR 地面投影位置、仅保留 yaw）
Bridge XYZ 点云
  → 现有 pointcloud_to_laserscan（target_frame=scan_ground）
  → /scan → RViz
```

当前仓储地面 z=0。高度带 0.10–1.20m、范围 0.30–12m，360 条射线。
虚拟帧从 Bridge 的实时外参和同时间戳 odom 求得，不再手写另一套 LiDAR 外参。
不改写 Bridge 的 base、LiDAR 或 map→odom TF；仅发布新的 scan_ground 子帧。
虚拟帧与原 LiDAR 原点的水平位置一致，避免把点云转换到 odom 原点后错误地计算距离。

初始实测横滚约 -1.58°，LiDAR 离地约0.613m。
直接在倾斜 LiDAR 帧裁剪[-0.4,0.8]m时，12m范围内仍混入597个地面点；
所以选择地面投影帧，以真实离地高度过滤。

## 云端目录与运行

稳定仿真继续位于 /home/admin/go2-warehouse/work，不覆盖它。
本目录脚本部署到 /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_stage2。
使用原 cloud_env.sh、Isaac Python、ROS Domain 41/localhost/FastDDS/use_sim_time=true。

```bash
# 云端 root 或 admin；先确认已有仿真，不重复启动
bash /home/admin/go2-warehouse/work/cloud/warehouse_stage1/manage_warehouse.sh status
bash /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_stage2/manage_scan.sh start
bash /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_stage2/manage_scan.sh status
bash /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_stage2/manage_scan.sh stop
```

scan 使用独立 tmux 会话 go2-scan-m1；start 拒绝已有会话或已有 /scan 发布者。
stop 仅停止自己的虚拟帧和点云转换进程，不停止仿真。
pointcloud_to_laserscan 只有在 /scan 存在订阅者时才订阅输入点云，这是原插件行为。

```bash
# 云端 root：检查器不发布速度；output 必须是尚不存在的新目录
runuser -u admin -- bash -c '
  source /home/admin/go2-warehouse/work/cloud/warehouse_stage1/cloud_env.sh
  /home/admin/isaacsim/python.sh \
    /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_stage2/check_scan.py \
    --output /home/admin/go2-warehouse/artifacts/my_scan_check
'

# 云端 root：打开 RViz；已有 RViz 时不要重复运行
bash /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_stage2/start_scan_rviz.sh
```

RViz fixed frame=odom，绿色为 /scan，Go2 base 轴表示机器人位置；
可勾选 Raw 3D cloud 对照原点云。两端均使用 Best Effort QoS。
本轮已打开独立 go2-scan-rviz tmux 会话供云桌面查看。

## RViz 依赖隔离

不安装系统包。prepare_rviz_runtime.sh 在数据盘下载并解包6个 Ubuntu22.04 amd64
运行库包（约3.47MB下载、约11.6MB解包），仅为 RViz 设置局部 LD_LIBRARY_PATH。
包名/版本固定在脚本中，下载校验在 nav_stage2/deps/rviz_debs/packages.sha256。

初次尝试的 Isaac 同名 Assimp 库缺少 RViz 所需符号，已保留失败日志。
不能仅凭 .so 名称或 ldd 无缺失就认为 ABI 兼容，最终以实际 RViz 插件加载验证。
Isaac/CUDA/PyTorch、系统包和原仿真环境未替换。

## 验收与限制

首轮163组严格同时间戳点云/scan/odom配对，独立重启155组；各12项检查通过。
独立 SciPy 坐标变换与实际scan返回值比对，匹配率100%，首轮最大距离误差约1.61e-6m。
每份scan277–279条有效返回，源数据约2.27万个地面点被排除，保留约5900个0.10–0.50m障碍点。
原始点云、独立投影和 JSON 留在 artifacts/m1_scan_20261014，并已备份本地。
RViz 实际打开且目视 Status: Ok；停止、独立重启和重复启动拒绝均实测。

仅验证当前姿态、位置和静止场景。0.10m以下与1.20m以上障碍被有意排除，
不是完整 Go2 身体碰撞覆盖；运动去畸变、传感器多帧累积、移动时TF和SLAM质量尚未验收。
scan_time=0.025是观测到的仿真消息周期；time_increment=0不是逐射线采样时间。
不适用于楼梯或非平面地面，也不是移植到JetBot后可直接复用的参数。
本轮没有启动 SLAM/Nav2、生成先验地图、发送速度或验证目标到达。

官方转换行为参考：
[pointcloud_to_laserscan Humble源码](https://github.com/ros-perception/pointcloud_to_laserscan/blob/humble/src/pointcloud_to_laserscan_node.cpp)。
