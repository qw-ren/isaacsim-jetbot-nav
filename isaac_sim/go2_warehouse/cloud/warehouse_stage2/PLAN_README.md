# M1：仓储开发辅助地图与无运动规划

2026-10-15 实测。负责人任权伟，导航与系统集成。
本小实验完成 map→global Costmap→Navfn，尚未完成 DWB/目标点行走。
前置 scan 说明见 README.md。本实验不改稳定 work/baseline。

## 运行原理

build_dev_map.py 用现有 USD 库读取同一 warehouse_multiple_shelves.usd，
以与仿真一致的 /World/Warehouse 引用构建 CPU USD stage，不启动另一个仿真实例。
它核对本次仿真 scene.json 的米制/Z-up、场景边界和1878个碰撞prim。
对与离地0.05–0.85m相交的1077个碰撞AABB做保守栅格投影。

输出 warehouse_dev_v2 为 **开发辅助几何图，不是SLAM地图**：
501×797、5cm分辨率、原点(-12.5,-18.5)。
无选入碰撞体的矩形场景内部假设为自由空间，外侧边界未知。
AABB会扩大旋转/非凸/不连通网格的占据，不能代替精确通行性或身体扫掠碰撞验收。
实际地图不会由本轮规划测试改写，后续M2仍需一次SLAM建图及正式定位。

## 云端准备（不安装系统包）

代码部署在 /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_stage2。
复用已有 deps/ros_humble、Isaac Python和昨天隔离的RViz库。
prepare_nav_runtime.sh 下载/解包4个匹配Ubuntu22.04库包，总计约1845kB：
GraphicsMagick++、GraphicsMagick、libde265、libheif；版本固定在脚本。
库仅供规划进程设置 LD_LIBRARY_PATH，不替换系统或Isaac/CUDA/PyTorch。
准备脚本拒绝在地图/规划进程运行时覆盖其库文件。

```bash
# 云端root/admin：应在规划尚未启动时准备；现有云端已完成，无需重复下载。
bash /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_stage2/prepare_nav_runtime.sh

# 云端root：示例为本轮实际scene文件，新实验请使用自己的scene输出。
runuser -u admin -- bash -c '
  source /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_stage2/nav_env.sh
  /home/admin/isaacsim/python.sh \
    /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_stage2/build_dev_map.py \
    --scene /home/admin/go2-warehouse/artifacts/m1_plan_20261015/simulation/scene.json \
    --output /home/admin/go2-warehouse/nav_stage2/maps/warehouse_dev_v2
'
```

生成器拒绝覆盖已有输出目录；当前v2已生成，不要直接重跑上述生成命令。
新版本须使用新目录，并在 start_plan.sh 中明确选择它，不覆盖已验收地图。

## 启动与验证

先用阶段一manager检查已有仿真，不重复启动；需要有正在推进的/clock和Bridge真值TF。

```bash
# 云端root/admin：仅启动map_server、global Costmap、Navfn和lifecycle manager。
bash /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_stage2/manage_plan.sh start
bash /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_stage2/manage_plan.sh status
bash /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_stage2/manage_plan.sh stop

# 云端root：已有本轮RViz时不重复打开。
bash /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_stage2/start_plan_rviz.sh

# 云端root：仅ComputePathToPose，无速度发布；output必须是新目录。
runuser -u admin -- bash -c '
  source /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_stage2/nav_env.sh
  /home/admin/isaacsim/python.sh \
    /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_stage2/check_plan.py \
    --map-dir /home/admin/go2-warehouse/nav_stage2/maps/warehouse_dev_v2 \
    --output /home/admin/go2-warehouse/artifacts/my_plan_check
'
```

tmux规划会话为go2-plan-m1，stop只停止自己的规划进程，原仿真和scan保留。
不启动controller_server、bt_navigator、behavior_server、AMCL或SLAM。
暂用Bridge真值odom和静态单位map→odom；没有把真实位姿当作SLAM定位。
global Costmap目前仅StaticLayer+InflationLayer，实时障碍层留待M3。
robot_radius=0.45m是暂定开发包络，inflation_radius=0.70m；尚非完整Go2碰撞认证。

## 实际验收

v2首轮及仅规划独立重启各17项通过：
地图/代价地图生命周期active、use_sim_time=true、/map单发布者、文件与ROS栅格逐格一致、
代价膨胀存在、从当前TF位姿到(2,-1)的Navfn路径成功，路径约2m/78点。
以1cm间隔检查路径处于地图和代价地图自由区域，中心到低处碰撞AABB最小距离约1.205m。
目标(0,0.3)在障碍内、(100,100)在图外，两者均ABORTED且无路径；
这验证规划拒绝，**不是M4无路可走时的运动停车验收**。
机器人保持静止，无cmd_vel发布者，地图文件哈希不变。
RViz实际显示Map/Costmap/Scan Status正常和橙色计算路径。

v1保留为失败记录：灰度205配free_thresh=0.25导致未知边界被解析为自由空间，
首轮15/16通过、整体失败。v2改用灰度128表示未知，并添加逐格往返校验。
不能仅凭图像是灰色就认定ROS地图是unknown。

证据在 artifacts/m1_plan_20261015，已备份本地并保留成功/失败日志。
本轮仿真通过manager设置--run-seconds 1800，有30分钟运行上限；不是云电脑自动关机。
运行上限到期后必须先检查状态再使用manager恢复，不能假定会话持续运行。

参考官方1.1.20实现：
[MapIO灰度与阈值解析](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_map_server/src/map_io.cpp)、
[NavfnPlanner](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_navfn_planner/src/navfn_planner.cpp)。
