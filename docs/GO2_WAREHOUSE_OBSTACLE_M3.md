# M3 实时障碍 Costmap 实验（2026-10-18）

## 实际范围

沿用保存的SLAM走廊地图、map_server、AMCL和Navfn。Global Costmap 在 StaticLayer 与 InflationLayer 之间新增原生 ObstacleLayer，输入现有 `/scan`。未改原先验地图、unknown规则、机器人半径.45m或膨胀.70m。

本轮没有controller/BT/速度桥；现场 `/unitree_go2/cmd_vel` Publisher count=0、Subscription count=1。机器人只站立。新静态箱体中心(4,-1,.5)、尺寸(.6,.6,1)m，机器人配置起点(2,-1)，目标约(6,-1)，均为仿真坐标。

## 验证结果

箱体出现前，箱体所在地图区域自由，路径穿过未来箱体位置。箱体出现后，实际LiDAR观测使Global Costmap出现致命表面及膨胀区域，Navfn重新请求得到绕行路径。先验文件哈希和发布的`/map`网格不变。

| 记录 | 箱体出现后的路径点数 | 路径中心距箱体保守净距 | 清除结果 |
|---|---:|---:|---|
| check_01，隐藏并关闭碰撞 | 184 | .4768m | 失败，残留一个致命格 |
| check_02，实际RemovePrim、独立重启 | 185 | .4848m | 失败，残留一个致命格 |

净距以箱体外接圆为边界，是对真实箱体净距的保守下界，不是整个仓库的碰撞认证。两次机器人最大位移均约.0196m，无运动命令。

首轮检查器错误要求箱体中心格为100；实际雷达标记的是可见表面，内部是膨胀代价99。第二轮改为检查箱体边界邻域的致命格，观测到13格，其他门槛不变。首轮原始失败JSON未覆盖。

移除后取到sim25.45s的原始三维点云，在已删除箱体的旧表面位置仍有4个点，约x3.697–3.700、y-1.028、z.168–.252m。因此目前存在雷达输入残留，不能只归因于Costmap清除；也未通过调用clear_costmap或扩大过滤距离掩盖它。

第三轮 `check_03` 仅在隔离M3副本尝试 `keepOnlyPositiveDistance=False`：致命表面13格、绕行185点、净距.4921m，但移除后仍残留2个致命格；10/11检查通过，整体false。该实验没有支持这一参数能解决问题的假设，因此停止后恢复修改前的传感器代码，不采用该诊断参数。失败版本源码单独归档。

## 文件与后续

代码在团队导出目录的 `cloud/warehouse_stage2/`：nav_obstacles.yaml、manage/start_obstacle_plan.sh、m3_obstacle_fixture.py、prepare_obstacle_copy.py、check_obstacle_costmap.py。原work/baseline及此前world-LiDAR运行副本保留。

证据在 `data/cloud_warehouse/m3_costmap_20261018/`；网格、scan和路径为实测NPZ，图由这些数据绘制。M3已验证“新增静态障碍标记→全局路径改变”，尚未验证实际Go2绕行、B替代通道或C安全终止，移除清除限制必须保留，不能宣告整个M3通过。

后续优先围绕课程任务推进：固定新增静态障碍的A类行走实验，继而扩大地图覆盖支持B/C。FAST-LIO2/LIVO2仍按NAV_PLAN列为后续计划，不插入本轮。

## 收尾

20:07本轮仿真、scan、定位、规划会话均已正常停止，无相关进程残留，云电脑仍开机。baseline12项、稳定work3项及86个stage2源码文件SHA通过。完整云端证据292个文件已取回并逐一SHA256核验通过（cloud_files.sha256），包含原始失败记录、运行源码、日志、地图及点云。未改原暂存区、未commit/push。
