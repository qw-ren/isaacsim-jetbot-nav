# 仓储导航主线入口

在云电脑终端执行，复用当前 Isaac Sim 4.5、ROS 和已有单例 manager。无需切分支或重装环境。

```bash
# 双通道：先验证实体封上路后规划切换，再实际沿下路导航。
bash /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_stage2/run_mainline_case.sh start branch

# 无路径：正常定位后静止封闭起点，导航失败并保持禁止运动；拆墙后同目标恢复规划。
bash /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_stage2/run_mainline_case.sh start closed

# 查看任务，或请求正常结束（两个场景不能同时启动）。
bash /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_stage2/run_mainline_case.sh status
bash /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_stage2/run_mainline_case.sh stop
```

每轮创建新的 `artifacts/mainline_<时间>_<场景>`，也可在 start 的场景参数后指定唯一 case ID；已存在目录拒绝覆盖。每轮源码复制到该目录的 source，运行参数写入 CASE.json。监督任务放在 admin 的 `go2-mainline-task` tmux 中，SSH 断开不终止任务；结果在 runner.log、RUNNER_RESULT.json 和 task/ 或 navigation/ 中，失败原始记录同样保留。结束或中断时，通过原有 manager 正常停止本轮启动的组件，不操作电源。

依赖已持久化的 B 先验地图、已验证的运行副本和高度裁剪安全几何。保护桥是速度唯一出口；closed 全程禁止运动，branch 仅由实际导航观察器在获得控制命令后显式解锁，结束后解除。两者都使用已有地图/AMCL与仿真真实odom，FAST-LIO2仍为隔离已部署模块。本阶段不做运动中突然封路。

区域到达参数使用已说明的 0.4m 目标区域、不要求最终朝向；巡航和终点精度只记录，碰撞/失联停车保持。closed 的结果只证明起点四墙封闭下的完整无路径任务处理，不代表远端双通道均封的普适验证。

本入口由原有场景实际命令整理而来。云端实际运行的场景及结果以交接记录和每轮 JSON 为准，语法通过不代表运行验收。

2026-10-20 实际验证：closed和branch均经本入口完整通过。closed原生导航ABORTED、拆墙后同目标规划成功，12项检查通过；branch原生规划前后11项通过，实际下侧NavigateToPose SUCCEEDED、14项功能/安全检查通过，巡航0.707928m/s。两轮自动停止后组件会话为空；分别见mainline_20261020_closed_01与mainline_20261020_branch_01，不可使用已有case ID重复覆盖。精确驻足仅作信息，FAST-LIO2尚未切换生产输入。
