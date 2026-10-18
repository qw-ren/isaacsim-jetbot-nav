# 仿真运动与 Nav2 验证副本

仅云端仿真；不能用作真机速度桥。默认 DISARMED，不自动接受目标运动。
保留 baseline 与 work；`prepare_motion_copy.py` 按 motion_base.sha256 检查 stable 源码后
复制到 motion_work_20261015，并应用 motion_runtime.patch。拒绝覆盖已存在的副本。

独立副本关闭键盘及 /cmd_vel 旁路，Bridge 仅收 /unitree_go2/cmd_vel；限制前向
0–0.8m/s、无横移、角速±0.6rad/s，主循环独立检查墙钟0.5秒超时并清零。
/nav_m1/runtime_cmd 是控制张量目标反馈，不是物理实测速度；实测取 /unitree_go2/odom。

velocity_guard.py 是唯一底层速度发布者。输入 /nav_m1/cmd_vel，墙钟20Hz检查
输入/odom/scan/clock新鲜度、发布者冲突、姿态、已知AABB前瞻与前方scan。
异常锁停，数据恢复不会自动重新启用；通过 /nav_m1/enable 显式启用。
试验地图采用保守0.45m圆半径，保护阈值0.55m；不是全身碰撞认证或制动模型。

启动导航前先通过固定起点直行、刹停、输入断流和自建保护进程退出故障验收。
check_motion.py 会生成原始遥测和 motion_check.json；故障注入只接受
/proc/PID/cmdline 中包含 velocity_guard.py 的本轮进程，不操作机器电源。

nav_motion.yaml 用 Navfn + DWB + 最小行为树，/cmd_vel 显式重映射到
/nav_m1/cmd_vel，由保护节点转发。DWB 前向范围0.6–0.8m/s，不在输出端强行提速。
当前专用于直通道试验；禁止据此宣称任意转弯、狭窄空间、动态障碍已通过。
停止/故障输出零是正常行为。20Hz仿真控制频率应在当前RTF约0.25下维持输入新鲜度。

先运行现有scan与planning manager，然后 manage_navigation.sh start；
启动本身不会发送导航目标。check_navigation.py 发送单个 NavigateToPose，等待DWB输出
后显式启用保护，结束/异常取消任务并解除启用，记录至少5秒仿真停车数据。
它不发布任何速度，因此不会与控制器争用输入；默认目标(6,-1)。

当前直通道配置不加载 RotateToGoal critic：该评分要求进入接近区后继续减速，
与本试验前向最小0.6m/s冲突，会先于到点检查使全部轨迹无效。
仍用SimpleGoalChecker位置20cm/朝向0.5rad检查目标，并独立要求停车后位置误差≤25cm。
这是明确受限的直通道参数，不支持一般目标朝向旋转；不是通用仓储导航完成声明。
启动脚本先等待至少10条实时odom/clock及planner active，避免时钟初始化期间配置响应丢失。
