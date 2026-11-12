# 语义接近点与连续机动候选，2026-11-12

继最远到达末段失败和连续关系任务预测净距锁停，用户授权继续改进。
原冻结Nav2参数、定位后端、安全门源码/0.40m阈值/1.5s预测和旧语义配置保留。
本版为独立可选`warehouse_semantic_maneuver_20261112.json`，实际行为有额外原生限速，
不能因为参数字典相同就声称与全程0.8m/s行为完全一致。

## 已取得诊断证据

`semantic_rel_diag_20261112_05`使用code11，与前轮code10七个生产模块及配置逐字节相同，
只增加独立只读路径末端、局部地图与DWB评分采集；探针不在supervisor进程必活列表内。
最远任务再次native6/Failed to make progress；后续关系任务native4成功、gate无fault。
这说明此前的连续关系预测锁停不是每次必现，不能称单一因果已证明。

一个末段采样sim71.850：胜出速度[0,0,0.03158]，总分3.2；
有效正线速候选的已计分最低代价4.8，来自BaseObstacle的scale0.4×raw12，
其余critic被short-circuit提前截断。此时RotateToGoal=0，不支持“旋转阶段锁死”归因。
记录中的map全局/received路径末端仍为请求点(-13,22.74)，无目标偏移证据。
这里只解释该采样的选择，不能据此说所有失败均由某个critic造成。

前轮03首次门控拒绝的原始NPZ重新离线计算：sim92.690，在未来0.8s处最小净距0.376796m；
候选约0.8m/s、0.157895rad/s。用相差-0.08s的估计TF近似投影，最近扫描点约map(-12.652,23.789)，
落在刚访问远端纸箱边缘附近。不是机器人真值输入，不是实际碰撞证明，也不用于在线控制。

原生机制参考：[Nav2 ControllerServer的SpeedLimit订阅与回调](https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_controller/src/controller_server.cpp)、
[DWB打分实现](https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_dwb_controller/dwb_core/src/dwb_local_planner.cpp)。
本轮结论以实际运行的评分/配置/轨迹为主，未将上游分支当作已安装二进制精确版本证明。

## 候选行为

1. 语义接近额外余量0.60→0.80m，机器人中心到对象投影盒表面最低0.40+0.01+0.80=1.21m。
   规划半径、Nav2的1m膨胀和5cm到达容差均不改；该变化让指定停靠点远离障碍软代价边缘。
2. 原生`/speed_limit`的绝对线速上限，临近箱/托盘/推车/挡板时0.4m/s，其余最高原0.8m/s。
   DWB原生实现同时同比缩小角速度采样上限，临近阶段约0.3rad/s，清除限速后恢复原0.6rad/s。
   采用当前map←chassis估计TF与活跃对象盒表面距离，不读取机器人truth；不将整排货架作为限速触发对象。
   小于2m进入、达到2.5m退出，带滞回避免边界频繁切换；缺少活跃道具观测时保守限速。
3. 只在明确启用`speed_policy`的配置创建SpeedLimit发布器；发目标前检查原生订阅和唯一限速源。
   行驶中约每0.2墙钟秒刷新，记录SPEED_LIMIT及原因。另一限速发布器出现则任务失败并disarm，
   不覆盖别人的限速。固定旧配置无此发布器。
4. 恢复巡航以及确认disarm且action终态后，均发布原生0=NO_SPEED_LIMIT，仅撤销本轮限速；
   不清门控fault、不恢复任务、不切控制源。发布的是限速消息，没有Twist或TF发布器。

这两个变化组合验证，不声称分别完成单因素因果隔离；也不把减速后的成功当作0.8m/s全速通过。

## 当前验证

20项单元测试通过，新增限速滞回、对象缺失、固定货架不触发、非法上限/距离参数拒绝，
以及恢复原上限必须用0而非绝对0.8的原生接口规则。
域52使用真正mapper/manager与Nav2夹具的11项ROS检查通过，含临近限速和取消后disarm/清除限速、
行驶中发0恢复巡航、刻意不订阅ROS状态时的日志终态确认及重复ID不读取旧结果。
这不是PPO模型、真视觉或实体机器人验证。

首候选code12/case06最远箱native4成功。但进一步核对原生kinematics实现发现：
绝对限速≥base_max_speed_xy不会撤销已有低限速；因此首版从.4请求.8仍可能保持偏慢。
06不是最终限速恢复验证，主动停止并保留；关系任务在会话关停时出现SAFETY_CLEANUP_FAILED，
不计到达/功能通过。最终code13修复为发送0清除，新增行驶中临近道具退出后的ROS夹具检查。
参考[原生kinematics实现](https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_dwb_controller/dwb_plugins/src/kinematic_parameters.cpp)。

最终code13，case `semantic_maneuver_20261112_07`：同一会话先七项预览/异常，
再最远纸箱→第二排关系纸箱→第一排最近纸箱。实仿结果待完成，不提前宣称通过。

07实际未执行任务：定位地图读完1852×3791图后change_state响应发送超时，
生命周期卡在定位地图配置，manager报Nav2 not ready；正常停止、保留为启动异常。
08使用相同生产源码/配置补验，诊断探针延后READY后启动，不能据此称服务超时已根治。

08随后出现CLI确认丢失：管理器已写unsupported_relation=INVALID_COMMAND，客户端未收到终态，
顺序测试卡在预览阶段，尚无NAVIGATING。确认管理器终态后只终止该CLI并正常停止案例，
`CLI_ACK_LOSS.json`保留。code14新增按任务ID读取持久事件日志作为状态确认补充：
发命令前在日志末端建立游标，仅处理随后完整新行，ROS/日志按事件去重；二进制读取避免部分UTF8行误解析。
命令仍走ROS、导航仍走原生action。`--case-log-only`用于明确测试不依赖ROS状态订阅，
不会复用旧任务文件冒充新结果。CLI输出带`ack_transport`标记。

最终code14/case09：七项预览符合预期，最远native4成功，真实终点3.63cm/2.78°；随后关系任务ODOM_TIMEOUT失败并disarm，第三个实际任务未运行。当前无相关进程，缺完整运行器收尾报告。详见[JETBOT_SEMANTIC_PROGRESS_RELEASE_20261112.md](JETBOT_SEMANTIC_PROGRESS_RELEASE_20261112.md)。

后续复现：云端admin，先status，无其他实例后用唯一新ID：

```bash
CODE=/home/admin/jetbot-warehouse/semantic_20261112_14/code
bash "$CODE/manage_jetbot.sh" status
bash "$CODE/manage_jetbot.sh" start UNIQUE_NEW_ID 400 operator stationary \
  "$CODE/warehouse_semantic_maneuver_20261112.json"
# 在另一个admin终端按原说明加载ROS、ROS_DOMAIN_ID=51、PYTHONPATH=CASE/source。
/home/admin/isaacsim/python.sh "$CODE/semantic_tools/run_maneuver_acceptance.py" \
  --case "$CASE" --runtime "$CODE" --extra-nearest
```
