# C：静止起点封闭与完整任务结束

此项只验证四面实体墙封闭起点后的原生导航失败，以及拆墙后同目标恢复规划。不是运动中突然封路，也不是远端双通道全部封闭的通用验证。全程保护桥保持 DISARMED，不执行机器人行走，不调用 clear_costmap，不修改先验地图。

在云端用现有 Isaac Python 执行 `prepare_closed_route_copy.py`，默认从 `motion_live_branch_20261019` 创建 `motion_live_closed_20261019`；目标已存在时拒绝覆盖。只修改新副本的 fixture 导入，保留 live RTX、GPU 物理和原模型控制。

新的仿真 artifact 目录继续使用原 B 的 `box_config.json`，不改其中 divider/barrier。C fixture 保留先验 divider，但 B barrier 始终不存在。四墙几何写入 `enclosure_config.json`，实际 USD 状态写入 `enclosure_state.json` 和兼容的 `obstacle_state.json`，事件追加到 `enclosure_events.jsonl`。

为先完成正常先验地图定位，可在启动仿真前往 artifact 目录写入 `obstacle_request.json`：

```json
{"id":"initial_open","enclosure_enabled":false,"enabled":false}
```

fixture 初始化时建墙，首个主循环 tick 会执行这个请求。启动流程仍使用既有单例 manager，正常启动 scan、AMCL/planner、保持 DISARMED 的保护桥和完整导航。起点 odom 为 `(1,0)`；保护桥几何应包含保留的 divider 及四墙。观察器等定位和 BT active 后静止闭合四墙，等待正常 scan/costmap 更新，再发送完整 NavigateToPose。拆墙后仍等待自然清除，再对**冻结的同一 map 目标**运行 ComputePathToPose。

```bash
source /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_stage2/nav_env.sh
/home/admin/isaacsim/python.sh /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_stage2/check_closed_route_task.py \
  --output /home/admin/go2-warehouse/artifacts/NEW_C_CASE/task \
  --fixture /home/admin/go2-warehouse/artifacts/NEW_C_CASE/simulation \
  --map /ABSOLUTE/B_PRIOR/map.yaml
```

可选 `--map-image`（默认同目录 map.pgm）、`--bt-xml`（默认现有 navigate_to_pose.xml，无清图/恢复节点）、`--action-timeout`（默认 120 秒墙钟）。目标固定为 odom `(6.5,0)` 经当前 map→odom 转换得到，前后共用该同一目标。

关键结果是 NavigateToPose 原生 ABORTED（状态 6）、拆墙后 ComputePathToPose 原生 SUCCEEDED（状态 4），同时桥始终未解锁、目标输出为零、没有行走、地图不变。原始 telemetry、动作结果和 costmap/scan NPZ 先落盘，再算检查结果。小位移检查只核查未行走；无新增巡航、精度或统计门槛。本地语法检查不等于云端运行通过，实际证据应由独立新 case 产生。
