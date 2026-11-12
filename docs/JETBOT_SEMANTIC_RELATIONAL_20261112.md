# 最远目标与货架空间关系指令，2026-11-12

在原对象语义闭环上扩展规则解析、可达对象排序和跨物体关系筛选。
物体观测仍来自USD，机器人仍使用原生IMU+FAST-LIVO2+AMCL；不改冻结Nav2/安全门参数。

## 指令与确定含义

|指令示例|处理规则|
|---|---|
|找最远的纸箱／到最远的纸箱那里|为每个纸箱找最短安全可达接近路径，再选择其中路径最长的纸箱|
|走到第二排货架旁边的那个纸箱|先找到编号2的货架排，再筛选与该排投影包络表面距离≤1.5m的纸箱；必须唯一|
|到第一排货架旁边最近的纸箱|上述关系筛选后，选择最短可达路径|
|到第一排货架附近最远的纸箱|上述关系筛选后，选择最远可达对象|
|到第九排货架旁边的那个纸箱|不存在该排时REFERENCE_NOT_FOUND，无导航输出|
|到第二排货架左边的纸箱|当前不支持左右关系，INVALID_COMMAND，无猜测执行|

“找”在本接口表示导航到物体旁边，不表示主动探索未知区域。全部登记对象可见于oracle，
不是模拟视觉视野。最远/最近是原生规划器返回的采样接近点路径比较，不宣称连续空间最优。
同一物体不会因其背侧接近点绕路长而被误选为“最远”。不可达物体不参与排序。

不带最近/最远的“那个”若对应多个匹配物体，返回AMBIGUOUS，附候选ID、关系距离及澄清提示，
操作者可补充最近/最远后发新指令。若相同排号对应多个独立参照对象，返回AMBIGUOUS_REFERENCE；
未找到箱子返回NOT_FOUND，无安全可达点返回UNREACHABLE。不会静默忽略排号退化为全场最近。
参照货架和目标本体在规划/行驶期间均检查过期、移动、旋转、尺寸及排号变化；失效先disarm再取消原生action。
本版仍为有限规则，不支持任意语言、多步动作、指代历史、左右/前后/中间等关系。

## 排号与几何登记

`warehouse_semantic_relational_20261112.json`是独立测试配置，原演示配置保留。
四排既有货架按本场地目录从西往东（map X递增）编号1–4，约位于X=-15.41/-10.46/-5.50/-0.55m。
编号来自显式场地登记，不随机器人位置或视角改变，也不推断为实际仓库已有标牌。
每排聚合7个既有货架Prim，以运行时可见几何的世界AABB并集给出一条shelf观测，
`object_id=/SemanticCatalog/shelf_row_N`、`row_index=N`。目录只登记本演示的四排，不涵盖整个仓库。
分组由`make_relational_demo_config.py`从原先验collision_boxes中对应的精确Prim路径生成，
仿真内重新读取真实USD包络；未创建新货架或修改先验地图。

为构造唯一关系案例，临时新增原生纸箱`box_relation`，中心(-8.4,12,0)m；
`box_near`置于(-13.9,12,0)、`box_far`仍为(-13,24,0)。
最终对照布局只保留原近端推车和这三个纸箱，暂不放置两托盘及远端推车。
新增纸箱到静态CAD包围盒最小间距约1.173m，未与固定环境重叠。
这不是最终团队场景或资产合同。1.5m是“旁边”的关系筛选阈值，
与0.4m规划半径、1m代价膨胀、1.01m接近目标中心距分别独立。
关系距离使用两物体保守XY包络之间的最短表面距离，支持旋转/倾斜投影；
不使用中心距离，也不对机器人真值做匹配。

可替换视觉输入可携带可选`row_index`；视觉/语义模块负责与场地目录关联，
仅检测出类别shelf而没有可靠排号时不猜编号。schema保持v1的可选字段扩展。

## 验证与复现

16项单元测试通过，包含长绕路接近点不改变最远对象、关系唯一/歧义/缺参照、
相交长矩形无内含角点、中文排号及严格拒绝不支持组合。
域52使用真实mapper/manager和确定性Nav2夹具的9项检查通过，含参照移动取消、
歧义/缺参照不发导航；没有连接仿真或发布Twist。

最终对照实仿case `semantic_rel_20261112_03`，独立代码 `semantic_20261112_10/code`。
03的七项预览/异常查询全部符合预期。实际farthest_nav选中box_far，
原生导航末段返回native6/Failed to make progress，估计终点距请求点约6.7cm，
gate无fault，已disarm；这不是到达成功，也不凭此归因某个critic。
随后独立提交relational_nav，正确选中box_relation，但在续航时触发
PREDICTED_SCAN_CLEARANCE，任务FAILED并disarm，03已正常停止。
04使用完全相同代码/配置，从声明出生点单独执行关系目标：
`走到第二排货架旁边的那个纸箱`正确选中box_relation并实际SUCCEEDED/native4，gate无fault，disarm确认。
没有放松原5cm到达容差或安全门，也不自动清故障/重试掩盖03。

03关系任务第一次锁停记录：sim92.690，原始/裁剪命令约[0.8m/s,0.158rad/s]，
1.5s预测扫描净距0.3768m低于0.40m门限，立即零速并锁停；这不是实际碰撞证明。
04从声明出生点单独成功并不能证明03由前一失败终点造成，也不能证明任意连续关系任务已稳定。

|独立实仿|语义选择/查询|实际运动验收|
|---|---|---|
|01，code08|只有近箱路径通过，远端路径排除|仅预览，无运动|
|02，code09|两个可达箱中正确选较远者，远箱因未见推车排除|仅预览，无运动|
|03，code10|七项预览/异常通过，最远箱与关系箱均选对|最远末段native6；后续关系任务预测净距锁停，均失败|
|04，同code10/同配置|从声明出生点正确选关系箱|关系导航native4，成功，gate无fault|

结论：已验证有限关系查询与一条实际关系导航；最远目标只完成正确选取和实际行驶尝试，
未通过远端到达验收。完整整组测试不算通过，不作为替换/重新冻结底层参数的证据。

04成功任务真实终点XY误差6.37cm、yaw误差2.89°，机器人中心到对象AABB表面1.068m。
使用完成时刻最近truth采样离线评分，无时间平移/轨迹拟合；不是AMCL全时刻精度。
四轮均主动manage stop，runner130/error=null；子进程含SIGINT收尾非零，不称全exit0。
03保留预测净距fault，其余无fault，全部已停止，没有操作电源/用户SSH Master。

正式云端精选交付 `/home/admin/jetbot-warehouse/semantic_rel_delivery_20261112_01`，
本地 `data/cloud_warehouse/jetbot_semantic_relational_20261112/delivery_01`。
精选仅包含根目录元数据/日志、source、map及semantic_tasks，原始传感器与完整观察帧留在云端。
`JetBot_Second_Row_Box_4x.mp4`为实际04关系任务，1280×720/25fps/8.20秒，
204张观察帧按仿真时间4倍速输出，已完整解码检查；文字仅离线叠观察画面。
视频SHA256 `f29369ccc7946893c0ae5502fc00c83d46101d3c996015f2e7ca8cdd4c7f637a`。
四个归档共579内部文件逐SHA核验，源码69可直接比较文件与本地一致，七个底层源文件与冻结基线
逐字节一致；parameter_overrides字典一致。域52验证的09模块与03/04的七个生产模块逐SHA相同。
`QUERY_AUDIT.json`独立从approach_audit重建每对象最短路径，确认排序/选择一致；
`full_navigation_suite_passed=false`明确保留运动失败。
小型摘要、核验与地图在`experiments/jetbot_semantic_relational_20261112`，录像/归档不入Git。
代码仍在原独立开发分支，未commit/push或合并main。

03已完成的查询预览：最远箱box_far最短路径27.671m，其他box_near16.901m、box_relation11.442m；
第二排关系筛选的表面距离分别为box_near2.503m、box_far1.603m、box_relation1.123m，
以1.5m阈值唯一选中box_relation。第一排附近两个纸箱则分别为0.589/1.489m，
无选择词时返回歧义，加最近/最远后分别选择box_near/box_far。
这些值来自各条独立查询时的Nav2路径和对象地图，不把它们当作执行轨迹长度。

此前两轮均只有预览、没有运动任务，证据保留：01使用原布局加新箱，有效候选仅box_near；
02移除托盘并调整两箱位置后，有效候选box_near/box_relation，其最短路径17.154/11.408m，
“最远可达”正确选择box_near。更远box_far路径首个不安全点约(-12.15,19.033)，
global cost=0，却距已登记cart_far表面小于规划包络，被独立对象几何检查排除。
这不是物理不可达证明，而是当前原生规划器返回的路径不安全，不能用oracle跳过保护。
顺序脚本预设box_far因此报预期不符，未进入实际运动；03移除该测试推车作清晰对照。

后续复现须先status，无会话后换新ID：

```bash
CODE=/home/admin/jetbot-warehouse/semantic_20261112_10/code
bash "$CODE/manage_jetbot.sh" status
bash "$CODE/manage_jetbot.sh" start UNIQUE_NEW_ID 400 operator stationary \
  "$CODE/warehouse_semantic_relational_20261112.json"
# 另一个云端admin终端，按原说明加载ROS环境，ROS_DOMAIN_ID=51，PYTHONPATH=CASE/source。
/home/admin/isaacsim/python.sh -m jetbot_semantic_nav.cli --case "$CASE" '找最远的纸箱'
/home/admin/isaacsim/python.sh -m jetbot_semantic_nav.cli --case "$CASE" '走到第二排货架旁边的那个纸箱'
```

任务JSON新增`relation_audit`、`object_ranking`，可核对实际参照、关系距离及每个可达对象的最短路径；
其余原始任务、路径、门控、进程和离线truth评分沿用原格式。
