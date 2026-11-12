# 对象语义导航逐轮验收，2026-11-12

本报告记录最初01–06案例及当时源码快照；后续最远/货架关系扩展的源码与新增案例见
[关系指令验收](JETBOT_SEMANTIC_RELATIONAL_20261112.md)，不要把此处06的源码核验理解为后续扩展也用旧源码测试。

全部为独立 Isaac Sim4.5 仓储仿真；固定三倍JetBot，原生IMU、FAST-LIVO2、AMCL、
Nav2/DWB及原安全门。开发基于PR6的0d957b9，原main及其他同学工作树未改。
这里的Ground Truth只提供物体观测，机器人定位不订阅/拟合真值。

## 试验配置与结果

场景新增 NVIDIA 原生推车/托盘/纸箱各2，类别与资产独立登记。
位置分别为cart(-3,7)、(-13,20)，pallet(-8,12)、(-8,20)，box(-13,12)、(-13,24)。
物体原生碰撞保留，不写入原规划/定位先验；固定环境原样。
初选东侧cart位置在离线检查中与既有柱体重叠，未用于运行，已换至无重叠位置。

|case|主要配置/变化|实际结果|
|---|---|---|
|semantic_20261112_01|物体表面到中心最低.66m，直接Nav2|推车native4；缺失/非法/取消通过；续航托盘native6，Failed to make progress，gate无fault|
|semantic_20261112_02|增加固定当前XY的出发前朝向目标，仍.66m|对齐native4，推车主任务native6/进度失败，停车在目标前约25cm，gate无fault|
|semantic_20261112_03|中心距表面1.01m、目标cost≤20，仍附加对齐|启动早期sim4.61 ODOM_TIMEOUT锁停，ODOM_FAULT记Estimator timestamp discontinuity；不算参数验证通过|
|semantic_20261112_04|上述配置加定位预热/健康等待|推车与托盘native4，取消通过；纸箱附加对齐出现XY修正循环，主动取消/native5；gate无fault|
|semantic_20261112_05|撤掉附加对齐，保留1.01m接近距离和低代价目标；关闭600wall秒只读评分探针|推车、托盘、纸箱依次native4，缺失/非法/取消均通过；gate无fault|
|semantic_20261112_06|在05上修复对象时间取整/消息先后造成误删|最终源码复测：推车/托盘/纸箱native4；缺失/非法/取消通过；gate无fault|

这些是开发过程，不是六次同配置成功率统计。保留失败、主动取消、启动异常，
不能据此归因某个DWB critic；没有放松保护阈值或修改冻结Nav2参数。
最终流程直接提交选定接近目标给Nav2，已撤回同XY附加对齐。

## 05及最终06功能通过记录

同一会话内顺序执行以下真实中文任务：

|任务|期望|结果|
|---|---|---|
|到最近的推车那里|选择同类近且可达实例并到达|SUCCEEDED/native4|
|到最近的标牌那里|当前未登记标牌，明确无目标|NOT_FOUND，禁用输出|
|到最近的人那里|当前不支持人作为目标|INVALID_COMMAND|
|托盘任务行进中取消|取消原生action并停车|CANCELLED/native5确认，disarm|
|到最近的托盘那里|取消后可接续新任务|SUCCEEDED/native4|
|到最近的纸箱那里|连续任务到达|SUCCEEDED/native4|

六个稳定ID观测实例持续更新；多候选规划及排除原因写入各任务的approach_audit。
“最近”是所有通过采样接近点检查的原生路径中长度最短者，不宣称连续空间全局最优。

05的离线真值误差只对比机器人chassis与请求的自由空间接近点，不与对象中心比较：

|目标|真实终点XY误差|真实终点yaw误差|中心到对象AABB表面|
|---|---|---|---|
|cart_near|3.05cm|2.74°|1.013m|
|pallet_near|6.46cm|2.85°|1.061m|
|box_near|6.47cm|2.41°|1.075m|

没有时间平移、全轨迹拟合或用真值在线校正。这个功能验收不新增“真实3cm/1–2°已达标”结论。
case05运行475.44墙钟秒，完成后主动manage stop，runner130/error=null；
SIGINT收尾子进程包含-2/254，不能称全部exit0。最终安全门故障为空、已发布关闭零速。
云电脑电源与用户SSH Master未操作。

最终06使用 `semantic_20261112_07/code`，同一会话重新完成上表六项，全部符合期望。
取消请求确认原生CANCELLED/native5；任务摘要中native_status空值不代表没有取消确认，详见任务事件。
运行442.58墙钟秒，主动停止后runner130/error=null，15个子进程含0/-2/254，
安全门无fault、全部任务disarm、关闭零速确认；无遗留run_sim/supervise进程。

|最终06目标|真实终点XY误差|真实终点yaw误差|中心到对象AABB表面|
|---|---|---|---|
|cart_near|3.16cm|2.86°|0.986m|
|pallet_near|7.90cm|2.83°|1.087m|
|box_near|7.66cm|2.78°|1.081m|

数值是实际完成时刻离线truth样本与请求接近位姿的差，不是AMCL全时刻全局定位误差。
最终06与05的差异也不能解释为定位精度改善/退化的统计结论。

## 可复核交付

本地根目录：`/home/liswnux/isaac-go2-ros2/data/cloud_warehouse/jetbot_semantic_20261112`。
最终精选归档在 `final_06`，解压证据在 `cases_06/semantic_20261112_06`；历史01–05在
`selected_01`/`cases_01`，失败记录保留。六个归档共904个内部文件已逐个核对SHA256，
最终归档SHA256为 `f06b0c9f2b0db06241a3634256344cd500f207283e0464db0d906bfc2a8860c9`。
69个可直接比较的部署源码与本地一致；有效nav2.yaml/行为树由原启动器生成，
另行确认七个底层源文件与基线逐字节一致、parameter_overrides字典完全相同。

最终视频 `final_06/JetBot_Semantic_Final_Cart_4x.mp4` 为1280×720、25fps、6.56秒，
163张实际观察帧按仿真时间4倍速，已完整解码检查并查看画面。
视频SHA256 `ca80953ef6d7a92aa52d2749c8b8d62eb3429a1bdf1c2e1f9d501647e6530c19`。
物体真值来源、实际中文命令及任务状态仅离线叠加在观察画面。
`Semantic_Map_Final.png` 展示原地图、独立对象层和离线真实轨迹。
仓库内 `experiments/jetbot_semantic_20261112` 仅保留小型结果/核验/图示，不提交录像与原始归档。

正式云端交付为 `/home/admin/jetbot-warehouse/semantic_delivery_20261112_03`。
`_02`是停止完成前产生的草稿目录，不作为最终runner/gate证据。

本轮文件变化：新增独立 `ros2_ws/src/jetbot_semantic_nav` 包（模块、schema、launch、
单元测试与离线工具），新增语义实验配置、两份说明和小型验收结果；
修改manage/run_sim/supervise接入可选语义模块，prepare_fixture_prior仅扩展递归源码摘要；
更新两级README。未改其他同学视觉/PPO实现。

## 代码与隔离接口验证

12个本地单元测试覆盖中文合法/否定/歧义、对象更新/过期/重复ID/非法数、
非单位坐标注册、旋转与倾斜盒平面投影、路径中间穿障、旋转代价栅格、
仿真暂停时的墙钟过期，以及亚纳秒未来时间戳不误删对象。

域52隔离ROS夹具运行真正的mapper/manager，对接确定性Nav2 action/service夹具，
没有仿真连接、没有Twist发布器。检查全候选阻塞不发导航/不使能、目标移动/过期
取消、用户取消、重复ID不重放、vision来源的带时间非单位map←optical变换。
这不是Isaac物理动态目标或真实视觉模型验收。

新健康预热后，隔离夹具暴露入口允许≤.1s时钟消息偏差但active查询严格拒绝未来数的
不一致：浮点取整即可能误删新观测。已把活跃/过期检查统一为入口相同的.1s偏差窗口，
过期仍同时检查5s仿真年龄/接收墙钟年龄；未改安全门时钟判据。

## 交付范围与限制

默认测试资产不是团队最终资产/场景合同；没有随机种子生成器、视觉模型或PPO训练。
语义类别支持货架/纸箱/托盘/标牌/推车，当前真实闭环验证为推车/纸箱/托盘；
固定货架可通过registered_prims登记，未把整仓货架/标牌自动完整标注。
推车当前是静止目标；目标变化处理的额外场景在隔离ROS夹具验证，不是动态追踪。
视觉适配器负责稳定ID、米制3D估计与置信筛选；本版不做跨源关联/检测模型置信标定。

模型输入接口不依赖oracle Prim命名；后续停止oracle，用vision来源观测替换。
未知TF帧或测量时刻TF不可用时拒收，不使用latest掩盖对齐问题。
高处对象的平面投影保守处理，未做悬空标牌几何与地面障碍的高度分层优化。
不改变冻结DWB对任意狭窄组合的已知能力边界，也未验证深度+录像并行高负载。

源码/命令见[JETBOT_SEMANTIC_ORACLE_20261112.md](JETBOT_SEMANTIC_ORACLE_20261112.md)。
原始精选、SHA256及视频保存在原工程data/cloud_warehouse/jetbot_semantic_20261112；
原始传感器NPZ与完整观察JPG保留云端case，精选归档范围有清单明确说明。
当前开发不自动推送GitHub或合并main。
