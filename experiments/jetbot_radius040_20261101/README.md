# 0.40m冻结前诊断与失败证据

本目录保留原障碍权重0.20和首步指令发布试验；当前研发基线使用规划半径0.40、BaseObstacle.scale=0.40，见[本版参数](../../docs/JETBOT_NAV_PARAMETERS_20261101.md)。不能把不同配置混成一组成功率。

02 B/A成功、03 C预期无路停车、05长路线成功、08带评分诊断成功；04 scan超时、06预测间距锁停、09首步发布候选失败均保留。01错误数值类型、07调试参数缺声明的启动失败原文在对应原始归档，不是导航算法结果。

REJECTION09_ANALYSIS记录被拒绝的确切指令/scan点/预测轨迹及门限；最接近DWB评分不是精确同一指令，不能据此指认critic。MODEL_COMPARISON08量化成功例两种模型的轨迹差异，不是06历史根因证明。GEOMETRY只作离线AABB近似，truth不进入在线控制。

四份原始归档与逐摘要核验见[ARCHIVES_VERIFIED](../jetbot_soft040_20261101/ARCHIVES_VERIFIED.json)，下载见本版Release。
