# A/B连续演示68

Nav2原生状态4，安全门无故障，45.56m实际路线，终点真实误差11.63cm。B先封当前货架通道，A后出现在相邻绕行通道；两种均运行中生成，使用NVIDIA原有USD碰撞资产。SHOWCASE_AUDIT.json检查生成时运动、此前全局路径与障碍相交、更新后绕开、实际穿越点和近似净距；不是用指令宣称换道。完整路径/轨迹及失败66/67在Release的showcase_selected.tar.gz。

VIDEO_METADATA.json给出四倍速时间轴、事件时刻、视频SHA与展示代码SHA。橙色箭头为局部计划的展示平移，不是算法观测；红信标为声明目标。VIDEO_CONTACT_SHEET.jpg为最终视频抽帧。复核脚本isaac_sim/jetbot_fastlivo/audit_showcase.py，注意其输出目录保留全轨迹，需要放到Git之外。
