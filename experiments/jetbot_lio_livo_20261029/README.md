# JetBot LIO / LIVO 同输入精度比较，2026-10-29

PPT可直接使用PNG/PDF图、accuracy_summary.csv表和trajectory_and_error.csv曲线数据。COMPARISON.json为完整评分口径；COMPARISON_SERIES.json保存次要初始航向归零轨迹，CSV/图已还原主声明航向口径，不能把两个口径混用。

3倍JetBot，43.660m现有货架路线，原生IMU无额外噪声。同一份点云/IMU/RGB记录分三组离线重放：FAST-LIO2 11.23cm XY RMSE、FAST-LIVO2 1.49cm、LIVO关闭图像3.98cm。视觉消融不冒充FAST-LIO2；不能将全部后端差异归因于视觉。

共同窗口7.0–68.83秒，20Hz评分网格；只按起步前静止IMU/首估计姿态和声明CAD起点对齐，没有全轨迹拟合，真值只离线评分。首轮1x播放LIO尾段缺5秒，主结果为相同输入0.5x重放复核（不是在线实时性能基准）。三组输入摘要一致。单个采集轨迹，不是统计重复或真机保证。

相机输入没有让所有航向指标都变好。完整复现输入数据库708489216B，SHA256 ed8fa6c1cb18d5582cce85500e21b87de982551c0533b0bb57d75209a06ea65e，受控本地/云端保存，不放Git；Release精选数据包包含三组原始重放输出/参数与绘图源。图表单位和CSV字段单位明确标注。
