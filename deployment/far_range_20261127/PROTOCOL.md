# 2026-11-27 货架通道远距离观测实验

case：`far_range_20261127_01`。使用已有 Isaac Sim 4.5、3倍 JetBot、
FAST-LIVO2/AMCL/Nav2 地图与保护门，最高前进速度0.4m/s。
视觉源码在启动前备份到服务器实验目录 `vision_before.tar.gz`。

路线取自已有 north warehouse rack aisle 路线：
`(0,6) -> (0,7) -> (-8,7) -> (-8,10) -> (-8,14) -> (-8,18) -> (-8,20)`。
每次原生 NavigateToPose 成功后停留4秒仿真时间，采集静止观测。
地图只使用固定先验和声明的静态夹具；不把视觉推测或真值输入控制链。

已知目标为真实仓库 USD CardBoxA，中心 `(-8,23,0.25)` 米，
包络尺寸约 `0.7 x 0.5 x 0.5` 米。它在启动前加入场景与规划先验，
离终点约3米，不强迫机器人接触或穿过目标。
目标的实际 USD 摘要、世界包络和碰撞体记录在 `STATIC_FIXTURES.json`。

模型 SHA256 为
`367994e796f76c875eebccb018141e71af8c04b1dcbe25a26f8d428abf134a3d`。
原始算法相机实际320x240，内参由CameraInfo记录，不能用离线640x480默认值。
模型版本与类别顺序逐条保存在 observations.jsonl 的 model 字段。

## 采集

- `observer_video/navigation_1x.mp4`：跟随观察相机，按仿真时间编码。
- `range_evaluation/detections_1x.mp4`：算法相机带检测框，每秒仿真时间采样。
  该录像是稀疏采样回放，不能冒充算法相机原生8FPS连续视频。
- `route_evidence/trajectory.jsonl`：采集时间TF给出的map系估计轨迹。
- `truth_trajectory.jsonl`：独立仿真真值，仅作离线对照。
- `range_evaluation/trajectory_map.csv`、`trajectory_comparison.png`：时间戳轨迹表和无拟合对齐的真值比较。
- `route_evidence/observations.jsonl`：完整二维观测，保留session/track ID、采集时间、模型身份。
- `frames.jsonl`、`rgb_*.png`、`depth_*.npy`：匹配帧、原始RGB-D、相机K及base到光学系外参。
- `VIDEO_SHA256.json`：视频校验值。

## 评估边界

距离分组为0–3、3–5、5–8、8–12、12米以上，使用目标包络中心到机器人真值
的水平距离；与观测中的表面点 range_m 定义不同。
用独立真值姿态和固定光学外参投影箱体CAD包络，深度一致性仅作为可见性代理。
检测框与投影包络IoU>=0.25时记录类别匹配，不能将该数字称为人工标注召回率。

目标XY到包络外侧的距离只反映明显偏离，零距离不代表精确定位正确，
也不能把表面观测与物体中心之差直接当误差。
该实验可量化机器人map轨迹对真值的误差，不能独立证明每个视觉表面点的真实误差。
八类都会照常输出，但固定已知目标只有box，不宣称完成八类别远距离精度验收。

服务器运行入口：`run_experiment.py`，后处理：`finalize_experiment.py`。
任务失败、保护门故障或超时会保留失败结果并停止；不放宽停车或安全阈值。

## 本次实际结果与复跑修正

本次case采集137张匹配RGB-D图片、1145条map观测、264个轨迹点；行驶约23.57米，
观察录像146.92秒。前5个导航任务通过，最后一个native_status=6，整个导航验收未通过。
controller_probe 的600墙钟秒上限与本次慢速渲染实验不匹配，正常退出后
supervise.py将扩展进程退出视为会话故障。关闭时保护门已发布零速，未放宽保护条件。

本目录的run_experiment.py后续已修正为关闭可选controller_probe分数采集器，
这不改变控制器或保护门；该修正未用于已经结束的01号case，不能当作已复跑通过。
原始CASE.json和服务器case/source保留实际运行版本。复跑需使用新的case ID。

已知box的70个投影/深度可见性代理样本中没有匹配box检测，不能报告目标定位精度。
机器人map估计轨迹对真值XY RMSE约0.01568米，P95约0.02579米；该指标只描述本次
轨迹，不代表视觉物体定位误差。机器可读结果见 experiments/far_range_20261127_01/RESULTS.json。
