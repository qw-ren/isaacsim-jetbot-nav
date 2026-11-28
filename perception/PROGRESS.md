# 视觉感知模块 · 进度与交接

> 最后更新：2026-11-12 ｜ 负责模块：**视觉目标识别 + 视觉/雷达信息处理**

## 一句话状态

核心库在本机全链路跑通（检测 → 跟踪 → 定位 → 融合 → 实时预览/录制），39 个单元测试全过；
8 类障碍物模型已用 Replicator 合成数据训练并部署（`weights/obstacles_best.pt`，val mAP50 0.958）；
**卡点只剩一件事：ROS2 节点还没在装了 Isaac Sim 的云电脑上实机联调过**。

## 模块职责

把「像素 / 点云」变成「物体类别 + 世界坐标」，经 ROS2 交给语义导航和 RL 局部避障。
感知只回答"看到了什么、在哪里"，**不决定去哪**。

## 技术方案

**检测**：Ultralytics YOLO26s（NMS-free），类别白名单按**类别名**解析（不按 id），换权重不用改代码。

**分层架构**：

| 层 | 文件 | 输入 → 输出 |
| --- | --- | --- |
| L1 语义理解 | `atongmu_perception/detector.py` | BGR 图像 → `Detection2D`（类别、置信度、像素框） |
| L1.5 跟踪 | `tracker.py` | 逐帧检测框 → 跨帧稳定 `track_id`（IoU + 匈牙利） |
| L2 几何定位 | `localizer.py` + `geometry.py` | 框 + 深度图 → 相机系/基座系 3D 点（无深度时退回地面约束测距） |
| L3 融合 | `fusion.py` | 单线激光投影进图像，±35% 深度窗内关联，0.35/0.65 加权 |
| L4 接口 | `types.py` + `pipeline.py` | 统一数据结构，JSON over `std_msgs/String` 发布 |

关键约定：深度取框内 25% 收缩区域的 **20% 分位**（取近表面，避免背景渗入）；
坐标系 base/world 为 x 前 y 左 z 上，相机光学系为 x 右 y 下 z 前；
`T_camera_laser = inv(T_base_camera) @ T_base_lidar`。

## 已完成

| 层 | 内容 | 验证方式 |
| --- | --- | --- |
| L1 检测 | YOLO26s，类别白名单可配 | bus.jpg 4 人 / 合成视频 122 次命中 |
| L1.5 跟踪 | IoU + 匈牙利匹配，类别门控、速度预测、`max_age` 过期 | 13 个单元测试：ID 稳定/顺序无关/跨类别不串/过期不复用 |
| L2 定位 | RGB-D 反投影 + 分位深度采样；无深度时地面约束兜底 | 合成场景误差 0.5–1.0 cm |
| L3 融合 | 激光-相机数据关联 + 加权融合 | 融合后 0.5–2.2 cm |
| L4 接口 | `PerceptionResult` / `ObjectPose3D` + ROS2 节点 | 本机语法检查，**未实机** |
| 实时预览 | 原生 OpenCV 窗口（HUD）、浏览器监控页（MJPEG + 置信度表格）、mp4 录制 | 无头 Edge 截图 + 帧数/统计接口校验 |
| 8 类障碍模型 | Replicator 生成 3000 帧 → YOLO26s 80 epoch → `weights/obstacles_best.pt` | 云电脑 val 450 张：P 0.935 / R 0.901 / mAP50 0.958 / mAP50-95 0.864 |

定位精度（10×6 m 合成房间，相机高 0.35 m，深度噪声 10 mm、激光噪声 20 mm，2% 深度丢点）：

- 俯仰 0°：person 0.9 / chair 0.9 / bottle 1.0 cm；融合后 0.6 / 0.5 / 2.2 cm
- 俯仰 12°：整体升到 3–5 cm（斜表面深度偏置，见 TODO）

## 命令速查（在 `perception/` 下）

```bash
# 检测
python scripts/demo_detect.py                              # 样例图
python scripts/demo_detect.py --source 0 --verbose         # 摄像头 + 逐帧置信度
python scripts/demo_detect.py --probe-cameras              # 摄像头体检（0-4 号 × 三种后端）

# 浏览器监控页（同时录制到 runs/preview_<时间戳>/annotated.mp4）
python scripts/live_preview.py --source 0
python scripts/live_preview.py --source 0 --host 0.0.0.0   # 云电脑，本机浏览器远程看

# 定位精度验证
python scripts/demo_locate_3d.py --pitch-deg 12

# 测试（不依赖 pytest）
python tests/run_all.py -v
```

参数要点：
- `--video-fps` / `--record-fps` 按实际处理速度设（本机约 8），否则回放会快进。
- 结束请用 **Ctrl+C**，强杀进程可能让 mp4 只写一半。
- `--backend` 可选 `auto/dshow/msmf`，默认 auto 自动兜底。

## 本机环境记录（踩过的坑，别再踩）

1. **只有一颗摄像头** `ACER HD User Facing`，**编号 0 可用（640×480）**，1–4 号打不开是正常的。
2. **`opencv-python-headless` 会覆盖 `opencv-python`**，导致 `GUI: NONE`、`imshow` 抛
   `The function is not implemented`，摄像头只录到 1 帧就崩。当前为 `opencv-python 4.13.0.92`
   （`GUI = WIN32UI`）。恢复方式：
   ```bash
   pip uninstall -y opencv-python-headless opencv-python
   pip install opencv-python==4.13.0.92
   ```
3. **中文计算机名**会让 `ThreadingHTTPServer` 启动时 `socket.getfqdn()` 抛 `UnicodeDecodeError`，
   必须覆写 `server_bind()` 跳过反向解析（`scripts/live_preview.py` 已处理）。
4. **torch 2.4.1 是 CPU 版**（RTX 4060 但无 CUDA），推理约 130–150 ms/帧 → 实际 7–8 FPS。

环境：Python 3.8.8 / ultralytics 8.4.82 / numpy 1.24.4 / Windows 11。

## TODO（优先级从高到低）

- [x] **目标跟踪**：IoU + 匈牙利匹配，给 `object_poses` 稳定 `track_id`
      （`tracker.py`，13 个测试；新目标 `min_hits` 帧后确认，丢失 `max_age` 帧后 ID 不复用）
- [ ] 与场景组对齐：话题名、**相机光学系 frame**、雷达 frame、目标类别集合
      （当前 9 类 COCO 白名单：person / chair / dining table / couch / bottle / cup / backpack / potted plant / laptop）
- [ ] ROS2 节点上云电脑实机联调：深度编码 `32FC1`/`16UC1`、TF 方向、QoS
      （先跑 `python3 ros2_ws/tools/perception_probe.py` 探明）
- [ ] 倾斜相机深度偏置：用框底中心点或激光点拟合面替代中心点采样
- [ ] 迁移到自定义 `atongmu_msgs` 消息类型
- [ ] 单线激光对矮小目标（瓶子 2.2 cm）收益有限，考虑多线或时序滤波
- [ ] **11.27 视觉链路**：用 Replicator 合成数据训练障碍物模型，输出「类别 + base_link/map 坐标 + track_id」给语义地图
      （脚本已写好：`isaac_sim/replicator/`，**尚未在云电脑 Isaac Sim 4.5 实测**）

## 11.27 里程碑对接（2026-11-12）

- 视觉验收：靠近障碍时输出**类别 + 机器人坐标系位置**，下游按 `track_id` 做多帧融合构建语义地图；
  不要求高精度，先跑通「Replicator 生成 → YOLO 训练 → ROS2 推理」。
- 待与航硕确认：**具体识别哪些障碍资产 / 类别命名**（当前 `dataset_config.yaml` 用的是
  box/cylinder/cone 占位几何体）。
- PPO 局部避障的底座（Isaac Lab vs 自建 Gym）与随机化范围也待与航硕商定，确认前不动 `rl/` 代码。

## 文件结构

```text
perception/
├── atongmu_perception/     核心库（无 ROS 依赖）
│   ├── config.py           配置数据类
│   ├── detector.py         L1 YOLO26 封装
│   ├── tracker.py          L1.5 IoU + 匈牙利跨帧跟踪
│   ├── geometry.py         针孔模型 / 变换 / 深度采样 / 地面约束
│   ├── localizer.py        L2 定位
│   ├── fusion.py           L3 激光-相机关联
│   ├── pipeline.py         主流程
│   ├── types.py            数据结构（接口契约）
│   ├── camera.py           摄像头打开/体检、GUI 可用性探测
│   └── viz.py              可视化
├── configs/perception.yaml 参数
├── scripts/                demo_detect / live_preview / demo_locate_3d
├── tests/                  39 个单元测试
└── weights/                模型权重（不进仓库）
```

接口契约见 [`../docs/perception_interface.md`](../docs/perception_interface.md)。

自定义障碍物模型的两个入口：

- 合成数据：`../isaac_sim/replicator/gen_dataset.py`（配置 `configs/dataset_config.yaml`）
- 训练：`scripts/train_yolo.py --data ../datasets/atongmu_obstacles/data.yaml`
