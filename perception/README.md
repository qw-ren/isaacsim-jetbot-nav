# 视觉感知模块（perception）

> 2026-11-20：当前按**二维平面**方案推进：图像检测框 + 类别/track_id/平面 XY，
> JetBot 专用入口为 `ros2 launch atongmu_perception_ros jetbot_planar.launch.py`。
> 新接口 `/perception/objects_2d` 与服务器验收见
> [二维对接说明](../docs/planar_perception_handoff.md)。以下三维定位描述保留为历史几何实现说明；
> 当前 ROS 平面流程不估计三维框或尺寸，旧语义导航三维盒协议需要下游另作二维适配。

阿童木课题 · **视觉目标识别 + 视觉/雷达信息处理**。

职责一句话：把「像素 / 点云」变成「物体类别 + 世界坐标」，经 ROS2 话题交给语义导航和 RL 局部避障。
感知只回答“看到了什么、在哪里”，**不决定去哪**。

## 分层结构

| 层 | 文件 | 输入 → 输出 |
| --- | --- | --- |
| L1 语义理解 | `detector.py` | BGR 图像 → `Detection2D`（类别、置信度、像素框），YOLO26 |
| L1.5 跟踪 | `tracker.py` | 逐帧检测框 → 跨帧稳定 `track_id`（IoU + 匈牙利匹配） |
| L2 几何定位 | `localizer.py` + `geometry.py` | 框 + 深度图 → 相机系/基座系 3D 点（无深度时退回地面约束测距） |
| L3 融合 | `fusion.py` | 激光点云投影到图像 + 与检测框数据关联 → 修正距离 |
| L4 标准接口 | `types.py` + `pipeline.py` | 统一 JSON 结构，供 ROS2 节点发布 |

核心只依赖 numpy / opencv，**不依赖 ROS、不依赖 Isaac Sim**，所以能直接在 Windows 上跑和测；
ROS2 只是外面的一层壳（`ros2_ws/src/atongmu_perception_ros`）。

## 快速开始（本地，无需 ROS / Isaac）

```bash
cd perception
pip install -r requirements.txt        # ultralytics / opencv / numpy / pyyaml

# 1) L1：单图检测（首次运行自动从 GitHub 下载 yolo26s.pt 到 weights/）
python scripts/demo_detect.py
python scripts/demo_detect.py --source path/to/video.mp4 --max-frames 100

# 1b) 摄像头：原生窗口 + 终端逐帧置信度
python scripts/demo_detect.py --source 0 --verbose --duration 15
python scripts/demo_detect.py --probe-cameras        # 摄像头打不开时先体检

# 1c) 浏览器监控页（实时画面 + 置信度表格；没有图形界面的云电脑也能用）
python scripts/live_preview.py --source 0            # 自动打开 http://127.0.0.1:8080
#     默认同时录制到 perception/runs/preview_<时间戳>/annotated.mp4
#     --no-save 关闭录制；--out 指定输出目录；--record-fps 调回放速度
#     用 Ctrl+C 结束，进程会正常收尾写完 mp4

# 2) L2+L3：合成场景 3D 定位 + 视觉-激光融合数值验证
python scripts/demo_locate_3d.py
python scripts/demo_locate_3d.py --pitch-deg 12 --noise-depth 0.02

# 3) 单元测试（不依赖 pytest）
python tests/run_all.py -v

# 4) 训练自定义障碍物模型（数据集用 Replicator 生成，见 ../isaac_sim/replicator/README.md）
python scripts/train_yolo.py --data ../datasets/atongmu_obstacles/data.yaml \
    --base yolo26s.pt --epochs 80 --imgsz 640 --batch 16 --device 0
#    训练完把 best.pt 放到 weights/，并在 configs/perception.yaml 改
#    detector.weights 和 detector.classes（classes 必须与 data.yaml 的 names 一致）
```

产物写到 `perception/runs/<时间戳>/`（已被 .gitignore 忽略，不进仓库）。

## 当前结果（2026-10-29，本机 RTX 4060 Laptop / torch 为 CPU 版）

**L1 检测**：`bus.jpg`，yolo26s，640，CPU

| 指标 | 数值 |
| --- | --- |
| 检测数 | 4 个 person |
| 单帧总耗时 | 429 ms（含预处理/后处理，CPU） |
| 说明 | 云电脑用 `device: cuda:0` 后预计 10–20 ms 量级 |

**L2/L3 定位**（合成场景：10×6 m 房间，相机高 0.35 m，深度噪声 10 mm，激光噪声 20 mm，2% 深度丢点）

| 物体 | 方法 | 真实距离 | 估计距离 | 位置误差 |
| --- | --- | --- | --- | --- |
| person | RGB-D 深度 | 3.783 m | 3.774 m | 0.9 cm |
| chair | RGB-D 深度 | 2.462 m | 2.452 m | 0.9 cm |
| bottle | RGB-D 深度 | 1.713 m | 1.703 m | 1.0 cm |
| person | 深度+激光融合 | 3.783 m | 3.779 m | 0.6 cm |
| chair | 深度+激光融合 | 2.462 m | 2.458 m | 0.5 cm |
| bottle | 深度+激光融合 | 1.713 m | 1.699 m | 2.2 cm |

结论：几何链路正确，误差在 1 cm 量级；小目标（瓶子）点少、融合收益有限，这是后续要处理的问题。
`--pitch-deg` 可验证相机低头安装的情况。

## 接口约定

话题、JSON 字段、坐标系约定见 [`../docs/perception_interface.md`](../docs/perception_interface.md)。
一句话：`/perception/object_poses` 发布 `{"objects":[{"class_name","position":[x,y,z],"localization":{...}}]}`，
坐标在 `map` 系（拿不到 TF 时退回 `base_link` 系）。

## 接入 Isaac Sim（云电脑）

```bash
# 1) 让 ROS2 能找到核心库（或把 perception/ 加入 PYTHONPATH）
pip install -e perception/

# 2) 编译节点
cd ros2_ws && colcon build --packages-select atongmu_perception_ros
source install/setup.bash

# 3) 先确认 Isaac Sim 的话题名和坐标系名
ros2 topic list | grep -E "image|camera_info|scan"
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame

# 4) 启动
ros2 launch atongmu_perception_ros perception.launch.py \
    image_topic:=/camera/color/image_raw depth_topic:=/camera/depth/image_raw
```

需要和场景组对齐的三件事：
1. 话题名（彩色图 / 深度图 / camera_info / scan）；
2. **相机光学系**的 frame 名（必须是 z 朝前的那种，如 `camera_color_optical_frame`）；
3. 雷达 frame 名与扫描角度范围。

## 已知限制 / 下一步

- [ ] 类别集合需要与场景组确认（当前 COCO 白名单：person / chair / bottle / cup / dining table / couch / backpack / potted plant / laptop）
- [ ] 激光只有单线，物体较矮时扫不到顶面，融合对远处小目标不稳（瓶子 2.2 cm，仍是目前最大误差来源）
- [x] 目标跟踪：IoU + 匈牙利匹配，`object_poses` 输出稳定 `track_id`（新目标 `min_hits` 帧后确认）
- [ ] 无深度图时的地面约束测距假设目标贴地，桌上物体（瓶子/杯子）会偏大
- [ ] ROS2 节点尚未在云电脑实机跑通（本机没有 ROS2），首次联调需要确认编码（`32FC1`/`16UC1`）与 TF 方向
- [ ] 与语义导航对接：等 `object_poses` 稳定后，改为自定义 `atongmu_msgs` 消息

## 文件

```text
perception/
├── atongmu_perception/     核心库（无 ROS 依赖）
│   ├── config.py           配置数据类
│   ├── detector.py         L1 YOLO26 封装
│   ├── tracker.py          L1.5 跨帧跟踪（IoU + 匈牙利）
│   ├── geometry.py         针孔模型 / 变换 / 深度采样 / 地面约束
│   ├── localizer.py        L2 定位
│   ├── fusion.py           L3 激光-相机关联
│   ├── pipeline.py         主流程
│   ├── types.py            数据结构（接口契约）
│   ├── camera.py           摄像头打开/体检、GUI 可用性探测
│   └── viz.py              可视化
├── configs/perception.yaml 参数
├── scripts/                demo_detect / live_preview / demo_locate_3d / train_yolo（自定义障碍物模型）
├── tests/                  36 个单元测试
└── weights/                模型权重（不进仓库）
```
