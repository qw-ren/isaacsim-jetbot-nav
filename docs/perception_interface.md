# 感知模块接口约定

> 2026-11-20：当前二维平面接口和 PR #6/#7/#8 对接以
> [planar_perception_handoff.md](planar_perception_handoff.md) 为准。
> 本文保留旧接口历史；其中 COCO 类别、默认 frame 及把目标位置直接发给 Nav2 的伪代码
> 不适用于当前 JetBot。物体观测点需要导航侧选择可达接近位姿，不能直接用作机器人目标。

> 面向团队里做 **语义导航** 和 **RL 局部避障** 的同学。本文档就是感知模块对外承诺的「标准接口」，
> 只要照着接，感知内部的实现怎么改都不影响你们。

## 1. 订阅 / 发布总览

感知节点（`atongmu_perception_ros`）发布两个数据话题、一个可视化话题：

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/perception/object_poses` | `std_msgs/String`（JSON） | **3D 目标**：类别 + 世界坐标（导航/RL 主要用这个） |
| `/perception/detections` | `std_msgs/String`（JSON） | 2D 检测框（调试、做可视化用） |
| `/perception/markers` | `visualization_msgs/MarkerArray` | RViz 里直接能看的小球 + 距离标签 |

之所以先用 `std_msgs/String(JSON)` 而不是自定义消息：你们可以零依赖先接上，不用等自定义 msg 包编译；
字段稳定后再平移成 `atongmu_msgs`，两边字段一一对应。

## 2. `/perception/object_poses`（语义导航 / RL 请读这个）

```json
{
  "stamp": 1730200000.123,
  "frame_id": "map",
  "latency_ms": 12.4,
  "objects": [
    {
      "detection_id": 0,
      "track_id": 7,                     // 跨帧稳定 ID；未确认的新目标可能暂时没有该字段
      "class_name": "chair",
      "confidence": 0.92,
      "position": [3.21, -1.05, 0.42],   // x 前, y 左, z 上（米）
      "frame_id": "map",
      "stamp": 1730200000.123,
      "localization": {
        "range_m": 3.38,                 // 到机器人的水平距离
        "method": "lidar+depth",         // depth | ground | lidar | lidar+depth | none
        "n_points": 18,
        "xyz_camera": [0.1, -0.2, 3.3],
        "xyz_base": [3.21, -1.05, 0.42],
        "yaw_base": -0.31               // 相对机器人朝向，弧度
      },
      "size_estimate": [0.5, 0.9, null]  // 宽, 高, 深（单视角深向估不出）
    }
  ]
}
```

字段说明：
- `track_id` 是跨帧稳定的目标编号（同一物体在连续帧里保持不变），适合表达
  「跟着刚才那个人」。新目标需要连续命中 `min_hits` 帧才会出现该字段，轨迹丢失
  `max_age` 帧后 ID 作废且不复用；做语义地图时建议以 `track_id` 为键做多帧融合。
- `position` / `xyz_base` 都是米，ROS 惯例：**x 前进、y 左、z 上**。
- `frame_id` 是 `map`（TF 能拿到 `map→base_link` 时）；拿不到时是 `base_link`，并在 `frame_id` 里如实写。
- `localization.method` 表示这个距离怎么来的，语义导航建议按它给结果加权：
  - `lidar+depth` / `lidar` / `depth` 可信度高；
  - `ground` 是单目地面约束兜底，目标不贴地（瓶子、杯子放桌上）时距离会偏大，只能当方向参考；
  - `none` 表示这帧完全没定位成功，下游应忽略。
- 感知**不输出**任何“该去哪”的决策，只输出事实。

## 3. `/perception/detections`（2D，调试用）

```json
{
  "stamp": 1730200000.123,
  "frame_id": "camera_color_optical_frame",
  "image_size": [640, 480],
  "detections": [
    {"class_id": 56, "class_name": "chair", "confidence": 0.92,
     "bbox": [220.0, 130.0, 340.0, 280.0], "stamp": 1730200000.123}
  ]
}
```

`bbox = [x1, y1, x2, y2]` 像素坐标，原点在左上角。

## 4. 坐标系与方向约定

- 世界/地图/基座/雷达：ROS 惯例，x 前、y 左、z 上。
- 相机光学系（`*_optical_frame`）：x 右、y 下、z 前（光轴）。**感知内部只在相机系做投影，对外输出一律转成 ROS 惯例**。
- 四元数顺序统一 `x, y, z, w`（ROS 标准）。
- 角度单位弧度，长度米，时间秒（Unix epoch）。

## 5. 目标类别（COCO 名，需与场景组最终确认）

`person` `chair` `dining table` `couch` `bottle` `cup` `backpack` `potted plant` `laptop`

只输出白名单内的类别；换权重后类别名不变即可无缝切换（代码里按名字解析 id）。

## 6. 简单用法示例（伪代码）

```python
# 语义导航：找最近的一把椅子
def on_msg(msg):
    data = json.loads(msg.data)
    chairs = [o for o in data["objects"] if o["class_name"] == "chair"]
    if not chairs:
        return
    nearest = min(chairs, key=lambda o: o["localization"]["range_m"])
    goal = nearest["position"]          # 直接当 Nav2 的目标点
    send_goal(goal, frame=data["frame_id"])
```

## 7. 对接前的清单（需要场景组 / 集成组确认）

1. 话题名：`image_topic` / `depth_topic` / `camera_info_topic` / `scan_topic`；
2. `base_frame`、`camera_frame`（**光学系**）、`laser_frame`、`map_frame` 的实际名字；
3. 深度图编码：Isaac Sim 的 ROS2 Bridge 通常发 `32FC1`（米），如果是 `16UC1`（毫米）节点会自动按毫米换算；
4. 目标类别集合。
