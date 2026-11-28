# atongmu_perception_ros

视觉感知模块的 ROS2 节点。核心算法在 `perception/atongmu_perception`（无 ROS 依赖），
这个包只是「订阅传感器 → 调 pipeline → 发布 JSON」的一层壳。

## 依赖

- ROS2 Humble（云电脑）
- 先 `pip install -e perception/`，让 ROS2 环境能找到 `atongmu_perception`

## 编译与运行

```bash
cd ros2_ws
colcon build --packages-select atongmu_perception_ros
source install/setup.bash
ros2 launch atongmu_perception_ros perception.launch.py
```

## 参数（可 launch 覆盖，见 launch 文件）

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `config_file` | 包内 `config/perception_ros.yaml` | 感知 YAML |
| `image_topic` | `/camera/color/image_raw` | 彩色图 |
| `depth_topic` | `/camera/depth/image_raw` | 深度图 |
| `camera_info_topic` | `/camera/color/camera_info` | 内参 |
| `scan_topic` | `/scan` | 单线激光 |
| `use_depth` / `use_scan` | `true` / `true` | 是否启用深度/激光 |
| `use_tf_extrinsics` | `true` | 用 TF 覆盖配置里的外参（推荐） |
| `base_frame` / `map_frame` | `base_link` / `map` | TF 坐标系 |
| `camera_frame` | `camera_color_optical_frame` | **光学系**（z 朝前） |
| `laser_frame` | `laser_frame` | 激光系 |

首次联调时先看 `ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame` 确认 frame 名，
再按实际值覆盖参数。

## 输出

见 `docs/perception_interface.md`。可视化：`ros2 run rviz2 rviz2` 后 add `MarkerArray`，topic 选 `/perception/markers`。
