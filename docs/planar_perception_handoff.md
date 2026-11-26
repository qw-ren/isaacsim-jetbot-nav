# 二维视觉感知对接（2026-11-20）

当前范围：图像上的二维检测框 + 地面平面中的目标观测点 `(x, y)`。
不做三维框、三维地图或三维重建。深度/激光投影和相机倾角变换内部仍使用已有
几何运算，最终只取平面 XY；不把物体表面点冒充物体中心，也不直接当成机器人目标。

## 接口

`/perception/detections` 保留二维像素框（std_msgs/String JSON）。新增
`/perception/objects_2d`，同样用 std_msgs/String，协议示例：

```json
{
  "schema": "atongmu.planar_observations",
  "version": 1,
  "source": "vision",
  "session_id": "example-session",
  "sequence": 3,
  "stamp": 12.5,
  "frame_id": "map",
  "objects": [{
    "object_id": "vision:example-session:7",
    "track_id": 7,
    "class_name": "box",
    "confidence": 0.9,
    "position_xy": [-3.0, 7.0],
    "position_reference": "visible_surface",
    "range_m": 2.5,
    "localization_method": "depth"
  }]
}
```

- XY、range 为米，stamp 为 `/clock` 仿真秒；range 是采集时到车体的水平距离，
  不等于 map 原点到该点的距离。无有效定位或未确认的轨迹不进入 objects。
- `depth/lidar/lidar+depth` 定位的是可见表面；`ground` 是框底边地面交点估计，
  对应 `position_reference=ground_contact_estimate`，悬空物体不满足该假设。
- 相机二维框不能确定物体平面完整轮廓，所以不输出虚构尺寸、朝向或三维盒子。
- object_id 含启动 UUID，防止重启混淆；跟踪丢失后可能产生新 ID，不是永久实例身份。
  下游需要做跨视角/多帧关联，不能简单把每个新 ID 永久累加成地图物体。
- 空 objects 表示此帧没有可用观测，不表示历史物体立即消失。
  下游应同时检查仿真时间和接收墙钟的过期，停止节点后旧目标也必须过期。
- 本接口无真值输入，无速度输出，不发布 TF。TF 不可用时不使用最新 TF 替代。

## 与已有 PR 的关系

核对版本：PR #6 `0d957b984c7c5cfb7570caf3be255eabdda149e2`；
PR #7 `5b3d11529f1de3d66ef930400279e2e11e7120a2`；
PR #8 `13e42258689dc3b28c8e8f5f8d506f3513249f6e`。

PR #7 的 `/semantic/objects/observations` v1 要求三维盒子，本协议**不能直接发到该话题**。
下一步需在导航分支增加二维观测消费者：关联观测点，在二维占据地图和激光几何上
选择可达接近点，复用现有任务管理和 Nav2。仅一个表面点无法支持旧版依赖物体完整
包围盒的路径检查。此次没有修改 PR #7 的 mapper，也没有声称视觉寻物导航已闭环。

PR #6 专用启动参数已写在 `jetbot_planar.launch.py`：

| 项目 | 配置 |
|---|---|
| RGB / CameraInfo | `/jetbot/image_raw` / `/jetbot/camera_info` |
| 对齐深度 | `/jetbot/depth/image_rect_raw`，32FC1 米制光轴 Z |
| 平面雷达 | `/jetbot/scan`，frame 为 `chassis` |
| 车体 / 光学 / 地图 | `chassis` / `jetbot/camera_optical` / `map` |
| 时间 / QoS | use_sim_time=true；BEST_EFFORT 传感器订阅 |
| 输出 | 检测框 + objects_2d；旧 object_poses 与三维 Marker 默认关闭 |

CameraInfo 和实际 TF 覆盖离线标定；必须先收到匹配分辨率/光学系的有效内参。
RGB-D 必须同时间戳、同 frame、同分辨率；未知深度编码拒绝。
相机与地图变换查询图像采集时间；雷达在 0.06 秒内才融合，并通过 odom 固定系
查询雷达采集时间到图像采集时间的变换。缺少该变换时只用深度/地面约束。
默认 require_map=true，缺少 map TF 时不发布结果。诊断可设 require_map=false，
此时结果如实标记为 chassis；下游不能把 chassis XY 直接当地图坐标。
时间回退后停止处理，换种子/reset 后须重启感知节点和下游会话以清除旧轨迹。

PR #8 自带的 `/rgb /scan /odom /cmd_vel` 与 PR #6 的命名和定位来源不同，
不能直接并行启动两套机器人/TF/控制图。先由仿真/导航集成方确认场景底座，
不要把 PR #8 的仿真 odom 当成 PR #6 的 FAST-LIVO2 估计里程计。

## 本地检查

```bash
cd perception
python tests/run_all.py -v
```

本地只验证 Python 算法/协议和源码语法，未运行 ROS2 或 Isaac Sim。

## 服务器验收步骤（连接后执行）

导航 PR #6 的 ROS_DOMAIN_ID 固定为 51。视觉、探针和验收终端都必须显式设置同域；
隔离接口测试历史使用的 52 不能用于连接实际导航会话。
仓库不是完整环境镜像；固定 FAST-LIVO 二进制、原始 USD、规划图和 NVIDIA 资产仍需
在云电脑已有目录中存在，来源见 PR #6 的 `assets/jetbot_resources.md`。

优先使用导航已经验证的环境脚本（普通 ROS 安装的 setup.bash 不包含所有工程补充库）：

```bash
source /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_stage2/nav_env.sh
export ROS_DOMAIN_ID=51
# 进入上传了本地新代码的项目根目录后：
export PYTHONPATH="$PWD/perception:$PYTHONPATH"
python3 ros2_ws/tools/check_cloud_perception_env.py
```

自检以当前 python3 实际导入 ROS、图像桥、核心库和模型运行库，检查所需资源路径。
它不安装依赖、不启动仿真。CUDA 不可用会出现在报告中；默认 CUDA 配置下推理仍需
修复 GPU 环境，或在独立感知配置副本中明确选 CPU。核心库/模型/ROS 需在同一兼容
Python 环境中；不要把 Windows Python 或 Isaac 内置 Python 的包目录直接拼给 ROS Python。

1. 使用现有 Isaac Sim 4.5、ROS2 Humble 与模型环境，不重新安装 torch/CUDA。
   确认核心库的 Python 与 ROS2 ABI 兼容，cv_bridge/message_filters/tf2_ros 可用。
2. 在独立部署目录放入修改后的 `perception/` 和 `ros2_ws/`；确认
   `perception/weights/obstacles_best.pt` 是八类权重，ROS Python 能导入核心库。
   例如在既有 ROS Python 环境设置 `export PYTHONPATH="$PWD/perception:$PYTHONPATH"`。
3. 使用 PR #6 的 v2 新 case，显式开启 extensions.depth=true。
   首次不同时录像，先保持机器人静止、目标在算法相机视野内。
4. 先运行 `python3 ros2_ws/tools/perception_probe.py --timeout 15`，确认话题、
   内参、光学坐标系和深度编码与上表一致。以下命令从项目根目录开始：

```bash
source /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_stage2/nav_env.sh
export ROS_DOMAIN_ID=51
export PYTHONPATH="$PWD/perception:$PYTHONPATH"
cd ros2_ws
colcon build --packages-select atongmu_perception_ros --symlink-install
source install/setup.bash
ros2 launch atongmu_perception_ros jetbot_planar.launch.py
```

另一个已 source 相同 ROS 环境并 export ROS_DOMAIN_ID=51 的终端，从项目根目录执行只读验收：

```bash
python3 ros2_ws/tools/check_planar_stream.py --seconds 30 --frame map \
  --out experiments/planar_check_NEW_CASE
```

脚本不会发送运动指令，目录必须是新目录。成功要求至少一帧非空、协议和坐标系正确、
仿真时间及序号递增、接收时龄合格；产出 observations.jsonl 和 summary.json。
该检查不证明识别精度、定位误差或导航成功。若无结果，先查 CameraInfo/同帧深度/采集时间
TF，再查 QoS、权重路径和类别；不要通过改 latest TF 或虚填 map 绕过问题。

节点会用最多 5 帧的有界队列等待采集时间 TF：每 0.02 秒重试，单帧最多等待
0.25 墙钟秒，过期丢弃。定时器使用单调时钟，仿真暂停时也能清理过期帧。
不会为了等 TF 阻塞订阅回调，也不把缺失的变换换成 latest TF。

5. 后续由现有导航流程控制机器人，检查转弯时静止目标在 map 中是否稳定，
   单独统计平面位置误差。下游二维消费者完成后再验收寻物导航。
6. 随机场景先用 1001/medium 验证，再换种子。记录 seed、density、layout_hash、
   配置和底图摘要、机器人版本及基线 ID；seed 本身不能完整确定实验。
   场景 reset 后重启感知和地图会话。训练/验证/测试的种子划分尚待 PPO 线统一。

## 本地补充验证（2026-11-20）

62 项测试通过，其中 14 项使用替身 ROS 消息/TF/发布器执行实际节点回调和真实几何
pipeline，覆盖采集时间 TF、TF 延迟重试与过期、帧坐标标记、旋转坐标变换、米/毫米
一致性、重复帧、RGB-D 不匹配和时间回退。30 个 Python 源文件语法检查通过，
package.xml 解析及 launch 文件打包路径检查通过。

发现并修正生产端零时间戳/非法序号/非法 track_id 的校验缺口，以及 TF 晚到时
直接丢帧且无重试的问题。这里的 ROS 替身不验证真实 DDS、执行器调度、cv_bridge ABI、
模型推理或 Isaac Sim。结果记录在 `experiments/planar_local_20261120/RESULT.json`。
