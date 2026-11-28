# 八类模型与二维观测交付

2026-11-28 补充：2026-11-27 服务器实测已输出1145条map观测、1844个物体记录。
原始流位于 `experiments/far_range_20261127_01/route_evidence/observations.jsonl`，
采集时间、session/track ID及模型身份均保留。包中的torch版本元数据为 `2.8.0`，
ultralytics为 `8.4.211`；历史完整CUDA构建版本记录为 `2.8.0+cu128`。
当前文件是已采集证据，不是正在运行的实时话题。

从项目根目录离线核对模型与全部观测（无需ROS、torch或GPU）：

```bash
python3 ros2_ws/tools/verify_model_handoff.py \
  --weights perception/weights/obstacles_best.pt \
  --manifest perception/weights/obstacles_best.manifest.json \
  --observations experiments/far_range_20261127_01/route_evidence/observations.jsonl
```

本轮远距离已知箱体约3–16米，70个投影/深度一致性代理样本中均没有匹配box检测，
该目标的视觉XY定位不能验收。轨迹XY RMSE约1.57厘米仅描述机器人定位。
5/6个导航任务成功，最后一项被600秒诊断进程运行上限中断；完整结论见同case下
`RESULTS.json`。输出协议可用不等于远距离识别与目标定位可靠。

权重：`perception/weights/obstacles_best.pt`（20,312,901 bytes）。
SHA256：`367994e796f76c875eebccb018141e71af8c04b1dcbe25a26f8d428abf134a3d`。
类别 ID 0..7：`shelf, wall, box, pallet, barrier, cart, sign, person`。
类别顺序及 checkpoint 版本 8.4.211 来自静态读取权重内 data.pkl，未执行 pickle。
机器可读清单见同目录 `obstacles_best.manifest.json`。

推理接口版本 `atongmu.planar.v1`。历史服务器环境为 ultralytics 8.4.211、
torch 2.8.0+cu128；新代码每次启动计算实际权重摘要、核对清单，并在每条二维
观测的 `model` 字段附上实际安装的两个库版本，不能用历史版本冒充当前实测。

`/perception/objects_2d` 保留图像采集 ROS 时间 `stamp`，新增 `time_domain`
（ros_sim / ros_system）。`frame_id=map` 的 `position_xy` 单位为米；配置
`require_map=true`，缺少采集时刻 TF 时等待或丢帧，不能拿最新 TF 或机器人系冒充 map。
`session_id` 区分进程会话，`track_id` 是会话内视觉轨迹编号，遮挡后可能改变，
不能当作永久物体身份。坐标代表可见表面测量或明确标注的地面投影估计。
协议可正确提供上述字段，不代表模型误检率和坐标误差已通过现场验收。

旧演示 `(0,6) -> (0,7) -> (-3,7)` 往返没有证明穿行货架。
新入口必须提供路线：

```bash
bash /home/admin/atongmu_deploy_20261120/demo_tools/launch_demo.sh NEW_ID record /absolute/path/route.json
```

route.json 必须包含 `frame_id: "map"`、当前导航地图 `map_sha256` 和
`waypoints: [[x,y,yaw], ...]`，至少两个点。摘要目前用于记录地图来源，
并未自动比对运行中的地图；通道几何、机器人余量及实际路径仍需从服务器场景
确认并由 Nav2 验证。不提供虚构的货架坐标。运行时保存 `REQUESTED_ROUTE.json`。

本地执行23项原有二维接口/真实回调函数测试及1项路线契约测试；回调使用模拟 ROS
输入和真实几何流水线，不等于服务器 DDS、GPU 推理或货架导航实跑。

## 组员获取与接入

交付分支：`feature/CV-guozi`。需一起获取整个 `perception/`、
`ros2_ws/src/atongmu_perception_ros/`、`ros2_ws/tools/` 与本说明。
仅下载 pt 文件不能得到坐标变换、跟踪和 ROS 输出。
精确权重与清单必须放在同一目录，且保持上述文件名。

在项目根目录用普通 Python 校验下载文件（无需安装 torch）：

```bash
python3 -c 'import hashlib,json; from pathlib import Path; p=Path("perception/weights/obstacles_best.pt"); m=json.loads(p.with_suffix(".manifest.json").read_text()); assert hashlib.sha256(p.read_bytes()).hexdigest()==m["sha256"]; assert p.stat().st_size==m["size_bytes"]; print(m["classes"], m["inference_version"])'
```

使用已有 ROS2 Humble / cv_bridge 兼容环境，历史模型依赖为
`ultralytics==8.4.211` 和 `torch==2.8.0+cu128`。CUDA wheel 需与部署机器匹配，
完整 ROS/GPU 环境不包含在 Git 仓库中。新节点实际导入的版本记录在每条观测里。
不要仅凭 `perception/requirements.txt` 的宽范围认定推理环境相同。

仿真与导航启动并发布 RGB、对齐深度、CameraInfo、scan、采集时刻 TF 和 /clock 后，
在项目根目录的兼容 ROS Python 终端执行：

```bash
source /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_stage2/nav_env.sh
export ROS_DOMAIN_ID=51
export PYTHONPATH="$PWD/perception:$PYTHONPATH"
export ATONGMU_PROJECT_ROOT="$PWD"
cd ros2_ws
colcon build --packages-select atongmu_perception_ros --symlink-install
source install/setup.bash
ros2 launch atongmu_perception_ros jetbot_planar.launch.py \
  config_file:="$ATONGMU_PROJECT_ROOT/ros2_ws/src/atongmu_perception_ros/config/perception_ros.yaml"
```

上述环境脚本是历史服务器的路径；其他机器应 source 其实际导航/ROS 环境。
`jetbot_planar.launch.py` 已启用 `use_sim_time=true`、`require_map=true`。
模型默认相对路径解析到源码 `perception/weights/`，因此须保留源码 PYTHONPATH，
或将 detector.weights 改为下载权重的绝对路径。

节点连续向 `/perception/objects_2d` 发布 JSON。另一终端设置相同 ROS 环境、
ROS_DOMAIN_ID 和 PYTHONPATH，从项目根目录验收并保存原始观测：

```bash
python3 ros2_ws/tools/check_planar_stream.py --seconds 30 --frame map \
  --out experiments/planar_check_NEW_UNIQUE_ID
```

验收程序输出 observations.jsonl 和 summary.json，检查时间、序号、map 系和有效点；
模型身份由启动校验以及消息里的 model 字段核对。要持续建图，应持续订阅话题，
30秒检查程序仅用于短时采样，不是持续建图服务。

单条消息包含 `stamp`（图像采集时刻，非发布时刻）、`time_domain=ros_sim`、
`frame_id=map`、`session_id`、`sequence`、`model` 和 `objects`。
每个 object 包含 `object_id`、`track_id`、`class_name`、`confidence`、
`position_xy`、`position_reference`、`range_m`、`localization_method`。
`model.weights_sha256` 对应本页摘要，`model.classes` 对应 ID 0..7，
`model.runtime` 给出实际 ultralytics/torch 版本。

货架上悬空物品不可用 ground 地面投影作为可靠位置，应优先消费有效深度/激光
定位；地面与货架物品都只能识别上述八类。持久语义地图需另做多帧关联、
定位误差处理及动态目标过期，不能把 person 固化为静态货架障碍。
