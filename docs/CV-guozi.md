# CV-guozi：视觉感知与沿途语义观测

本PR首次提交视觉模块，基于main独立新增文件，不合并或修改同学的导航、语义任务与随机场景分支。

## 入口

- `perception/README.md`：检测、跟踪、RGB-D/雷达几何处理与训练入口。
- `ros2_ws/src/atongmu_perception_ros/README.md`：ROS2封装。
- `docs/planar_perception_handoff.md`：当前二维协议与JetBot话题/TF约定。
- `docs/semantic_route_20261126.md`：固定视角、往返任务、连续检测图及观测图。
- `isaac_sim/replicator/README.md`：合成数据生成，真实USD资产pilot仍是小样本验证。

## 验证

```bash
python perception/tests/run_all.py
python -m unittest discover -s deployment/server_demo -p test_capture_route.py -v
```

本地感知75项、采集坐标变换2项通过。此前服务器部署版本71项通过，两个版本的测试范围不同。
最终仿真case `semantic_route_20261126_04`四段导航成功，计划约8米往返，位置误差
2.83/2.66/3.81/4.27cm；43张检测图、160组雷达/轨迹、683包二维观测。
精选JSON见 `experiments/semantic_route_20261126/final_04/`。

## 环境与依赖边界

核心几何测试依赖perception的requirements；检测推理另需YOLO权重与兼容的PyTorch环境。
权重、数据集、USD资产、录像与完整运行日志不入Git，由项目现有共享存储提供。
ROS2运行需要Humble、cv_bridge、tf2_ros、相机内参、对齐RGB-D、采集时刻TF及仿真时钟。

`deployment/server_demo/`是现有云端部署的增量工具，需要导航PR #6对应源码、
已有`start_demo.sh`/`stop_demo.sh`和独立视觉运行环境；这些导航依赖未复制进本PR。
`configure_fixed_view.py`只应对已建立的独立部署副本执行，会备份并适配该副本，
不是对团队冻结导航源码或PR分支的修改。只克隆main加本PR不能独立启动完整导航仿真。

## 结论范围

当前只验证感知与导航并行、沿途采集和map坐标累计观测；导航仍使用已有先验地图，
没有完成在线SLAM、视觉寻物决策或PPO训练。检测存在叉车/托盘等明显误检，
语义图中的类别未人工核验，跨帧身份与置信融合尚需完善，不写入Nav2障碍地图。
