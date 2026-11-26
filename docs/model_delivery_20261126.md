# 八类模型与二维观测交付

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
