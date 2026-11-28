# 沿途检测与二维语义观测图

本次修改位于 `deployment/server_demo/`。自动演示从约 `(0,6)` 开始，依次提交
`(0,7,π/2)`、`(-3,7,π)`、`(0,7,0)`、`(0,6,-π/2)`，约8米往返。
这些是原生 Nav2 任务，不发布手动速度，不放宽保护门或到点阈值。

`configure_fixed_view.py` 为服务器启动脚本和模拟器保存备份，添加固定观察相机，
补齐新 tmux 会话的 USD 依赖环境。相机只影响观察窗口，不改机器人算法相机和传感器。
固定机位为`(-1.5,9,4)`，看向`(-1.5,6.7,0.2)`，避开屋顶和y=3.4..3.8立柱。
终点控制调参：最大前进速度0.4m/s，线速度采样41、角速度采样31，
停止判据速度0.02m/s。位置/朝向到点阈值仍为0.05m/0.05rad，安全保护未放宽。

`demo_video.sh` 并行运行 `capture_route.py`。每2秒**仿真时间**保存相机与检测、
map系平面观测三者时间戳匹配的图片。雷达端点、机器人轨迹使用采集时刻TF，
遇到延迟最多等待2秒，不拿最新TF替代。采集失败会使演示返回失败。

每个case的 `route_evidence/` 包含：

- `detection_*.png`：沿途带类别、置信度与采集时间的检测图，空检测帧也保留。
- `frames.jsonl`：逐图对应的原始检测与平面语义观测。
- `observations.jsonl`：全程map坐标下的视觉语义测量。
- `scans.jsonl`：采集时刻变换到map坐标的有效雷达回波端点。
- `trajectory.jsonl`：map系估计轨迹，和仿真真值分开。
- `occupancy_map.json`：若收到`/map`则保存栅格、原点和分辨率；这是已有导航地图，非新建SLAM地图。
- `semantic_route.png`：固定map坐标的累计雷达端点、估计轨迹与视觉测量点。
- `SUMMARY.json`：图片数、雷达采样数、TF等待次数、估计行驶距离与采集结果。
- `route_detections.jpg` / `REVIEW.json`：按至少0.75米空间间隔挑选的检测拼图与联合检查结果。

语义图是**观测证据图**：灰色是雷达回波，蓝色是定位轨迹，红色是未人工核验的视觉类别。
不把回波连线推断为空闲空间，不虚构物体中心或尺寸，不改Nav2地图。视觉点按类别与25cm
空间单元保存最近观测，不等于对象身份关联；person只保留原始观测，不固化为静态地图。
已有5秒语义缓存保持原有时效，避免把历史人或推车直接写入导航障碍层。

现有导航使用预建地图与雷达定位/避障，本轮不应表述为已完成在线SLAM。
检测模型仍有外观域差距，沿途图片用于检查误检、定位与覆盖，不能直接证明语义正确率。

服务器启动方式：

```bash
bash /home/admin/atongmu_deploy_20261120/demo_tools/launch_demo.sh NEW_UNIQUE_ID record
```

输出根目录：`/home/admin/jetbot-warehouse/artifacts/NEW_UNIQUE_ID/`。
完成情况同时查看 `VISIBLE_DEMO_RESULT.json`、`route_evidence/SUMMARY.json` 和录像；
不能仅凭图片或视频文件存在宣称导航成功。
