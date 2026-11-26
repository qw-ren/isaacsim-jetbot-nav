# 工作进度日志

按日期记录已完成、决策与阻塞，便于汇报和交接。阶段目标与验收口径见
[`milestone_2026-11-27.md`](milestone_2026-11-27.md)。

---

## 2026-11-26：固定视角、终点控制与沿途语义证据

- 修复新tmux环境缺失USD路径导致仿真启动失败；固定相机改为仓库屋顶以下、立柱同侧。
- 首轮实际导航到点附近发生漂移并超时，保留`semantic_route_20261126_02`失败证据。
  将最大前进速度改为0.4m/s，线/角采样41/31，停止速度判据0.02m/s；保留0.05m/0.05rad
  到点条件与所有保护阈值。`03`两段导航成功后为调整录像机位主动停止，不作为完整四段验收。
- 最终case `semantic_route_20261126_04`：从约(0,6)出发，经(0,7)、(-3,7)、(0,7)回到(0,6)，
  四个原生Nav2任务全部成功，估计终点误差2.83/2.66/3.81/4.27cm，无保护门fault。
- 43张时间戳匹配检测图、160组雷达/轨迹、683包map系视觉观测；接收1次已有占据地图。
  562帧观察相机画面合成89.96秒MP4，仿真正常响应停止、无SIM错误，相关tmux会话退出。
- 新增雷达端点/视觉测量/估计轨迹累计观测图、空间抽样检测拼图、原始JSONL和联合检查。
  计划往返约8米；估计轨迹折线累计10.38米包含定位抖动与转向，不能当作真值里程。
- 两项新增坐标变换测试通过；服务器既有部署版71项感知测试通过（不冒充最新数据分支75项）。
- 仍使用已有地图定位导航；累计语义图是观测图，不是在线SLAM或已验证的语义导航地图。
  视觉误检、跨帧关联与语义置信融合仍需完善，未替换检测模型或训练PPO。
- 复现说明：`docs/semantic_route_20261126.md`；本地证据：`experiments/semantic_route_20261126/`。

---

## 2026-11-24：真实仓储资产数据试验

- 优先处理合成几何体到真实仓库外观的域差距，继续保持二维检测和平面定位范围。
- 根据PR8已验证资产表生成真实USD小样本配置：纸箱4种、托盘2种、路障锥、推车，共8实例；
  保留原八类ID，不将湿地牌同时作为barrier和sign。缺少四类正样本，明确标记pilot_only。
- 云端生成12帧时发现继承语义导致复合标签barrier,cone及零件重复框。修改生成脚本，
  只在组合场景层清除引用实例旧语义，给实例根节点统一类别，不修改官方资产源文件。
- 同种子复测12帧（10训练/2验证），框数由150降至86，无空帧，无复合标签跳过警告，
  结构检查通过；已抽看框图。定位是数据流程小样本验证，不是模型精度评估。
- 标签校验增加小数类别ID、非有限数及框边界检查；本地75项测试通过。
- 未训练或替换生产权重。后续仍需仓库背景、机器人视角、叉车负样本和缺失类别覆盖。
- 证据：`experiments/real_asset_pilot_20261124/RESULT.json`，云端数据目录以该记录为准。

---

## 2026-11-20（本地开发，未服务器验证）

- 用户确认视觉与地图只做二维；新增 `/perception/objects_2d` 平面观测协议，
  含类别、会话内跟踪 ID、XY、采集时间和定位方式，不输出三维盒或虚构尺寸。
- 新增 `jetbot_planar.launch.py` 对齐 PR #6 的 JetBot v2 话题及 chassis/optical frame。
- ROS 节点使用采集时间 TF 和 BEST_EFFORT 传感器订阅；拒绝不匹配的 RGB-D，
  修正 odom 退回后错误标成 map 的问题。地图 TF 不可用时可拒绝输出或明确输出 chassis。
- 核对 PR #6/#7/#8 后修正旧日志结论：团队导航基线、语义真值实验和随机场景已在
  未合并分支中交付，本地副本未包含，不能据此认定团队 baseline 全缺。
- 新增平面协议/坐标回归测试，以及只读服务器观测验收脚本；本地 46 项测试通过，
  perception/ros2_ws 共 29 个 Python 文件 AST 语法检查通过。
- PR #7 当前要求三维盒子，下游二维观测消费者尚未实现；没有声称模型语义导航闭环完成。
- 服务器暂不连接。接口、复现记录与后续步骤见 `planar_perception_handoff.md`。
- 同日补充测试：62 项通过（含 14 项替身 ROS 回调测试），30 个 Python 文件语法检查通过。
  修复生产端时间戳/序号/轨迹字段校验缺口；增加 TF 延迟到达时的有界等待重试和过期清理。
  真实 ROS2/模型推理/Isaac Sim 仍未验证，结果见 `experiments/planar_local_20261120/RESULT.json`。

### 同日云端部署与真实联调

- 后续已获授权连接云电脑并部署：ROS2 Humble/Nav2安装，固定源码FAST-LIVO重建，
  PR4/5/6/7/8交付分目录保存，视觉使用独立兼容环境。
- 新增二维观测缓存；本地及云端71项测试通过，仓储配置73项通过，1001/medium布局hash复现。
- 实际RGB-D/TF到map平面视觉观测通过30秒检查（63帧/62非空/0错误），输入审计通过。
- 视觉运行期间Nav2短目标(1.5,6,0)成功，保护门无fault；本轮测试后正常停止。
- 本次是系统ROS与重建后端的独立部署变体，不继承原冻结版本所有验收结论。
- 发现实际仓库外观下存在误检；二维缓存尚未驱动旧三维盒语义管理器，随机布局未联合导航。
- 完整范围、证据和启动/停止命令见 `server_deployment_20261120.md`。

---

## 2026-11-12

### 决策

- **语义地图只做平面**：二维占据栅格，不做三维重建、点云建图、三维框和体素表示。
  感知输出的 `position` 只取 `(x, y)` 建图，`z` 仅用于过滤悬空误检；激光用单线雷达的平面扫描。
- **八类障碍物定稿**：`shelf / wall / box / pallet / barrier / cart / sign / person`，
  每类是否进先验地图、能否作语义目标见 milestone 文档中的物体表。
- **baseline 不再等外部资产**：本地资产库已有 `jetbot.usd`、`Simple_Warehouse`、
  `Digital_Twin_Warehouse`、`Simple_Room`，可以自行搭场景。

### 完成

1. **Replicator 取景修复并重新生成数据集**
   - 问题：原配置场地 7 m（对角半径 4.95 m）、相机距离 4.5–7.5 m，相机会落进障碍物堆里，
     抽帧可见画面被近处立方体占满、大量框贴边。
   - 改法：场地缩到 5.2 m（`area: ±2.6`），相机拉到 7–10 m 外环、高度 0.35–1.5 m，
     注视点抖动幅度跟场地大小走。
   - 先用 60 帧验证，抽帧确认全景可见、框贴合后重新生成全量。

2. **数据集（`/home/admin/datasets/atongmu_obstacles`）**
   - 3000 帧：train 2550 / val 450，0 空标签帧（空标签也会写 txt 作负样本）。
   - 共 33,499 个框：shelf 5,176 / wall 5,207 / box 7,919 / pallet 3,660 /
     barrier 4,058 / cart 1,982 / sign 3,541 / person 1,956。
   - 生成速度约 1000 帧/分钟，Isaac Sim 4.5 headless。

3. **YOLO 训练**
   - YOLO26s，80 epoch，batch 16，640×640，device 0（RTX 5880 Ada 16 GB），约 65 分钟。
   - val 450 张 / 5,014 实例：**P 0.935 / R 0.901 / mAP50 0.958 / mAP50-95 0.864**。
   - 分列 mAP50：person 0.979、shelf 0.968、pallet 0.968、wall 0.963、box 0.963、
     barrier 0.948、cart 0.940、sign 0.937。
   - 弱点：sign 的 mAP50-95 只有 0.712（小而薄），pallet 0.820（扁平面上下边界）。
   - **指标只代表合成数据**，证明流程与类别可分性，不代表真实场景精度。

4. **部署与验证**
   - `best.pt` → `perception/weights/obstacles_best.pt`。
   - `perception/configs/perception.yaml` 与
     `ros2_ws/src/atongmu_perception_ros/config/perception_ros.yaml` 的 `detector.classes`
     改为上述 8 类，`weights` 指向新模型。
   - 云电脑 `demo_detect.py`（cuda:0）：单图检出 8 个目标，冷启 1.58 s，稳态推理约 10 ms/帧。
   - 云电脑 `demo_locate_3d.py`：平面/深度定位最大误差 2.2 cm（阈值 10 cm）。

5. **修掉一个静默 bug**
   - `DetectorConfig.from_dict` 用 `hasattr(cls, k)` 过滤字段，而 `classes` 是
     `field(default_factory=...)`，类上没有同名属性 → yaml 里的 `classes` 从未生效，
     一直用代码里的 COCO 默认表。之前用预训练 COCO 权重时两者恰好一致，所以没暴露。
   - 改为按 dataclass 字段名过滤，新增 `perception/tests/test_config.py` 三个回归用例。
   - 本地 39 个单元测试全部通过。

### 环境事实（云电脑）

| 项 | 现状 |
| --- | --- |
| 系统 / GPU | Ubuntu 22.04，RTX 5880 Ada 16 GB，驱动 570.172.18 |
| 训练环境 | `/home/admin/venvs/yolo`：torch 2.8.0+cu128、torchvision 0.23.0、ultralytics 8.4.211 |
| Isaac Sim | 4.5.0，`/home/admin/isaacsim`；资产 `/home/admin/isaacsim_assets`（112 G） |
| ROS2 | **未安装**；Isaac Sim 自带 humble 运行库缺 `cv_bridge` / `tf2_ros` |
| 外网 | GitHub 直连可用但约 100 KB/s；PyPI 走清华镜像 |

- **CUDA 大版本坑**：PyPI 最新 torch 会拉 CUDA 13 运行库，驱动 570 最高支持 CUDA 12.8，
  装完 `cuda.is_available()` 为 False 且静默退回 CPU。必须锁 cu128 的 wheel。

### 未完成 / 阻塞

- **baseline 全缺**：Isaac Sim 场景、JetBot、相机/雷达、ROS2 Bridge、SLAM/Nav2
  在两份代码库中都不存在（`D:\文件\...` 那份只有 10 个占位文件）。
- **ROS2 联调未开始**：需装 ROS2，或给节点补不依赖 `cv_bridge` 的兜底。
- **PPO 未开始**：Isaac Lab 是否可用 / 用哪套底座待与航硕确认。
- **真实障碍资产未到位**：当前用几何体近似，存在外观域差距。

### 下一步

1. 搭 baseline 场景（JetBot + 相机 + 深度 + 单线雷达 + ROS2 Bridge，二维 SLAM + Nav2）。
2. 感知节点在 ROS2 中跑通，核对 `class_name` / `(x, y)` / `track_id` 与语义地图约定。
3. PPO 训练环境（随机化障碍），替换 DWB 并与经典方法对比。
4. 语义目标导航闭环：识别目标类别 → 平面坐标 → Nav2 目标点。

### 汇报材料

- `docs/progress_2026-11-12.html`：可视化的进度页（浏览器直接打开）。
