# Replicator 合成数据 → YOLO 障碍物检测

目标：用 Isaac Sim 的 Replicator 自动生成带 2D 框标注的障碍物数据集，训练一个**够用就行**的
检测模型，让「视觉输出障碍类别 + 机器人坐标系位置 → 语义地图」这条链路先跑通。

类别已按课题物体表确定（2026-11-12），共 8 类：

| 类别 | 角色 | 先验地图 | 语义目标 |
| --- | --- | --- | --- |
| `shelf` 货架 | 固定仓储结构、通道边界 | 是 | 是 |
| `wall` 墙体 | 固定边界 / 不可通行 | 是 | 一般否 |
| `box` 纸箱 | A 类局部障碍、堆料 | 动态加入 | 是 |
| `pallet` 托盘 | 较大局部障碍 / 通道封堵 | 动态加入 | 是 |
| `barrier` 挡板 | B 类通道封闭 | 动态加入 | 否 |
| `cart` 推车 | 大体积 / 后续移动障碍 | 否 | 是 |
| `sign` 标牌 | 货架编号、区域识别 | 否 / 附着货架 | 是 |
| `person` 人 | 后续动态障碍 | 否 | 后续可作目标 |

> 真实 USD 资产还没到位，当前用 `primitive` 近似形体（立方体/圆柱）跑通流程；航硕给出资产路径后
> 只改 `configs/dataset_config.yaml`，代码不用动。`shelf` / `wall` 最终应来自场景 USD（先验地图），
> 这里的随机摆放只是为了检测链路先有正样本。

## 1. 环境

- Isaac Sim **4.5**（云电脑），使用它自带的 Python，不要用系统 Python：
  - Linux：`<isaac_sim>/python.sh`
  - Windows：`<isaac_sim>\python.bat`
- 不需要 ROS 2，也不需要先启动 Isaac Sim 界面（脚本自带 Standalone App）。

## 2. 先自检

```bash
# Linux
./python.sh isaac_sim/replicator/gen_dataset.py \
    --config isaac_sim/replicator/configs/dataset_config.yaml --check

# Windows
python.bat isaac_sim\replicator\gen_dataset.py ^
    --config isaac_sim\replicator\configs\dataset_config.yaml --check
```

`--check` 会启动一个无头 SimulationApp、导入 Replicator、打印类别与输出目录后退出。
如果这里就报错，先解决 Isaac Sim / GPU / 显示驱动问题，不要急着生成。

> 已在云电脑 Isaac Sim **4.5.0** 上实测通过（RTX 5880 Ada，headless，约 1000 帧/分钟）。

## 3. 生成数据集

```bash
# 先用 100 帧验证流程（几分钟）
./python.sh isaac_sim/replicator/gen_dataset.py \
    --config isaac_sim/replicator/configs/dataset_config.yaml \
    --num-frames 100 --overwrite

# 正式生成
./python.sh isaac_sim/replicator/gen_dataset.py \
    --config isaac_sim/replicator/configs/dataset_config.yaml --overwrite
```

产物（`output_dir` 相对配置文件解析，默认仓库根目录 `datasets/atongmu_obstacles/`）：

```text
datasets/atongmu_obstacles/
├── images/train/*.jpg      # 85%
├── images/val/*.jpg        # 15%
├── labels/train/*.txt      # YOLO: class cx cy w h（归一化）
├── labels/val/*.txt
├── data.yaml               # 直接给 ultralytics
└── dataset_info.json       # 类别、帧数、随机种子、配置路径
```

**没有目标的帧也会写空 txt**，作为负样本抑制误检。

### 3.1 校验数据集（普通 Python 即可，不需要 Isaac Sim）

```bash
python isaac_sim/replicator/validate_dataset.py \
    --data datasets/atongmu_obstacles/data.yaml --draw 20
```

会检查图片/标签是否配对、类别 id 是否越界、归一化坐标是否合法，并把 20 张带框的样例
写到 `datasets/atongmu_obstacles/validation/`。**先看这 20 张框是否贴住障碍物**，再开始训练。

## 4. 训练

训练在装了 ultralytics 的机器上跑（本机或云电脑的普通 Python 环境都行）：

```bash
cd perception
python scripts/train_yolo.py \
    --data ../datasets/atongmu_obstacles/data.yaml \
    --base yolo26s.pt --epochs 80 --imgsz 640 --batch 16 --device 0
```

训练完成后把 `best.pt` 放到 `perception/weights/`，并在 `configs/perception.yaml` 改：

```yaml
detector:
  weights: weights/obstacles_best.pt
  # 必须与 data.yaml 的 names 完全一致
  classes: [shelf, wall, box, pallet, barrier, cart, sign, person]
```

然后照常运行 `scripts/demo_detect.py` / `scripts/live_preview.py` 验证；
`object_poses` 里的 `class_name` + `position`（`base_link` 或 `map`）+ `track_id` 就是语义地图的输入。

## 5. 随机化范围（当前配置）

| 维度 | 范围 |
| --- | --- |
| 障碍物位置 | 5.2 m × 5.2 m 场地内随机，中心最小间距 0.55 m |
| 障碍物朝向 | 0–360° 随机 |
| 障碍物尺寸 | 每类各自范围（见 yaml `scale`），17 个物体 |
| 相机距离 / 高度 | 7.0–10.0 m / 0.35–1.5 m，始终朝场景中心 |
| 相机看向高度 | 0.30–0.90 m，注视点在场内抖动 |
| 灯光 | 平行光方向每帧随机 |
| 输出分辨率 | 640×480（与感知默认内参一致） |

这些范围先服务「流程跑通」；后续和航硕确认场景后，把真实障碍物的尺寸、密度、相机安装高度
换成本课题场景的分布即可。

## 6. 换成真实 USD 资产

在 `configs/dataset_config.yaml` 的 `obstacles` 里，把对应条目的 `primitive` 换成 `usd`：

```yaml
- class_name: shelf
  usd: ~/atongmu_assets/shelf.usd
  count: 2
  scale: [[1.0, 1.0, 1.0], [1.0, 1.0, 1.0]]
  z_offset: 0.0     # 资产原点离地高度，按资产实际底面调
```

注意：

- `class_name` 就是 YOLO 类别名，必须与语义地图/场景组的命名一致（上表 8 类）；
- 一个 USD 里的多网格会被当作一个整体随机摆放；
- 语义标签加在资产根 prim 上，Replicator 的 2D 框会自动带上整棵子树；
- `shelf` / `wall` 属于先验地图里的固定结构，最终应直接从场景 USD 中出，而不是随机撒；
- 资产很多时建议先各跑 50 帧，看 `labels/` 里的框是否正常，再放大 `num_frames`。

## 7. 已知限制与 4.5 实测记录

- 只在**平地 + 随机障碍物**上生成，没有墙壁/房间结构；等场景组给出仓库 USD 后把 `new_stage`
  换成加载场景，`shelf` / `wall` 由场景提供，randomizer 只负责动态障碍。
- primitive 只是近似形体（纸箱=立方体、托盘=扁立方体、人=圆柱），与真实资产的形状/纹理有差距；
  资产到位后替换，类别名保持不变。
- **相机半径必须大于场地对角半径**（2026-11-12 抽帧检查）：早期配置 7 m 场地 + 4.5 m 相机距离，
  相机会落进障碍物中间，画面被近处的立方体占满、框大量贴边。现在场地 5.2 m、相机 7–10 m，
  抽帧确认全景可见、框贴合。
- 材质/光照目前是默认灰白，没有颜色/纹理域随机化；对「流程跑通」够用，后续可加。
- Isaac Sim 4.5 的坑（脚本已处理）：
  - `rep.create.plane` 是 `UsdGeom.Plane`（XZ 竖直面），在 Z-up 场景里会变成一堵墙
    → 改用压扁立方体当地面，顶面在 z=0；
  - `rep.create.camera` 会给子 `Camera` prim 自动加 `(90,0,90)`，并可能被延迟执行的
    `modify.pose` 节点写回 → 预热后清掉子 prim 的 xformOpOrder，只保留父 Xform 的 look-at 矩阵；
  - 自定义 randomizer 的函数体**只在建图时执行一次**，不能用 Python `random` 做逐帧随机
    → 改为逐帧手动 `rep.orchestrator.step()` + 直接写 USD 变换，最小间距逻辑保持可用；
  - `SimulationApp.close()` 会直接结束进程，`data.yaml` 必须在 close 之前生成。
- Replicator 的 RGB 输出是 RGBA，已裁成 RGB 保存。
