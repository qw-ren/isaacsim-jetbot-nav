# 对象级语言关联与固定导航位姿 · 2026-11-19

后续云端微调、独立测试与失败记录见[JETBOT_OBJECT_GROUNDING_FINETUNE_20261119.md](JETBOT_OBJECT_GROUNDING_FINETUNE_20261119.md)。以下为首版从零训练的历史记录。

用户已确定采用对象级目标：模型选对象ID，ID绑定一个map导航位姿，无对象交互需求。
本地分支feature/jetbot-object-grounding-20261119，基于已发布5b3d115；本轮未推送、未启动云端仿真。

## 当前实现

`object_grounding.py`：字符嵌入+GRU语言编码器、对象几何/类别编码器、查询条件候选打分、无匹配/歧义两个额外输出，总计45923参数。不是预训练中文模型，没有正则语言解析回退；ID和候选顺序不作为输入。位置特征是相对估计机器人位置及与各排货架盒表面的距离。输入为对象表，不消费RGB或机器人truth。第一版把语言理解和对象选择联合训练，不强制显式槽位输出。

`goal_bindings.py`：独立对象ID→固定自由空间接近位姿，包含绑定时对象位置、尺寸、姿态。未知ID/错误场景/对象移动旋转变形拒绝。manager还核对规划图摘要和world→map注册信息，保留原有代价地图、原生路径与路径碰撞检查。对象被选中但无绑定时失败，不自动改选别的对象或生成新坐标。

`export_success_bindings.py`只导出已有native4且disarmed的成功接近点，不把物体中心当目标，不为未到访物体编造坐标。`CASE09_SUCCESS_BINDINGS.json`仅包含历史box_far的(-13,22.54,1.5708)，对应同一USD/map/注册；这不是所有对象已配齐，也不是可跨布局复用。

manager的可选`navigation.semantic.object_grounding`包含checkpoint、checkpoint_sha256、bindings（完整内联JSON）、min_confidence、min_margin；旧配置没有该键，继续旧流程且不导入torch。新模型仅允许dry_run；非预览任务返回MODEL_NOT_QUALIFIED，硬性限制不可用配置绕过。尚未完成ROS集成或实仿验收，不提供已可运行的新仿真配置。

## 第一轮训练，必须保留的失败

使用既有isaac_bin环境Torch2.7.0，未安装/修改任何仿真依赖。3000合成训练场景，400独立种子开发场景，500独立种子且不同表达模板测试场景；从零训练50epoch，按开发集准确率选checkpoint。每个场景纸箱/托盘/推车/标牌/四排货架，候选顺序随机。

开发集准确率93.25%；通过置信阈值的覆盖率78%，该子集准确率99.04%。然而未见表达测试仅50.2%，其中对象选择1/215正确，无匹配249/252，歧义1/33；置信阈值后覆盖率91%，准确率49.67%。说明模型对未见语言失效并可能高置信误判，不能部署。softmax分数不视为校准的可靠概率。结果见experiments/jetbot_object_grounding_20261119/PROTOTYPE01_EVALUATION.json。

训练标签由合成场景几何生成，是监督学习标签生成，不是在线规则解析。第一版最近/最远定义为机器人到物体中心欧氏距离，**与旧版最短可达Nav2路径排序不同**；未采原生路径标签，不能宣称保持原“最远可达”语义。货架旁边按合成盒表面1.5m生成。仅整数数字、有限句式，不支持完整自然语言、否定、任意多层关系。

24项纯Python测试通过（包含原20项和绑定的场景/位置/姿态/形状/未知ID/非有限值检查）。实际checkpoint候选置换、重命名ID、不认识字符拒绝、空候选不选物体、重复ID拒绝五项检查通过。没有新增ROS域52或实仿通过结论。

## 笔记本复现

在隔离worktree内：

```bash
export PYTHONPATH="$PWD/ros2_ws/src/jetbot_semantic_nav"
/home/liswnux/miniconda3/envs/isaac_bin/bin/python \
  ros2_ws/src/jetbot_semantic_nav/tools/train_object_grounding.py \
  --output /tmp/jetbot_grounding_UNIQUE_NEW_ID --samples 3000 --epochs 50
python3 -m unittest discover -s ros2_ws/src/jetbot_semantic_nav/test -v
```

原checkpoint和数据在原工程data/cloud_warehouse/jetbot_grounding_20261119/prototype_01，不进入Git；报告记录checkpoint摘要。离线`ground_object_query.py`可加载模型、对象JSON和绑定表，只输出预览JSON，不发布ROS。成功位姿导出工具接受--case和--output，目标输出文件必须尚不存在。

下一步优先采用有中文预训练能力的轻量编码器，并准备独立人工自然指令、否定/缺参照/歧义与对象组合测试。先把目标选择和拒绝能力验证好，再接只读ROS预览，最终才考虑启用Nav2；现有连续任务ODOM_TIMEOUT历史问题并未因语言模块改进而解决。
