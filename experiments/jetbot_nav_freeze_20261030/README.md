# 2026-10-30 导航/定位参数实验记录

解释与命令见 [参数说明](../../docs/JETBOT_NAV_PARAMETERS_20261030.md)，精度定义见 [口径说明](../../docs/JETBOT_ACCURACY_DEFINITIONS_20261030.md)。本目录为小型可审查证据，原始传感器/控制/源码日志另作归档，不加入Git大文件。

冻结入口`FROZEN_PROFILE.json`；`ARCHIVE_VERIFIED.json`记录15个case、100,778,033B归档、2038原始文件及外层/逐文件SHA核验。下载见[预发布](https://github.com/qw-ren/isaacsim-jetbot-nav/releases/tag/jetbot-nav-baseline-20261030)。`RUNTIME92_SOURCE_MATCH.json`记录最终真实运行和待发布六个改动运行文件的字节一致性；包含新图map_server和最终速度适配。

- `VALIDATION*.json`：原生状态、输入审计、安全门、runner/子进程收尾，以及成功任务的真实停车和全程定位；native状态6在C中是预期无路，在其他任务中是失败。`process_clean`同时要求runner和子进程0，不能把失败runner1、子进程0说成整体成功。
- `LOCALIZATION*FAILED.json`、`LOCALIZATION81_INCOMPLETE.json`：失败/未完成路线的定位描述，不能与完整路线混合排名。
- `LOCALIZATION90_NOPATH.json`：C终止前的全局定位误差；不计到达精度。
- `SHOWCASE_AUDIT.json`、后续重复BA审计：B/A触发、前后计划、实际换通道/绕过与离线近似间距；不以仅原生成功代替实际路径检查。
- `C90_ASSET_TRIAL_AUDIT.json`：目标空闲、通路不连通、原生ABORTED且退出归零的独立验证。
- `TERMINAL87.json`、`jetbot_terminal87_evidence.py`：87末5秒原生速度与离线真实速度的不一致，含坐标近似局限、输入SHA。原始速度仍保留。
- `TWIST88.json`：88投影速度与完整原始速度的对应。88源码尚未恢复投影前raw完整校验，但所有原始样本有限、最大norm .866<3m/s；89及后续源码保留原raw保护，局部边界检查见`LOCAL_CHECKS.json`。
- `AMCL_RUNTIME80.yaml`、`LIKELIHOOD*.json`：旧模型参数与早期扫描的离线评分描述；不冒充当前prob模型的评分记录，不做位姿优化或时间/轨迹拟合。CLI原文按字节保留（含末尾空行），不为源码空白检查改写观测证据。
- `LOCALIZATION_MAP_COMPARISON.png/pdf`：原保守规划轮廓与静态表面定位图的示意。图创建时标注candidate/unvalidated，后续真实回归状态以独立JSON和参数说明为准。
- `plot_accuracy.py`、`ACCURACY_TRIALS.csv`、`ACCURACY_COMPARISON.png/pdf`：按每次实验分开画全局定位和终点，图来自小型汇总，输入SHA见`PLOT_PROVENANCE.json`。不将不同路线合并成精度或成功率统计。

AMCL指标使用所记录发布样本自身的scan时间戳，约5Hz探针可能漏更新；导航全局位姿另以.05s网格统计，并保留按行程加权和运动期指标。角度包角，不做全轨迹拟合。truth仅离线评价，从未接入定位/控制。单配置少量重复不能保证所有路线或每时刻3cm。
