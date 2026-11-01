# 0.40m研发基线v2证据

冻结入口[FROZEN_PROFILE.json](FROZEN_PROFILE.json)，历史试验候选快照[CANDIDATE_PROFILE.json](CANDIDATE_PROFILE.json)保持不改。后者的not_published状态描述冻结前，不覆盖本版冻结清单。使用说明见[参数文档](../../docs/JETBOT_NAV_PARAMETERS_20261101.md)。

- VALIDATION10/12/13/14/16为五例完整通过；13是预期无路停车，不计到达。
- VALIDATION15保留导航成功后的FAST-LIVO退出-6，process_clean=false；不能以runner0掩盖子进程异常。
- 11未进入仿真，视口启动崩溃及分类原文在完整原始归档。
- SHOWCASE10/14_AUDIT核验完整B/A事件及实际轨迹；C13_AUDIT核验目标空闲、不连通与原生中止。
- GEOMETRY为离线圆形实体对AABB的近似，非接触力认证；LOCALIZATION13保留无路用例的全程误差。
- SOFT040_RUNTIME_AUDIT来自controller实际服务参数与case YAML/CASE；每例参数/配置SHA和最小门控预测间距均保留。
- LOCAL_PUBLICATION_CHECKS记录发布源码与实际运行源码的差异、禁用/启用诊断的门控行为对照及冻结一致性检查。
- ARCHIVES_VERIFIED记录16个case的四个原始精选归档，包括各阶段失败，不将不同布局/路线合并成成功率统计。

所有数值按每例分别报告，原始AMCL采样误差、连续导航位姿误差、真实停车和估计容差分开；无全轨迹拟合，truth不进入在线控制。原始大文件见本版Release。
