# JetBot晚间A/B功能验证

case15与case20为原生多目标SUCCEEDED；case16/17/19原生失败、case18正常中止、case14未启动。不要将只读关闭小试验的空输入检查当传感器验收。详见docs/JETBOT_FASTLIVO_AB_20261023.md。

route20.png将实际全局代价地图快照与离线参考轨迹、估计导航反馈并列显示；参考轨迹不输入导航。实际参考路程11.857152m、终点偏差0.491749m，不是精密停靠验收。COSTMAP_EVIDENCE统计含膨胀的实际代价值，不是直接物理接触证据。

完整原始JSONL/日志/地图/冻结代码见本地及云端jetbot_evening_20261023_01.tar.gz（43,149,842B），SHA8903190d3d9e63eefed5dd15df610f91b81a1f0f44957e1c5c6ee6d9412cd6f8；574文件逐一验证。配合此前jetbot_migration_20261023_01.tar.gz恢复固定后端/依赖/原场景，仍需现有Isaac/ROS/资产镜像。本轮未推GitHub。
