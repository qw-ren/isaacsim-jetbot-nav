# JetBot 导航仿真

本目录存放 JetBot 导航项目使用的 Isaac Sim 仿真场景。

## 仿真场景

`jetbot_sim.usd` 描述了运行在 `full_warehouse` 仓库环境中的 JetBot。场景包含差速驱动控制器，以及用于发布仿真时钟、相机、IMU、雷达、里程计和 TF 的 ROS 2 图。

场景引用了 Isaac Sim 4.5 资产库中的以下资产：

- `full_warehouse.usd`
- `jetbot.usd`
- Hesai XT-32 雷达资产（`XT-32.usd`）

这些资产是外部依赖，没有包含在本仓库中。USD 文件使用相对于场景文件的路径引用，指向 isaacsim_assets/ 资产库。若本机资产库位置不同，需要调整引用路径，才能正确打开场景。

## 打开场景

1. 在 Isaac Sim 中打开 `jetbot_sim.usd`。
2. 确认场景引用的 Isaac Sim 资产可用，并且路径能够正确解析。
3. 启用 ROS 2 Bridge 扩展，并选择与当前 Isaac Sim 版本兼容的 ROS 2 环境。
4. 开始仿真。

## ROS 2 接口

| 用途 | Topic | Frame ID |
| --- | --- | --- |
| 机器人速度控制 | `/cmd_vel` | — |
| 仿真时间 | `/clock` | — |
| RGB 相机 | `/rgb` | `sim_camera` |
| IMU | `/imu` | `sim_imu` |
| 雷达扫描 | `/scan` | `sim_lidar` |
| 里程计 | `/odom` | — |
| IMU、相机和雷达的静态 TF | `/tf_static` | — |
| 里程计 TF | `/tf` | — |

## 备注

- 场景使用 Z 轴向上的坐标系。
- 本仓库仅存放场景文件；Isaac Sim 资产库需要单独安装或配置。
