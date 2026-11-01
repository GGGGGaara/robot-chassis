# TRACER 2.0：ROS 2 导航与三维建模

团队基线：2026-10-27。已有 Ubuntu 22.04 + ROS 2 Humble + Nav2 的队友，从 [团队接手入口](docs/team/README.md) 开始；先补 RViz / 示例与工程环境，再接真实建图数据。

2026-10-31 新增开发版：[整层实时感知导航](docs/team/FLOOR_NAVIGATION.md)。参考地图只辅助候选规划，真实点云的地面 / 全车高度净空、车身扫掠与新鲜度共同决定能否行驶；支持整层录图保存、导入和独立的长距离任务入口。未完成整层实车验收。

## 当前能力

MID-360 → DimOS / FAST-LIO2 / 光追建图与重定位 → ROS 2 适配 → Nav2 → Collision Monitor → 独立门控 → CAN 底盘。对外接口统一 ROS 2，后端内部仍使用 LCM。支持以当前底盘为原点输入前进 / 左移米数，默认只规划，并持续保存三维 CDR / PLY 模型。

[进度与分工](docs/team/ALIGNMENT.md) 区分已实现、离线验证和待实车验收。真实 A→B、尺量距离和 CAN 长期稳定性未宣布通过；本次设备断开，只做离线工程与交接。

## 队友先跑这个

项目仓库：[GGGGGaara/robot-chassis](https://github.com/GGGGGaara/robot-chassis)。队友在 Ubuntu Bash 中执行：

```bash
git clone https://github.com/GGGGGaara/robot-chassis.git
cd robot-chassis
bash tools/team/install_visualization.sh
bash tracer_navigation/build_ros2.sh
bash tools/team/demo.sh
```

这是隔离 domain 74 的合成可视化，不接雷达或底盘，不执行 Nav2 运动。无桌面 / 第二阶段接入见 [快速开始](docs/team/QUICKSTART.md)。真实传感器入口 `start_ros2.sh` 会连接设备，不能当作离线示例。

## 找文件

| 目录 | 职责 |
|---|---|
| `docs/team/` | 进度、快速开始、环境接口、验收、GitHub 和验证摘要 |
| `config/` | 共享模板，个人覆盖文件 `local.env` 忽略 |
| `tracer_navigation/` | 当前导航源码、ROS 包、配置和 RViz；运行产物留本地 |
| `realtime_mapping_usage/` | 建图后端入口、兼容预览、重定位检查 |
| `third_party/` | 构建必要 SDK / 消息快照、DimOS 来源锁定和本地补丁 |
| `tools/` | 队友安装 / 示例 / 自检、文件盘点、Git 审查和打包 |
| `ku/`、`TRACER2.0/` | 原上游 / 原厂资料保留，不作为嵌入 Git 仓库提交 |
| `reconstruction_20261017_113920/`、`data/` | 当前参考地图和历史实验，私有资产 |
| `vendor/`、`archives/` | 厂商工具、资料、历史交付与备份，保留本地 |
| `docs/organization/`、`history/`、`reports/` | 全量本地索引、旧入口备份、项目报告，默认不发布 |

文件没有因 Git 忽略而删除。完整策略与首次提交 / 推送步骤见 [GitHub 指南](docs/team/GITHUB.md)；协作见 [CONTRIBUTING](CONTRIBUTING.md)，第三方归属见 [THIRD_PARTY](THIRD_PARTY.md)。

原有 ROS 话题接口仍见 [架构说明](tracer_navigation/ROS2统一架构与队友接口.md)，任务定义见 [坐标导航与建模指南](tracer_navigation/坐标导航与实时建模_使用指南.md)。
