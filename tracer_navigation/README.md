# 当前导航工程入口

2026-10-31：新增 [整层实时导航开发版](../docs/team/FLOOR_NAVIGATION.md)。`floor_navigate.sh` 只规划为默认；实时地图节点与最终门控以新观测决定通行，参考图不再作为局部安全依据。旧离线闭环不能代替这一配置的现场验收。

团队交接优先阅读 [快速开始](../docs/team/QUICKSTART.md)、[进度](../docs/team/ALIGNMENT.md) 和 [环境接口](../docs/team/ENVIRONMENT.md)。历史版本说明已备份到本机 `docs/history/tracer_navigation_README_20261027.md`。

- `start_ros2.sh`：真实传感器统一预览，默认不接底盘；会尝试连接雷达，设备断开时不用它跑示例。
- `tools/team/demo.sh`（仓库根下）：独立合成 RViz 示例，domain 74，无硬件、无运动命令。
- `navigate_relative.sh 前进米数 左移米数`：基于任务开始时车体中心的相对目标，默认只规划。
- `robot.yaml`：车体 / 外参 / 限速 / 看门狗配置；地图为 checkout 相对路径。
- `project_env.sh` + `config/local.env`：本机路径；`ros_env.sh`：Python 3.10 / ROS overlay。
- `build_ros2.sh`：仅编译 bringup，适合第一阶段示例；`build.sh`：编译 SDK、消息、底盘驱动和 bringup，生成实际 Nav2 配置，需真实参考地图与 LCM schema。
- `rviz/team.rviz`：固定图、三维模型、实时障碍、路径与里程计显示。
- `results/`、`model_sessions/`、`generated/`：本地实验与生成物，不进源码 Git。

当前主线对外 ROS 2，内部建图兼容 DimOS / LCM。地图保持只读，三维模型单独更新；完整 ROS 话题表见 [原架构接口](ROS2统一架构与队友接口.md)。坐标 / 文件格式见 [任务指南](坐标导航与实时建模_使用指南.md)。

实车需要重新定位、尺度核验、反馈稳定性与现场验收。旧零速检查、模拟闭环和地图重合不能替代新版本实车 A→B。当前外参仍 `verified=false`；保留 0.10m/s、0.15rad/s、15m 路线限额与约 0.70m 近车保护。
