# 项目状态（2026-10-31 整层实时导航开发）

设备已断开，本次未启动传感器、CAN 驱动或运动任务。历史实测不代表当前在线状态。

## 本次新增：参考地图辅助、实时证据决定通行

- 已接入 ROS 2 实时净空栅格与最终速度门控。参考图用于定位和候选路线，不能把实时未知或新障碍改成可通行。
- 增加整层点云快照、米制 / 地面高度显式导入、map 坐标或相对坐标任务入口；整层入口候选路线限额 200m / 3600s，短程任务仍为 15m。
- 行进时继续保存三维模型；建模、定位参考与实时避障分开处理。新地图与软件需要重新验收。
- 当前为待现场验证的开发版本。49 项回归测试通过；真实 Humble/Nav2 的离线场景验证和连续到点仿真结果见 [本次验证记录](docs/team/floor_validation_20261031.json)。不能把离线通过视为整层实车通过。
- 录图、导入、启动、输入坐标与覆盖限制详见 [整层导航指南](docs/team/FLOOR_NAVIGATION.md)。单个倾斜雷达可能无法证明车身高度范围内的净空，缺观测时会停车，不能依赖旧地图补成空闲。

团队仓库：[GGGGGaara/robot-chassis](https://github.com/GGGGGaara/robot-chassis)。原始初始提交保留，后续源码以 main 分支为准；地图和实验资产不纳入公开仓库。

## 已完成

- ROS 2 统一启动：建图后端管理、重定位 / 避障点云适配、模型保存、Nav2、安全门控。
- 相对目标按任务开始时底盘中心 / 朝向解释，单位米；区分直线、绕障路线与实走距离。
- 10 月 23 日有统一框架 / 坐标 / 模型离线证据，10 月 24 日有重定位匹配和模型更新证据。
- 本轮补团队路径配置、RViz / 隔离合成示例、增量安装、自检、最小依赖快照与补丁、Git 协作结构和文件索引。CI 作为未启用模板保留，当前 OAuth 缺 workflow 权限。
- 本轮构建 / 消息烟测 / 移植测试结果见 [验证摘要](docs/team/validation_20261027.json)。

## 尚未完成或未宣布通过

- 真实机器人完整 A→B、现场米制尺度及新配置下连续避障验收。
- USB-CAN 历史 ENOBUFS、异常回显 / 反馈间断根因。部分零速样本通过，不能据此结案。
- 雷达安装精确标定；`mount.verified=false`，不自动恢复旧验收。
- 全面原生 ROS 2 建图后端重写、未知区域自主探索、所有系统依赖完整锁定。
- 队友机器的现场运行、图形驱动兼容性和 GitHub 云端 CI 实际执行。

## 队友当前阶段

队友已有 Ubuntu 22.04 + Humble + Nav2。先按 [快速开始](docs/team/QUICKSTART.md) 完成 M1 可视化，再按 [环境与接口](docs/team/ENVIRONMENT.md) 对接真实数据。不要重复写一套导航，也不要并行运行两套同名 Nav2 / TF。

进度 / 分工见 [ALIGNMENT](docs/team/ALIGNMENT.md)，实车通过条件见 [验收清单](docs/team/ACCEPTANCE.md)。本次改动使软件校验变化，旧实车验收不沿用。

## 历史证据位置（本地保留，默认不进 Git）

- `tracer_navigation/results/ros2_unification_20261023/`
- `tracer_navigation/results/coordinate_mapping_20261023/`
- `tracer_navigation/results/alignment_check_20261024_163649/`
- `tracer_navigation/results/small_range_test_20261024/底盘通信复测记录.md`
- `tracer_navigation/results/team_alignment_20261027/`

旧状态和旧 README 已保存在 `docs/history/*20261027*`；旧文档中的“还没 ROS”“5m 上限”等阶段说明不能作为当前操作依据。当前线速度仍为 0.10m/s、角速度 0.15rad/s、近车保护约 0.70m；短程规划总长 15m，显式整层入口 200m。上限不是已实测成绩。
