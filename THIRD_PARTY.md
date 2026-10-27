# 第三方来源与公开边界

| 组件 | 本次处理 | 许可证 / 来源说明 |
|---|---|---|
| AgileX / Weston Robot ugv_sdk | `third_party/tracer_vendor/sdk` 构建源码快照，74 个 vendor 文件清单的一部分；原 LICENSE 保留 | Apache-2.0；来源提交、已有本地差异与每文件 hash 见锁文件 |
| tracer_msgs | 已核对 7 个文件与 AgileX 官方 tracer_ros2 的锁定提交一致（忽略换行差异），原作者与 package.xml 不改 | package.xml 原声明 BSD；官方仓库根许可证为 Apache-2.0，原文另存 `third_party/TRACER_ROS2_UPSTREAM_LICENSE`；保留此上游差异，不擅自重写包声明 |
| DimOS | 父仓库仅提交来源 / commit 和本地补丁，不复制整个上游及其 Git 元数据 | 原项目 Apache-2.0；下载后遵守上游与其子依赖许可证 |
| ROS 2 / Nav2 / RViz | 通过 Ubuntu ROS apt 源安装，不把安装树纳入本库 | 各组件保留其独立许可证，参考官方仓库 |
| 地图 / 实验模型 / 录制 / 答辩材料 / 厂商资料 | 本地保留，默认排除 Git；私有资产按需交接 | 可能含场地或个人资料，不能因放在工作区就认定适合公开 |

本次没有替整个工作区自动指定新开源许可证。仓库内既有 ROS 包的许可声明不自动覆盖所有厂商资料、地图和报告。可以先建立团队私有仓库；公开发布前按真实归属补齐项目许可及消息源码版权说明。

用户提供的目标仓库为公开的 [GGGGGaara/robot-chassis](https://github.com/GGGGGaara/robot-chassis)。消息源码的官方来源、对照结果及许可证 hash 记录在 `third_party/tracer_msgs_provenance.json`，对应提交 `832ea9a48fe725d47d766a099d358062ea2f0f31`。本次仅发布选定工程源码与团队文档，实验地图和私人资料不随之公开。
