# 团队工程工具

| 入口 | 用途 |
|---|---|
| `team/install_visualization.sh` | 已有 Humble/Nav2 上增装 RViz 与示例工具，不接设备 |
| `team/demo.sh` | domain 74 的无设备可视化示例，可加 `rviz:=false` |
| `team/doctor.sh` | demo / integration 两级只读环境诊断 |
| `team/verify_offline.sh` | 移植、消息、环境、安全及坐标 / 模型单元验证 |
| `team/install_ros_adapter.sh` | 第二阶段的 ROS 侧 LCM/schema 依赖 |
| `team/fetch_dimos.py` | 恢复锁定的上游提交与本地补丁，拒绝覆盖现有 checkout |
| `repo_audit.py` | 审查真实 Git 候选集，写本地清单；不上传 |
| `workspace_inventory.py` | 全工作区元数据索引，不跟随链接、不删除 |
| `export_team_bundle.py` | 生成源码与私有地图两个校验交付包，不覆盖既有包 |
| `prepare_team_sources.py` | 维护方从原件重建最小 vendor 快照，逐文件核验 |

使用顺序见 [快速开始](../docs/team/QUICKSTART.md)。部分工具需已有 ROS / 资产；不要把带硬件名字的历史脚本当作示例运行。

## 独立历史工具

`legacy/` 中保留早期 Windows USB-CAN 抓包、只读 CAN CSV 解析、RPLIDAR 串口抓取程序。它们依赖原有 Python 环境，不是 ROS2 统一入口。

只读解析示例，从工作区根目录运行：

```powershell
python tools/legacy/tracer_can_monitor.py data/protocol/final_motion.csv --every 100
```

`can_capture.py`、`rplidar_capture.py` 会访问真实设备，需现场手动使用；本次整理和离线验证不会启动采集。
