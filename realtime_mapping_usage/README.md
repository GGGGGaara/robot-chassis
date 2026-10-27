# 建图后端与兼容工具

团队统一入口见 [快速开始](../docs/team/QUICKSTART.md)。新实验由 `tracer_bringup/system.launch.py` 管理后端，不与这里的旧多终端入口同时启动。

- `start_fixed_map_relocalization.sh --mapping`：连接真实 MID-360，固定图定位并持续建模；由 ROS 后端管理器调用。
- `start_mid360_raytrace.sh`：早期独立采集 / 建图入口，会连接真实设备。
- `start_fixed_map_path_preview.sh`：兼容网页看图 / 选点，监听 LCM，本身不发送底盘命令。
- Python 检查 / 诊断程序和历史中文说明保留，路径覆盖来自 `tracer_navigation/project_env.sh`，需要独立 DimOS 环境。

无设备时用根目录 `tools/team/demo.sh`；不要为看示例而运行真实后端。当前外部接口是 ROS 2，内部 LCM 仍存在，详细边界见 [环境合同](../docs/team/ENVIRONMENT.md)。
