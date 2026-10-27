# 队友快速开始：已有 Humble + Nav2

在 Ubuntu 22.04 的 **Bash** 执行。项目仓库是 [GGGGGaara/robot-chassis](https://github.com/GGGGGaara/robot-chassis)。可以克隆到自己的目录；以下都从仓库根目录执行。不要复制本机 `.ros_venv`、`ws/install` 或另一用户的 `/home` 目录。

## 第一阶段：补工具，跑无设备演示

```bash
git clone https://github.com/GGGGGaara/robot-chassis.git ~/tracer-team
cd ~/tracer-team
bash tools/team/install_visualization.sh
bash tracer_navigation/build_ros2.sh
bash tools/team/demo.sh
```

安装脚本在既有 ROS apt 源基础上增装 RViz2、默认插件、Nav2 RViz 插件、rqt_graph、tf2_tools、官方 talker/listener 示例及编译 / Python 工具，不重装 Nav2，不修改 apt 源，也不自动运行设备。需要 sudo 和网络。可视化需要图形桌面 / WSLg / 合适的远程图形会话。

应看到约 6m × 6m 的合成房间地图、墙面三维点云、红色模拟障碍点、绿色路径和移动的模拟 TF / 里程计。画面里的运动是数值动画，车辆不会动。默认不添加 RViz 发目标工具，避免把看图误认为已具备实车执行能力。

示例固定在 **localhost、ROS_DOMAIN_ID=74**；真实主线默认 42，历史离线测试用 43。不要让队友另一个 Nav2 / 定位栈也在示例域启动同名话题。脚本在 WSL 镜像网络中可能请求一次 sudo 修正仅供 domain 74 的 UDP 回环路由，不接触 CAN。

没有桌面时先跑消息测试；不应因无 GUI 宣称图形已验证：

```bash
export ROS_DOMAIN_ID=74 ROS_LOCALHOST_ONLY=1
source tracer_navigation/ros_env.sh
python tools/team/test_portability.py
python tools/team/smoke_demo.py
```

烟测自行启动 / 停止隔离演示，并验证真实 ROS 消息、米制栅格、XYZ 点云、TF、路径、里程计和不存在速度 / 底盘发布者。它不是完整 Nav2 运动仿真。输出位于 `tracer_navigation/results/team_alignment_20261027/`；该名称表示本次交接测试入口，重复运行会刷新本地结果。

检查安装：

```bash
bash tools/team/doctor.sh --profile demo
```

另一个终端查看示例数据，先执行同样的域与 source：

```bash
export ROS_DOMAIN_ID=74 ROS_LOCALHOST_ONLY=1
source tracer_navigation/ros_env.sh
ros2 topic list
ros2 topic echo /team/demo_status --once
ros2 run rqt_graph rqt_graph
ros2 run tf2_tools view_frames
```

## 第二阶段：接入项目真实数据链

第一阶段成功后，先读 [环境与接口](ENVIRONMENT.md)。本阶段需要另一份私有地图包和 DimOS / FAST-LIO2 原生环境，不能靠 Nav2 + RViz 安装替代。

```bash
cp config/team.env.example config/local.env
# 按本机路径编辑 local.env；不提交这个文件。
bash tools/team/install_ros_adapter.sh
python3 tools/team/fetch_dimos.py  # 新机器且 ku/dimos-main 不存在时才运行
# 解压私有资产包到仓库根目录，核对资产清单 SHA256。
# 按 ENVIRONMENT.md 完成 Python 3.12 与两个原生后端构建。
bash tracer_navigation/build.sh
bash tools/team/doctor.sh --profile integration
```

`fetch_dimos.py` 只恢复源码与补丁，不会自动创建 Python 3.12 环境或编译原生模块。不能把 `doctor --profile demo` 通过当作第二阶段完成。

设备断开时不要运行 `start_ros2.sh` 默认后端；它会尝试连接雷达。设备接好、网络准备并明确进入现场流程后，统一预览才使用：

```bash
bash tracer_navigation/start_ros2.sh
```

真实数据可视化另开终端（只看数据，不启动第二套导航）：

```bash
source tracer_navigation/ros_env.sh
ros2 launch tracer_bringup visualization.launch.py project_root:="$TRACER_PROJECT_ROOT"
```

默认 `demo:=false`，只启动 RViz。不要把 `demo.sh` 的合成模型与真实场景混在同一域。

## 常见问题

| 现象 | 优先检查 |
|---|---|
| `tracer_bringup` 找不到 | `build_ros2.sh` 是否成功；当前终端是否 source；`TRACER_RUNTIME_ROOT` 是否一致 |
| 找不到 `.ros_venv/bin/python` | 在本机运行安装入口创建，不要复制 Windows / 别人机器的 venv |
| RViz 黑屏或 `DISPLAY` / OpenGL 错误 | 图形桌面 / WSLg / 显卡驱动；先用 `smoke_demo.py` 区分 ROS 数据与图形问题 |
| RViz 有节点但没数据 | 两终端域一致；Fixed Frame=map；地图 Transient Local；实时障碍 Best Effort |
| integration 找不到地图 / LCM 类型 | 解压私有资产、核对 local.env，运行 adapter 安装脚本 |
| 模型有了但导航不就绪 | 查看定位 / 感知有效话题和 TF；“后端进程 running”不等于重定位成功 |
| RViz 显示两份车体或跳动 | 两个 TF 发布者或两套 Nav2，停止重复实例 |

参考：[ROS Humble 官方文档](https://docs.ros.org/en/humble/)、[ROS RViz 包索引](https://index.ros.org/r/rviz/)、[Nav2 官方文档](https://docs.nav2.org/)。本项目示例用于补齐可视化，不能替代官方导航仿真或本项目实车验收。
