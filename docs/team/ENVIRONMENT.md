# 环境、依赖与接口合同

## 2026-10-31 实时楼层配置更新

启用 `robot.yaml/realtime_navigation.enabled` 后：map_server 的保存图改发 `/navigation/reference_map`；`/map` 是实时观测与参考代价融合的候选图；`/navigation/live_map` 是 odom 下的实时通行证据，局部控制不加载历史静态图。原始 `/lidar/points` 的未裁剪射线用于全高度净空，最终门控拒绝过期和未知区域。TF 所有权、原有速度 / CAN 安全链不变。

下方传统接口表中的 `/map` 名称保留，但新配置语义以上述内容为准。新增入口、录图 / 导入、整层任务和实测限制见 [FLOOR_NAVIGATION](FLOOR_NAVIGATION.md)。

## 三层环境

| 层 | 要求 | 复用策略 |
|---|---|---|
| 队友已有底座 | Ubuntu 22.04、ROS 2 Humble、能跑的 Nav2 | 保留；增装可视化 / 示例即可开始 M1 |
| ROS 工程 | 系统 Python 3.10 + `.ros_venv --system-site-packages`；ament / colcon | 使用 apt 提供的 rclpy，不把 Python 3.12 包路径整体塞进 PYTHONPATH |
| 建图后端 | 当前验证环境为 Python 3.12，DimOS 锁定提交，FAST-LIO2 和 Rust 光追程序 | 独立 venv / 原生构建；通过 LCM 在内部传数据，对外由桥接输出 ROS |

`tools/team/install_ros_adapter.sh` 安装 ROS 侧 `lcm==1.5.3` 与 `dimos_lcm==0.1.4` 的纯 Python schema（后者 `--no-deps`，不拉入另一套 LCM native fork）。这些是已观察版本，不宣称 apt、所有 Python 间接依赖或原生系统都已完整锁定。

## 路径配置

源码按 checkout 相对路径定位；Linux 编译目录默认 `$HOME/tracer_nav_runtime`，避免 ROS Humble 某些生成工具处理中文 / 空格路径的限制。`project_env.sh` 读取可选 `config/local.env`；默认无需写 `.bashrc`。

| 环境变量 | 用途 |
|---|---|
| `TRACER_RUNTIME_ROOT` | 本用户 ASCII 路径下的 SDK / ROS 构建安装目录 |
| `TRACER_VENDOR_ROOT` | 默认 `third_party/tracer_vendor`；本机保留原厂路径作为兼容 fallback |
| `TRACER_DIMOS_ROOT` | 默认仓库 `ku/dimos-main`；父 Git 排除其嵌套仓库 |
| `TRACER_MAPPING_VENV` | 默认 `$HOME/venvs/dimos-mapping`，Python 3.12 后端 |
| `TRACER_STATIC_MAP_LCM` | 覆盖二维参考 `global_costmap.lcm` 路径 |
| `TRACER_PREMAP` / `DIMOS_PREMAP` | 后端三维重定位参考 `global_map.pc2.lcm` |
| `TRACER_LCM_TYPES_DIR` | 可选纯 Python schema 路径；默认先找 ROS venv 安装包，再找后端 venv |
| `ROS_DOMAIN_ID` | 主线默认 42；历史测试 43；本次示例强制 74 |

修改源码、路径配置与生成配置会影响实车校验；本次不迁移旧 `site_acceptance.json`，不自动标记外参 verified。

## 真实后端的第二阶段交接

1. `fetch_dimos.py` 从锁文件恢复 `2a6e7465b768e29a0dc4d0722ec096e50e1b168c`，应用本地 `map.py` 补丁；不会碰现有用户 checkout。
2. 在独立 Python 3.12 venv 安装该提交的映射依赖。当前本机版本记录在 `third_party/dependencies.lock.json`；不要安装与该提交无关的最新 DimOS 来替代。上游安装入口见该提交 `docs/quickstart.md`、`docs/requirements.md`、`docs/usage/native_modules.md`。
3. 依该提交的 Nix / Rust / 原生安装文档构建并检查下列程序。Python 安装成功不等于这两个程序存在；不能把本机 `result` 软链接或 venv 直接复制给队友当安装包。
4. 取得私有地图 ZIP，解压到源码根，核验资产清单。`global_map.pc2.lcm` 是重定位点云，`global_costmap.lcm` 是二维规划地图，不能互换。
5. 完成 `install_ros_adapter.sh`、`build.sh` 与 `doctor --profile integration`。真实传感器连接和控制网络准备属于现场流程。

| 程序 / 文件 | 当前后端期望位置（相对 DimOS 根） |
|---|---|
| FAST-LIO2 | `dimos/hardware/sensors/lidar/fastlio2/cpp/result/bin/fastlio2_native` |
| 光追 | `target/release/voxel_ray_tracing` |

此处明确保留后端安装差距，没有把整套原生环境伪装成已自动化完成。当前队友的 M1 可视化和 ROS 工程工作不依赖它。`doctor` 的文件存在检查也不能证明原生二进制 ABI / 依赖库可运行；后续还需 `ldd` 与离线 CLI 检查。

## 不变的接口合同

完整接口表见 `tracer_navigation/ROS2统一架构与队友接口.md`，以如下关键项对齐：

| 接口 | 类型 / 坐标 | QoS / 规则 |
|---|---|---|
| `/map` | OccupancyGrid，map，米 / 栅格 | 地图显示使用 Reliable + Transient Local |
| `/lio/odom` | Odometry，odom → base_link，米 / 弧度 | lio_bridge 负责输出；车体中心 |
| `/points_navigation` | PointCloud2，base_link | SensorData / Best Effort；已自体过滤，保留采集时间 |
| `/points_navigation_sensor` | PointCloud2，mid360_link | 同一批观测、真实射线起点，供障碍层清除 |
| `/mapping/global_points`、`local_points` | PointCloud2，map，米 | Reliable，depth=1，Volatile；用于模型显示 / 保存 |
| `/compute_path_to_pose` | Nav2 ComputePathToPose action | 只规划；输入 map 坐标 |
| `/navigate_to_pose` | Nav2 NavigateToPose action | 实车只走工程任务入口与验收流程 |
| `/navigation/safety_status` | String JSON | mode / armed / reason / ages，区分未使能与故障锁定 |

TF **仅一位所有者**：`map → odom → base_link → mid360_link` 由 lio_bridge 输出。队友已有 AMCL、robot_localization 或底盘 TF 如发布同一条边，必须二选一，不能同时运行。现有 `tracer_bringup` 已包含 Nav2，不能再启动队友的整套 Nav2 与其争用同名节点 / action。

控制链为 `/cmd_vel_nav → /cmd_vel_smoothed → /cmd_vel_collision_checked → 独立门控 → /tracer/cmd_vel → 底盘`。不要从示例节点、RViz 或自己的脚本直接发布硬件最终速度。远处障碍用于规划与行进中判断；贴车碰撞区和过期停车仍保留。

相对目标：`bash tracer_navigation/navigate_relative.sh 1.5 0.5` 表示任务发起时车头前方 1.5m、左侧 0.5m；默认只规划。侧移坐标是差速车的目标位置，不表示底盘能横向平移。

## 图形与跨机边界

当前 FastDDS 配置故意限制本机回环；两台电脑即使同 domain，也不能据此认定互通。首轮优先在同一台 Ubuntu 中 source 同一 overlay 联调，或通过网页看图。跨机 DDS、时间同步、网卡、发现和安全策略另做独立配置变更，不在本次自动放开网络接口。
