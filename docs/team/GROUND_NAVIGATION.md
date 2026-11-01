# 地面导航模式（2026-11-01）

根据现场测试范围增加可选 `realtime_navigation.clearance_mode: ground`。未配置时仍为 `full_height`。

```yaml
realtime_navigation:
  clearance_mode: ground
  ground_clearance_height_m: 0.3
```

地面模式只把“可通行所需射线证据”限定为地面到 0.3m；仍需新鲜地面支撑、三个 0.1m 高度带、整车平面扫掠范围、正确定位与数据新鲜度。证据不足仍未知，过期不会自动变空闲。

**实际观测到的障碍继续按整车 1.2m 高度处理**，包括观测到的悬空障碍。没有降低物理车高、缩小碰撞外形或删除近车停车区、Collision Monitor、速度上限、CAN 反馈看门狗与现场验收。

这是一种限定场景的平面导航配置，不证明未观测上方安全。悬空桌边、货架等可能位于盲区；测试路线须由现场确认具有整车所需净空。模式切换须重启数据链路、重新核对定位；参数和软件变化使旧实车验收失效，默认不驱动车辆。

`/navigation/live_map_status` 报告 `clearance_mode`、`required_clearance_height_m`、`physical_robot_height_m`、`upper_clearance_required`。速度门控状态也报告模式，避免把地面模式解释成全高度净空验证。

回归测试：`python -m unittest test_live_navigation test_floor_import test_safety test_coordinates_and_models -q`，2026-11-01 共 55 项通过。新增六项覆盖近地放行、已观测上部障碍优先、只有地面点不放行、证据过期、默认全高度模式及非法参数拒绝。离线测试不代替实车停车或动态避障验收。

现场临时外参保留在本机 `robot.yaml`，不把估计角度当作团队通用安装标定。完整参考地图 / 实时通行职责见 [整层导航说明](FLOOR_NAVIGATION.md)。
