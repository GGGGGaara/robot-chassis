# 依赖边界

`tracer_vendor/` 是本机厂商目录中构建所需的 SDK 与 ROS 消息源码快照，逐文件 SHA256 在 `dependencies.lock.json`；未复制嵌套 `.git`、PDF、二进制或厂商系统备份。SDK 原许可证保留，示例仅参与编译，不会自动执行。

`patches/dimos-local.patch` 保存当前 DimOS 唯一已跟踪源码修改。原 `ku/dimos-main` 及其 Git 历史完整保留，父仓库不把它作为嵌入 Git 仓库提交。新机器执行 `python3 tools/team/fetch_dimos.py` 恢复锁定提交和补丁；已有目录会被拒绝覆盖。

`tracer_msgs_provenance.json` 另记录消息包 7 个文件与 AgileX 官方锁定提交的对照，`TRACER_ROS2_UPSTREAM_LICENSE` 保存该上游仓库根许可证。原 package.xml 的 BSD 声明保持不变，来源差异见根目录 `THIRD_PARTY.md`。

锁文件记录的是当前依赖来源与可复现源码，不代表所有系统包已固定，也不代表已自动配置 FAST-LIO2、Rust 原生构建和 Python 3.12 建图环境。先按 `docs/team/QUICKSTART.md` 完成不接设备的演示，再按第二阶段交接后端。
