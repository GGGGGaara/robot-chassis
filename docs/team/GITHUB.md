# 文件归属、GitHub 与交付方法

## 本机组织方式

本工作区既有活跃源码，也有约数万份历史 / 厂商 / 依赖文件。为了保留现有运行路径，本次以目录职责、索引和 Git 白名单整理，没有重新搬动运行依赖或删除唯一数据。

| 路径 | 处理方式 | 团队应使用它做什么 |
|---|---|---|
| `README.md`、`PROJECT_STATUS.md` | 更新为当前入口 | 不再把旧阶段待办当当前状态 |
| `docs/team/` | 进入 Git | 进度、快速开始、接口、验收、Git 操作和本轮验证摘要 |
| `config/` | 模板进入 Git，`local.env` 忽略 | 团队共用变量名称，个人填写本机路径 |
| `tracer_navigation/` | 源码 / 当前配置 / ROS 包进入 Git | 导航、安全门控、坐标任务、模型保存、RViz |
| `realtime_mapping_usage/` | py / sh / html / md 进入 Git | 后端脚本与兼容预览；本地生成报告留本地 |
| `third_party/tracer_vendor/` | 只复制必要源码，进入 Git | SDK、tracer_msgs 可离线重建；原 license 与 hash 保留 |
| `third_party/patches/`、锁文件 | 进入 Git | DimOS 提交及本地改动复现，原 checkout 不动 |
| `tools/` | 进入 Git | 安装、自检、示例、Git 候选审查、文件盘点、ZIP 交付 |
| `tracer_navigation/results/`、`model_sessions/`、`generated/`、根层 JSON / NPY / 日志 | 留在原位，忽略 | 保存实测原证据和运行产物，不混进源代码历史 |
| `ku/`、`TRACER2.0/` | 原件及嵌套 Git 全保留，父仓库忽略 | 查上游历史、厂商原始资料；不要 `git add -f` 嵌套仓库 |
| `reconstruction_20261017_113920/`、`data/` | 原地保留，忽略 | 当前参考图和历史实验；按私有资产交接 |
| `vendor/`、`archives/` | 分类维持，忽略 | 安装包、资料、系统备份和旧交付可追溯 |
| `docs/reports/`、`reference/`、`history/`、`organization/` | 保留本地，默认忽略 | 答辩 / 使用记录、手册、旧文档、全量文件索引 |

旧 `PROJECT_STATUS` 与两份 README 已复制到 `docs/history/*20261027*`。全量本地索引为 `docs/organization/文件清单_20261027.csv`，统计和跳过链接 / 错误记录为 `inventory_20261027.json`。它是元数据盘点，不表示逐行读完全部厂商源码，也不是数据备份。

## 父 Git 仓库状态

本地仓库接入用户提供的 [GGGGGaara/robot-chassis](https://github.com/GGGGGaara/robot-chassis)，远端为公开仓库，默认分支 `main`。原初始提交 `5190e56` 保留；本地团队源码基线接在该历史之后，不强制覆盖远端历史。`ku/dimos-main`、原 SDK 的内嵌 Git 不改动。提交使用现有 Git 身份，不代填新的身份。

`validation_20261027.json` 的 repository 部分记录的是首次离线准备时的状态；之后接入和发布结果以远端提交历史、Actions 和 `git status` 为准。

`.gitignore` 使用根层白名单，避免把地图、录制、文档私人材料、虚拟环境和整个上游仓库加入；`.gitattributes` 为 Bash 强制 LF。必要 vendor 快照保留原字节，避免换行转换破坏锁文件 hash。

先查看准备进入版本库的具体内容：

```bash
git status --short
python3 tools/repo_audit.py
git diff --cached --stat
git diff --cached -- tracer_navigation/common.py
```

第一次提交全是新增文件；后续即可看到增量。审查脚本只做大小 / 路径 / 特征模式和语法检查，不承诺识别所有私人内容、许可证问题或秘密。

## 后续提交与推送

用户已经创建仓库，本机 origin 为 `https://github.com/GGGGGaara/robot-chassis.git`，无需重复创建或执行 `remote add`。这是公开仓库，不推送私有地图、模型、录制、厂商安装包、答辩材料或虚拟环境。

后续变更在审查后提交；以下提交说明按实际工作修改：

```bash
git remote -v
git status --short
python3 tools/repo_audit.py
git add -- .
git diff --cached --stat
git commit -m "docs: update team handoff"
git push -u origin main
```

有并行团队工作时，先 fetch 并用分支 / PR 集成，不使用 force push 覆盖队友历史。通过 GitHub 的正常登录 / 凭据管理完成授权，不把 token 写入 URL、脚本或聊天。第三方版权来源见 `THIRD_PARTY.md`，不要给厂商代码改署名或擅自重写其许可。

## 交付给队友的两份包

`archives/deliveries/team_source_20261027.zip`：Git 候选源码、文档、SDK / 消息最小快照和 `SOURCE_MANIFEST.json`；没有 `.git` 历史、venv、运行日志、地图或完整建图原生环境。

`archives/deliveries/team_private_maps_20261027.zip`：只含当前二维 / 三维参考地图及 `PRIVATE_ASSETS_MANIFEST.json`，按场地私有数据处理，不上传公开仓库。它不是全部 12GB 工作区或全部实验数据的备份。

两个 ZIP 均有 `.zip.sha256`，并逐文件重新读取归档核验。它们保留第一次离线整理的快照，不因 Git 后续提交自动更新；当前源码优先从仓库克隆。M1 不需要私有地图包，M2 将私有包解压到同一根目录即可维持相对路径。

Linux 校验包：`sha256sum -c team_source_20261027.zip.sha256`；Windows 可用 `Get-FileHash -Algorithm SHA256` 与校验文件对照。私有地图包同理。确认来源后再解压，不用旧交付包覆盖当前源码。

后续用分支 / PR 继续工作，参考 `CONTRIBUTING.md`。当前 GitHub OAuth 令牌没有 `workflow` 权限，第一次包含活动工作流的推送被 GitHub 拒绝，未改变远端。为完成源码同步，活动路径 `.github/workflows/offline.yml` 保留本机并从 Git 排除；等价的未启用模板提交到 `docs/team/ci/offline.yml`。

GitHub Actions 当前**未启用、未执行**。有合适权限的维护者审查模板后，将其复制到 `.github/workflows/offline.yml`，删除相应忽略规则，再提交启用。模板仅运行离线测试，不连接硬件；本次没有扩展令牌权限。
