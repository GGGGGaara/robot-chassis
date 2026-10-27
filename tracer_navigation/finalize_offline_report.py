"""Assemble evidence from completed offline tests; never controls hardware."""
import json
from datetime import datetime
from zoneinfo import ZoneInfo
from common import ROOT,driver_fingerprint,software_fingerprint

files={
    'safety':'test_safety_report.json','driver':'vcan_driver_report.json',
    'navigation':'integration_report.json','mission':'mission_offline_report.json',
    'closed_loop':'closed_loop_vcan_report.json','audit':'offline_check_report.json',
    'can_feedback':'can_feedback_test_report.json',
    'recorded_relocalization':'recorded_relocalization_report.json',
}
data={name:json.loads((ROOT/path).read_text()) for name,path in files.items()}
driver_sha=driver_fingerprint();software_sha=software_fingerprint()
checks={name:bool(report.get('passed')) for name,report in data.items()}
checks['no_physical_can']=not data['audit']['physical_can_present']
checks['driver_build_matches']=data['driver'].get('driver_sha256')==data['closed_loop'].get('driver_sha256')==driver_sha
checks['software_matches']=data['driver'].get('software_sha256')==data['closed_loop'].get('software_sha256')==data['audit'].get('software_sha256')==software_sha
checks['site_acceptance_revoked']=data['audit']['hardware_acceptance']!='accepted'
report={'timestamp':datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),'offline_only':True,
        'physical_navigation_verified':False,'checks':checks,'passed':all(checks.values()),
        'evidence':files,'driver_sha256':driver_sha,'software_sha256':software_sha,
        'closed_loop_position_error_m':data['closed_loop']['position_error_m']}
(ROOT/'results/offline_completion_20261019.json').write_text(json.dumps(report,indent=2))
c=data['closed_loop'];soak=c['soak']
rows=[('安全逻辑',data['safety']['tests'],files['safety']),
      ('虚拟 CAN 驱动与故障注入',len(data['driver']['checks']),files['driver']),
      ('实际 Nav2 预览和动态避障',len(data['navigation']['checks']),files['navigation']),
      ('任务入口规划和拒绝无硬件执行',len(data['mission']['checks']),files['mission']),
      ('虚拟 CAN 的 A → B 闭环',len(c['checks']),files['closed_loop']),
      ('只读 CAN 准备检查',len(data['can_feedback']['checks']),files['can_feedback']),
      ('历史点云人工姿态扰动重定位',1,files['recorded_relocalization'])]
table='\n'.join(f'| {label} | {count} 项通过 | [{path}]({path}) |' for label,count,path in rows)
text=f'''# 2026-10-19 导航离线收尾

设备断开后完成。本报告不表示实车 A → B 或现场 USB-CAN 稳定性已经通过。当前旧现场验收已撤销，下次连接后必须按 [最新操作卡](下次接线_自主导航.md) 复验。

## 已定位的现象

现场控制模式为 CAN、底盘未报故障，但出现状态反馈间隔过长和发送 `ENOBUFS`。独立内核时间戳采样显示系统状态帧约 5 Hz、轮速帧约 50 Hz；完整导航下的旧驱动零速度测试出现 33 次未就绪，且捕获到发送 errno 105。换接口及强制绑定未证明解决问题，不能归因于单一 USB 过滤驱动。

离线还发现 WSL 镜像模式将 `127.0.0.1` 的 UDP 路由至 Windows 虚拟接口，造成 ROS 节点能发现、跨进程数据不通。修正本项目端口路由后，Python/C++ 消息收发恢复。

## 本版修改

- CAN 发送间隔至少 50 ms，避免恢复调度时突发写入；从运动切换为零速时立即发送。
- 系统状态帧独立使用 0.6 s 超时；轮速仍为 0.3 s、命令仍为 0.25 s。状态帧容许间隔有所调整，实车效果需要复验。
- 接收使用内核时间戳，旧队列帧不能被当成新反馈；发送失败会报告 errno，恢复后先发零速度。
- 驱动具体原因加入状态检查和任务结果；故障仍解除使能，不自动恢复行驶。
- ROS 使用本机 UDP，并为 domain 42/43 设置局部回环路由；启动脚本自动检查。
- 初次使能时停车区内存在近处点云会拒绝启动；新增 30 秒硬限制零速度的连接检查入口。

## 最终测试

| 内容 | 结果 | 证据 |
|---|---|---|
{table}

虚拟 CAN 闭环采用实际 Nav2、实际底盘驱动和理想运动学模型，系统状态按 5 Hz 注入。A={c['start']}，B={c['goal']}，到点约 {c['duration_s']:.1f} s，位置误差 {c['position_error_m']:.3f} m。模拟障碍物出现后停车，移除后恢复，到点后保持零输出；额外 {soak['duration_s']:.0f} 秒内完成 {soak['successful_plans']} 次规划，正常退出。

![离线导航结果](results/offline_navigation_results.png)

依赖、Python/Bash/XML 语法、ROS 回环路由和原始地图校验通过；地图未修改，设备断开期间没有物理 CAN 接口。汇总校验结果：`{report['passed']}`。

## 下次仍需完成

先在完整导航预览运行时，通过 `check_connection_zero.sh` 的 30 秒零速度检查；再复验真实前进、左转及停车，重新验收，然后执行空场短距离 A → B。若仍出现 `ENOBUFS`、USB 回显错误或反馈超时，保留新诊断报告，继续检查适配器、固件及 USB/WSL 链路；不要继续增加队列或放宽超时。

完整步骤见 [10 月 20 日实车操作卡](明日实车操作_20261020.md)。本轮现场原始诊断保存在 `results/现场诊断_20261019/` 和 `results/reconnect_20261019/`。

## 晚间实验后的离线补充

晚间曾完成现场地图重合确认，之后 WSL 发行版退出、服务消失。USB-CAN 重连后最后一次检查为接收 0 帧，切到正常 CAN 应答模式的 5 秒检查仍未收到报文。此问题尚未被证实修复；明天先恢复两种底盘反馈，再按操作卡进行稳定性和运动测试。

`prepare_wsl.sh` 已改为检查真实反馈：缺少系统状态、缺少轮速、反馈过期或空监听均失败。六项虚拟 CAN 场景验证通过。`collect_support.sh` 可保存故障时状态和已有报告的原始修改时间；零速度报告新增开始时间。明天保留前台 Ubuntu 终端，重启服务后重新定位和核对地图，旧 JSON 不算本次成功。

驱动 SHA256：`{driver_sha}`  
软件 SHA256：`{software_sha}`
'''
(ROOT/'离线收尾_20261019.md').write_text(text,encoding='utf-8')
print(json.dumps(report,indent=2))
raise SystemExit(0 if report['passed'] else 1)
