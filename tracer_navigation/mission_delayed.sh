#!/usr/bin/env bash
# Operator-triggered once, in the foreground. No bypass of mission/site/obstacle checks.
set -eo pipefail
TASK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ $# != 1 || "${1:-}" != --execute ]]; then
  echo '用法：mission_delayed.sh --execute。未启动任务。'; exit 2
fi
source "$TASK_DIR/ros_env.sh"
bash "$TASK_DIR/commission.sh" check
"$TASK_DIR/.ros_venv/bin/python" - "$TASK_DIR" <<'PY'
import json,subprocess,sys
from pathlib import Path
root=Path(sys.argv[1])
state=json.loads(subprocess.check_output([sys.executable,str(root/'check_runtime.py')],text=True))
names=('map_server','controller_server','planner_server','behavior_server','bt_navigator','velocity_smoother','collision_monitor')
gate=state.get('safety',{})
if not all(state.get(n)=='active' for n in names):raise SystemExit('导航节点尚未全部 active。')
if gate.get('mode')!='hardware' or gate.get('armed'):raise SystemExit('需要未使能的实车模式。')
if not state.get('/tracer/ready'):raise SystemExit('底盘未就绪。')
if not all(state.get('/navigation/'+k+'_valid') for k in ('localization','perception')):raise SystemExit('定位或感知未就绪。')
if state.get('hardware_driver_publishers')!=1 or state.get('hardware_cmd_publishers')!=1:
    raise SystemExit('底盘驱动或速度发布者数量异常。')
PY
trap 'echo "倒计时取消，未启动导航。"; exit 130' INT TERM
echo '20 秒后进行检查，检查通过后将尝试一次真实 A→B 导航。'
echo '请退开停车及转向范围，保持路线空旷、线缆余量和物理急停可用。Ctrl+C 取消。'
for remaining in 20 15 10 5; do
  echo "倒计时：$remaining 秒"
  sleep 5
done
trap - INT TERM
echo '检查退开后的新鲜点云；仅在未使能且近处点云清空时重置运行缓存，并等待地图更新。'
"$TASK_DIR/.ros_venv/bin/python" "$TASK_DIR/reset_costmap_cache_disarmed.py"
echo '检查通过，交给原任务入口重新规划并检查；不提高速度，不跳过碰撞检查。'
exec "$TASK_DIR/.ros_venv/bin/python" "$TASK_DIR/mission.py" --execute
