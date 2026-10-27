#!/usr/bin/env bash
# Operator entry: full navigation preview must already be running, disarmed.
set -eo pipefail
TASK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$TASK_DIR/ros_env.sh"
if pgrep -f '/tracer_safe_base( |$)' >/dev/null; then echo '已有底盘驱动，请先退出实车导航。'; exit 1; fi
echo '连接检查：驱动硬限制为零速度，持续 30 秒；不会执行前进或转向。'
"$TASK_DIR/.ros_venv/bin/python" - "$TASK_DIR" <<'PY'
import json,subprocess,sys
from pathlib import Path
root=Path(sys.argv[1])
state=json.loads(subprocess.check_output([sys.executable,str(root/'check_runtime.py')],text=True))
names=('map_server','controller_server','planner_server','behavior_server','bt_navigator','velocity_smoother','collision_monitor')
if not all(state.get(n)=='active' for n in names):raise SystemExit('先等待全部导航节点 active。')
gate=state.get('safety',{})
if gate.get('mode')!='dry-run' or gate.get('armed'):raise SystemExit('需要未使能的导航预览。')
if not all(state.get('/navigation/'+k+'_valid') for k in ('localization','perception')):raise SystemExit('定位或感知尚未就绪。')
PY
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 listen-only off
sudo ip link set can0 up
restore() {
  sudo -n ip link set can0 down
  sudo -n ip link set can0 type can bitrate 500000 listen-only on
  sudo -n ip link set can0 up
}
trap restore EXIT
"$TASK_DIR/.ros_venv/bin/python" "$TASK_DIR/check_zero_driver.py" --seconds 30
