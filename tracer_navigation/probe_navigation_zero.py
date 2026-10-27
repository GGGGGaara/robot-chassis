"""Full navigation preview + physically hard-zero driver; never sends a goal."""
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
ROOT=Path(__file__).resolve().parent
log=(ROOT/'navigation_zero_probe.log').open('w')
launch=subprocess.Popen(['bash',str(ROOT/'start_navigation.sh')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
try:
    time.sleep(12)
    if launch.poll() is not None:raise RuntimeError('Navigation preview exited; inspect log')
    status=subprocess.run(['bash',str(ROOT/'navctl.sh'),'status'],capture_output=True,text=True,timeout=30)
    report=json.loads(status.stdout)
    names=['map_server','controller_server','planner_server','behavior_server','bt_navigator','velocity_smoother','collision_monitor']
    if not all(report.get(k)=='active' for k in names):raise RuntimeError('Not all lifecycle nodes active')
    if report.get('safety',{}).get('mode')!='dry-run' or report.get('safety',{}).get('armed'):
        raise RuntimeError('Expected disarmed preview')
    subprocess.run([sys.executable,str(ROOT/'check_zero_driver.py'),'--seconds','20'],check=True)
finally:
    os.killpg(launch.pid,signal.SIGINT)
    try:launch.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(launch.pid,signal.SIGTERM);launch.wait(timeout=5)
    log.close()
