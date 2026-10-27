"""Actual zero-only driver on vcan: startup delay may settle, later loss must fail."""
import json
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import threading
import time
from common import ROOT, require_virtual_can

require_virtual_can()
if os.environ.get('ROS_DOMAIN_ID')!='43':raise SystemExit('Test requires ROS domain 43')
out=ROOT/'results/zero_startup_fix_20261020'
checks={}

def run(label,drop_during_measurement=False):
    started=threading.Event();finished=threading.Event();measure_time=[];output=[]
    child=subprocess.Popen([sys.executable,str(ROOT/'check_zero_driver.py'),'--interface','vcan_navtest','--seconds','3'],
        stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,env=dict(os.environ,ROS_DOMAIN_ID='43'))
    def read_output():
        for line in child.stdout:
            output.append(line)
            if 'Feedback stable; starting full' in line:
                measure_time.append(time.monotonic());started.set()
    reader=threading.Thread(target=read_output);reader.start()
    began=time.monotonic()
    try:
        with socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW) as bus:
            bus.bind(('vcan_navtest',))
            last_system=0.
            while child.poll() is None and time.monotonic()-began<22:
                now=time.monotonic()
                # Hold all input absent long enough to exercise actual driver startup.
                if now-began>=3.:
                    dt=now-measure_time[0] if measure_time else -1
                    dropping=drop_during_measurement and 1.<dt<2.
                    if not dropping and now-last_system>=.2:
                        bus.send(struct.pack('=IB3x8s',0x211,8,bytes.fromhex('00010A5000000000')))
                        last_system=now
                    bus.send(struct.pack('=IB3x8s',0x221,8,bytes(8)))
                time.sleep(.01)
        if child.poll() is None:child.kill();raise TimeoutError('Virtual zero check did not exit')
        reader.join(timeout=3)
        code=child.wait()
        report=json.loads((ROOT/'zero_virtual_check.json').read_text())
        (out/(label+'.json')).write_text(json.dumps(report,indent=2))
        (out/(label+'.log')).write_text(''.join(output))
        if drop_during_measurement:
            checks['loss_after_start_still_fails']=(code!=0 and not report['passed'] and report.get('ready_false',0)>0
                and any(v['reason']=='system_state_timeout' for v in report.get('driver_faults',[])))
        else:
            checks['delayed_start_then_full_measurement_passes']=(code==0 and report['passed'] and report.get('startup_ready_false',0)>0
                and report['ready_false']==0 and report['duration_s']==3.)
        checks[label+'_all_control_frames_zero']=report.get('all_motion_frames_zero') is True
        checks[label+'_virtual_interface_only']=report.get('interface')=='vcan_navtest' and report.get('test_platform')=='virtual_can'
    finally:
        if child.poll() is None:child.kill();child.wait()
        reader.join(timeout=3)

run('delayed_start')
run('in_measurement_loss',True)
report={'checks':checks,'passed':all(checks.values()),'physical_can_touched':False}
(out/'test_report.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
raise SystemExit(0 if report['passed'] else 1)
