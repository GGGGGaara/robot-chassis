"""Real CAN connectivity check with the driver hard-clamped to zero_only=true."""
import json
import argparse
import signal
import socket
import struct
import subprocess
import time
import os
import rclpy
from std_msgs.msg import Bool,String
from tracer_msgs.msg import TracerStatus
from ament_index_python.packages import get_package_prefix
from common import ROOT,driver_fingerprint,software_fingerprint,require_virtual_can

p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=5)
p.add_argument('--interface',choices=('can0','vcan_navtest'),default='can0')
args=p.parse_args()
if not 2<=args.seconds<=60:raise SystemExit('duration must be 2..60 seconds')
if args.interface=='vcan_navtest':
    require_virtual_can()
    if os.environ.get('ROS_DOMAIN_ID')!='43':raise SystemExit('Virtual test requires isolated ROS domain 43')
rclpy.init();node=rclpy.create_node('zero_driver_check');ready=[];speeds=[];commands=[];diagnostics=[];last_ready=0.;last_wheel=0.
startup_ready=[];startup_diagnostics=[];measuring=False;latest_ready=False;latest_diagnostic=None;last_diagnostic=0.;startup_false_generation=0

def diagnostic_sample(msg):
    global latest_diagnostic,last_diagnostic
    latest_diagnostic=json.loads(msg.data);last_diagnostic=time.monotonic()
    (diagnostics if measuring else startup_diagnostics).append(latest_diagnostic)
node.create_subscription(String,'/tracer/driver_diagnostics',diagnostic_sample,100)
def ready_sample(msg):
    global last_ready,latest_ready,startup_false_generation
    last_ready=time.monotonic();latest_ready=msg.data
    (ready if measuring else startup_ready).append(msg.data)
    if not msg.data:startup_false_generation+=1
def wheel_sample(msg):
    global last_wheel
    last_wheel=time.monotonic();speeds.append([msg.linear_velocity,msg.angular_velocity])
node.create_subscription(Bool,'/tracer/ready',ready_sample,10)
node.create_subscription(TracerStatus,'/tracer_status',wheel_sample,10)
bus=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
bus.setsockopt(socket.SOL_CAN_RAW,socket.CAN_RAW_FILTER,struct.pack('=II',0x111,0x7ff))
bus.bind((args.interface,));bus.setblocking(False)
exe=get_package_prefix('tracer_navigation_driver')+'/lib/tracer_navigation_driver/tracer_safe_base'
prefix='zero_driver_check' if args.interface=='can0' else 'zero_virtual_check'
log=(ROOT/(prefix+'.log')).open('w');driver=None
report={'timestamp':time.time(),'zero_only':True,'forward_pulse_requested':False,'interface':args.interface,
    'test_platform':'physical_can0' if args.interface=='can0' else 'virtual_can',
    'driver_sha256':driver_fingerprint(),'software_sha256':software_fingerprint()}
def drain_commands():
    while True:
        try:cid,dlc,data=struct.unpack('=IB3x8s',bus.recv(16))
        except BlockingIOError:break
        commands.append(data.hex())
        if dlc!=8 or data!=bytes(8):raise RuntimeError('Nonzero or malformed motion frame during zero-only check')
try:
    for _ in range(20):rclpy.spin_once(node,timeout_sec=.02)
    if node.count_publishers('/tracer/ready'):raise RuntimeError('Hardware driver already running')
    driver=subprocess.Popen([exe,'--ros-args','-p','enable_drive:=true','-p','zero_only:=true',
        '-p','can_interface:='+args.interface],stdout=log,stderr=subprocess.STDOUT)
    startup_begin=time.monotonic();stable_since=None;last_false_generation=startup_false_generation
    while True:
        rclpy.spin_once(node,timeout_sec=.02);drain_commands();now=time.monotonic()
        if driver.poll() is not None:raise RuntimeError('Zero-only driver exited during initialization')
        diagnostic=latest_diagnostic or {}
        healthy=(latest_ready and now-last_ready<.2 and now-last_wheel<.2 and now-last_diagnostic<1.2
            and diagnostic.get('ready') and diagnostic.get('reason')=='ok'
            and diagnostic.get('system_age_s',1)<.6 and diagnostic.get('motion_age_s',1)<.3)
        if not healthy or startup_false_generation!=last_false_generation:stable_since=None
        last_false_generation=startup_false_generation
        if healthy and stable_since is None:stable_since=now
        if stable_since is not None and now-stable_since>=1.:break
        if now-startup_begin>10:raise RuntimeError('Fresh CAN feedback did not stabilize within 10 seconds')
    report['startup_duration_s']=time.monotonic()-startup_begin
    report['measurement_started_timestamp']=time.time()
    measuring=True
    print(f'Feedback stable; starting full {args.seconds:g}-second zero-only measurement.',flush=True)
    end=time.monotonic()+args.seconds
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.02)
        drain_commands()
    report.update(duration_s=args.seconds,ready_received=len(ready),ready_true=ready.count(True),ready_false=ready.count(False),ready_last=ready[-1] if ready else None,wheel_feedback_count=len(speeds),
        final_ready_age_s=time.monotonic()-last_ready,final_wheel_age_s=time.monotonic()-last_wheel,
        max_abs_wheel_linear=max((abs(v) for v,w in speeds),default=None),
        max_abs_wheel_angular=max((abs(w) for v,w in speeds),default=None),motion_frames=len(commands),
        all_motion_frames_zero=bool(commands) and all(v=='0000000000000000' for v in commands),
        driver_faults=[v for v in diagnostics if not v.get('ready')])
except (Exception,KeyboardInterrupt) as error:
    report['error']=repr(error)
finally:
    if driver is not None:
        driver.send_signal(signal.SIGINT);driver.wait(timeout=4);report['driver_exit_code']=driver.returncode
    report['startup_ready_false']=startup_ready.count(False)
    report['startup_driver_faults']=[v for v in startup_diagnostics if not v.get('ready')]
    report['passed']=bool(not report.get('error') and report.get('ready_last') and report.get('ready_false')==0 and report.get('ready_received',0)>=args.seconds*25
        and report.get('final_ready_age_s',1)<.3 and report.get('final_wheel_age_s',1)<.3
        and report.get('motion_frames',0)>=args.seconds*10 and report.get('all_motion_frames_zero')
        and report.get('max_abs_wheel_linear',1)<.01 and report.get('max_abs_wheel_angular',1)<.02 and report.get('driver_exit_code')==0)
    (ROOT/(prefix+'.json')).write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
    log.close();bus.close();node.destroy_node();rclpy.shutdown()
raise SystemExit(0 if report.get('passed') else 1)
