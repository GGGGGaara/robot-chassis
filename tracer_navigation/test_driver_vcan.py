"""Exercise CAN codec/watchdogs on vcan_navtest ONLY, never on physical can0."""
import json
import os
import signal
import socket
import struct
import subprocess
import time
from pathlib import Path
os.environ['ROS_DOMAIN_ID']='42'
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool,String
from ament_index_python.packages import get_package_prefix
from common import ROOT,require_virtual_can,settings

require_virtual_can()
rclpy.init();node=rclpy.create_node('vcan_driver_probe');pub=node.create_publisher(Twist,'/vcan_test/cmd_vel',10)
ready=[];node.create_subscription(Bool,'/vcan_test/ready',lambda m:ready.append(m.data),qos_profile_sensor_data)
diagnostics=[];node.create_subscription(String,'/vcan_test/diagnostics',lambda m:diagnostics.append(json.loads(m.data)),100)
bus=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW);bus.bind(('vcan_navtest',));bus.setblocking(False)
executable=get_package_prefix('tracer_navigation_driver')+'/lib/tracer_navigation_driver/tracer_safe_base'
report={'interface':'vcan_navtest','physical_can_touched':False,'checks':{}}
# The write interceptor polls this flag; keep it on Linux tmpfs/filesystem.
# A flag on /mnt/d adds Windows filesystem stalls to every CAN write.
fail_file=Path('/tmp')/f'tracer_vcan_write_fail_{os.getpid()}.flag';fail_file.unlink(missing_ok=True)
injector=ROOT/'generated/test_can_write_fault.so'
subprocess.run(['gcc','-shared','-fPIC','-O2',str(ROOT/'ws/src/tracer_navigation_driver/test_can_write_fault.c'),'-ldl','-o',str(injector)],check=True)
driver_env={**os.environ,'LD_PRELOAD':str(injector),'TRACER_TEST_CAN_FAIL_FILE':str(fail_file)}
def capture(seconds,feedback=True,command=None,state=None,system=True,motion=True,system_period=0.):
    values=[];end=time.monotonic()+seconds;last_system=float('-inf')
    while time.monotonic()<end:
        if feedback:
            now=time.monotonic()
            if system and now-last_system>=system_period:
                bus.send(struct.pack('=IB3x8s',0x211,8,state or bytes.fromhex('00010A5000000000')));last_system=now
            if motion:bus.send(struct.pack('=IB3x8s',0x221,8,bytes(8)))
        if command is not None:
            msg=Twist();msg.linear.x=command[0];msg.angular.z=command[1];pub.publish(msg)
        rclpy.spin_once(node,timeout_sec=.02)
        while True:
            try:cid,dlc,data=struct.unpack('=IB3x8s',bus.recv(16))
            except BlockingIOError:break
            if cid==0x111:values.append(struct.unpack('>hhhh',data))
    return values
try:
    for zero in (False,True):
        log=(ROOT/f'vcan_driver_{zero}.log').open('w')
        driver=subprocess.Popen([executable,'--ros-args','-p','enable_drive:=true','-p','can_interface:=vcan_navtest',
            '-r','/tracer/cmd_vel:=/vcan_test/cmd_vel','-r','/tracer/ready:=/vcan_test/ready',
            '-r','/tracer_status:=/vcan_test/status','-r','/tracer/wheel_odom:=/vcan_test/odom',
            '-r','/tracer/driver_diagnostics:=/vcan_test/diagnostics',
            '-p',f'max_linear_mps:={settings()["robot"]["max_linear_mps"]}',
            '-p',f'zero_only:={str(zero).lower()}'],stdout=log,stderr=subprocess.STDOUT,env=driver_env)
        try:
            initial=capture(3.)
            discovery_deadline=time.monotonic()+10
            while not ready and time.monotonic()<discovery_deadline:initial+=capture(.3)
            assert ready and ready[-1],{'ready':ready[-10:],'publishers':node.count_publishers('/vcan_test/ready'),'pid':driver.pid,'returncode':driver.poll(),'CAN_commands':len(initial)}
            values=capture(.7,command=(.03,.1));assert values
            if zero:
                assert all(v==(0,0,0,0) for v in values);report['checks']['zero_only_rejects_motion']=True
            else:
                assert values[-1]==(30,100,0,0),values[-1]
                report['checks']['codec_command_and_unused_fields']=True
                values=capture(.6);assert values[-5:] and all(v==(0,0,0,0) for v in values[-5:])
                report['checks']['command_timeout_zero']=True
                capture(.4,command=(.03,0.))
                values=capture(.6,feedback=False,command=(.03,0.));assert len(values)>=3 and all(v==(0,0,0,0) for v in values[-3:]),values
                report['checks']['feedback_timeout_zero']=True
                values=capture(.5,command=(2.,2.));assert values[-1]==(round(settings()['robot']['max_linear_mps']*1000),150,0,0),values[-1]
                report['checks']['driver_speed_limits']=True
                values=capture(.5,command=(.08,0.));assert values[-1]==(80,0,0,0),values[-1]
                report['checks']['configured_speed_above_old_50mm_limit']=True
                values=capture(1.3,command=(.03,0.),system_period=.4)
                assert values and all(v[0]==30 for v in values[-10:]),values[-10:]
                report['checks']['slow_status_with_one_missed_5hz_frame']=True
                values=capture(.9,command=(.03,0.),system=False)
                assert all(v==(0,0,0,0) for v in values[-3:]),{'values':values,'diagnostics':diagnostics[-4:]}
                assert diagnostics[-1]['reason']=='system_state_timeout'
                report['checks']['status_loss_stops_within_600ms']=True
                capture(.3,command=(.03,0.))
                values=capture(.5,command=(.03,0.),motion=False)
                assert all(v==(0,0,0,0) for v in values[-3:]),{'values':values,'diagnostics':diagnostics[-4:]}
                assert diagnostics[-1]['reason']=='wheel_feedback_timeout'
                report['checks']['wheel_loss_still_stops_within_300ms']=True
                capture(.3,command=(.03,0.))
                driver.send_signal(signal.SIGSTOP)
                try:
                    capture(.04,command=(.03,0.))
                    capture(.75,feedback=False,command=(.03,0.))
                finally:driver.send_signal(signal.SIGCONT)
                values=capture(.15,feedback=False,command=(.03,0.))
                assert values and all(v==(0,0,0,0) for v in values),values
                report['checks']['queued_old_feedback_cannot_refresh_watchdog']=True
                capture(.3,command=(.03,0.))
                fail_file.write_text('test vcan ENOBUFS only')
                try:
                    values=capture(.35,command=(.03,0.))
                    assert ready and not ready[-1]
                    assert diagnostics[-1]['reason']=='can_write_error' and diagnostics[-1]['send_errno']==105
                    report['checks']['enobufs_reports_not_ready']=True
                finally:fail_file.unlink(missing_ok=True)
                values=capture(.4,command=(.03,0.))
                assert values and values[0]==(0,0,0,0),values
                assert values[-1]==(30,0,0,0) and ready[-1]
                report['checks']['write_recovery_sends_zero_before_motion']=True
                for label,state in (('physical_estop_feedback','01010A5000000000'),('rc_control_mode','00030A5000000000'),('fault_feedback','00010A5000010000')):
                    values=capture(.5,command=(.03,.1),state=bytes.fromhex(state))
                    assert all(v==(0,0,0,0) for v in values[-5:]);report['checks'][label+'_zero']=True
                for label,command in (('nonfinite',(float('nan'),0.)),('reverse',(-.03,0.))):
                    values=capture(.5,command=command)
                    assert all(v==(0,0,0,0) for v in values[-5:]);report['checks'][label+'_rejected']=True
                capture(.3,command=(0.,0.))
        finally:
            driver.send_signal(signal.SIGINT);driver.wait(timeout=3);log.close()
            assert driver.returncode==0,driver.returncode
    report['checks']['clean_shutdown']=True;report['passed']=True
    from common import driver_fingerprint,software_fingerprint
    report['driver_sha256']=driver_fingerprint();report['software_sha256']=software_fingerprint()
finally:
    fail_file.unlink(missing_ok=True)
    (ROOT/'vcan_driver_report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
    bus.close();node.destroy_node();rclpy.shutdown()
