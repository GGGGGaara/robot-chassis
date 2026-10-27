"""Isolate Python/C++ DDS delivery on virtual CAN only, with zero commands."""
import json,os,signal,socket,struct,subprocess,time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from common import ROOT,require_virtual_can
from ament_index_python.packages import get_package_prefix
require_virtual_can();rclpy.init();n=rclpy.create_node('dds_delivery_probe')
counts={'python_self':0,'cpp_reliable':0,'cpp_best_effort':0}
def add(key):
    def callback(msg):counts[key]+=1
    return callback
n.create_subscription(Bool,'/dds_probe/self',add('python_self'),10)
n.create_subscription(Bool,'/dds_probe/ready',add('cpp_reliable'),10)
n.create_subscription(Bool,'/dds_probe/ready',add('cpp_best_effort'),qos_profile_sensor_data)
p=n.create_publisher(Bool,'/dds_probe/self',10)
bus=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW);bus.bind(('vcan_navtest',));bus.setblocking(False)
exe=get_package_prefix('tracer_navigation_driver')+'/lib/tracer_navigation_driver/tracer_safe_base'
log=(ROOT/'dds_delivery_probe.log').open('w')
child=subprocess.Popen([exe,'--ros-args','-p','enable_drive:=true','-p','can_interface:=vcan_navtest','-p','zero_only:=true',
    '-r','/tracer/ready:=/dds_probe/ready','-r','/tracer_status:=/dds_probe/status','-r','/tracer/wheel_odom:=/dds_probe/odom',
    '-r','/tracer/driver_diagnostics:=/dds_probe/diag'],stdout=log,stderr=subprocess.STDOUT)
try:
    start=time.monotonic()
    while time.monotonic()-start<8:
        for cid,data in ((0x211,bytes.fromhex('00010a5000000000')),(0x221,bytes(8))):bus.send(struct.pack('=IB3x8s',cid,8,data))
        p.publish(Bool(data=True));rclpy.spin_once(n,timeout_sec=.005);time.sleep(.005)
        while True:
            try:bus.recv(16)
            except BlockingIOError:break
    report={'virtual_can_only':True,'counts':counts,'publishers':n.count_publishers('/dds_probe/ready'),
        'endpoints':[str(v.qos_profile) for v in n.get_publishers_info_by_topic('/dds_probe/ready')]}
    print(json.dumps(report,indent=2));(ROOT/'dds_delivery_report.json').write_text(json.dumps(report,indent=2))
finally:
    child.send_signal(signal.SIGINT);child.wait(timeout=4);log.close();bus.close();n.destroy_node();rclpy.shutdown()
