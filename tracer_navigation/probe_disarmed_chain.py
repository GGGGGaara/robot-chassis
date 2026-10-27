"""Dry-run-only command-chain probe; refuses any physical driver or command publisher."""
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from common import ROOT,settings,cloud_xyz,near_stop_count

rclpy.init();node=Node('disarmed_chain_probe');state={};samples=[];clouds=[]
node.create_subscription(String,'/navigation/safety_status',lambda m:state.update(gate=(time.monotonic(),json.loads(m.data))),10)
def cloud(m):
    if m.header.frame_id!='base_link':return
    age=time.time()-m.header.stamp.sec-m.header.stamp.nanosec*1e-9
    clouds.append({'t':time.monotonic(),'stamp_age':age,'near':near_stop_count(cloud_xyz(m),settings())})
node.create_subscription(PointCloud2,'/points_navigation',cloud,qos_profile_sensor_data)
for topic in ('/cmd_vel_smoothed','/cmd_vel_collision_checked'):
    node.create_subscription(Twist,topic,lambda m,t=topic:samples.append({'t':time.monotonic(),'topic':t,'v':m.linear.x,'w':m.angular.z}),10)
publisher=None
def guard():
    stamp,gate=state.get('gate',(0,{}))
    if time.monotonic()-stamp>.4 or gate.get('mode')!='dry-run' or gate.get('armed') is not False:
        raise RuntimeError('Fresh disarmed preview required')
    if node.count_publishers('/tracer/ready') or node.count_publishers('/tracer/cmd_vel'):
        raise RuntimeError('Physical driver/publisher exists; no preview probe allowed')
report={'timestamp':time.time(),'hardware_driver_started':False,'arming_requested':False,'physical_velocity_publisher':False}
try:
    end=time.monotonic()+2
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    guard()
    publisher=node.create_publisher(Twist,'/cmd_vel_nav',10)
    begin=time.monotonic();end=begin+6
    while time.monotonic()<end:
        guard();m=Twist();m.linear.x=.03;publisher.publish(m);rclpy.spin_once(node,timeout_sec=.03)
    report['duration_s']=time.monotonic()-begin
    report['samples']=samples
    report['clouds']=clouds
    report['near_min_max']=[min(v['near'] for v in clouds),max(v['near'] for v in clouds)] if clouds else None
    report['topics']={}
    for topic in ('/cmd_vel_smoothed','/cmd_vel_collision_checked'):
        values=[v for v in samples if v['topic']==topic and v['t']>begin+1]
        report['topics'][topic]={'messages':len(values),'zero':sum(abs(v['v'])+abs(v['w'])<1e-6 for v in values),
            'max_v':max((v['v'] for v in values),default=None),'last':values[-1] if values else None}
    report['completed']=True
except Exception as error:
    report['error']=str(error);report['completed']=False
finally:
    if publisher is not None:
        end=time.monotonic()+1
        while time.monotonic()<end:publisher.publish(Twist());rclpy.spin_once(node,timeout_sec=.02)
    report['final_gate']=state.get('gate',(0,None))[1]
    (ROOT/'results/disarmed_chain_probe_20261020.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k not in ('samples','clouds')},indent=2))
    node.destroy_node();rclpy.shutdown()
raise SystemExit(0 if report.get('completed') else 1)
