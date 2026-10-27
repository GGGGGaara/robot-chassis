"""Read-only capture of preview control-chain outputs, never sends a command."""
import json
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from common import ROOT

rclpy.init();node=Node('velocity_preview_diagnostic');data={}
topics=['/cmd_vel_nav','/cmd_vel_smoothed','/cmd_vel_collision_checked','/nav_cmd_preview']
for topic in topics:
    data[topic]=[]
    node.create_subscription(Twist,topic,lambda msg,t=topic:data[t].append([msg.linear.x,msg.angular.z]),10)
end=time.monotonic()+5
while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
report={'read_only':True,'hardware_cmd_publishers':node.count_publishers('/tracer/cmd_vel'),'topics':{}}
for topic,values in data.items():
    report['topics'][topic]={'messages':len(values),'max_abs_linear':max((abs(v) for v,w in values),default=0.),
        'max_abs_angular':max((abs(w) for v,w in values),default=0.),'last':values[-1] if values else None}
(ROOT/'velocity_preview_check.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
node.destroy_node();rclpy.shutdown()
