import json
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool,String
from sensor_msgs.msg import PointCloud2
from lifecycle_msgs.srv import GetState
from common import ROOT

rclpy.init();node=Node('navigation_runtime_check');values={};clients={}
for topic in ('/navigation/localization_valid','/navigation/perception_valid','/tracer/ready'):
    node.create_subscription(Bool,topic,lambda msg,t=topic:values.update({t:msg.data}),10)
node.create_subscription(String,'/navigation/safety_status',lambda msg:values.update({'safety':json.loads(msg.data)}),10)
node.create_subscription(String,'/tracer/driver_diagnostics',lambda msg:values.update({'driver_diagnostic':json.loads(msg.data)}),10)
def cloud(msg):
    xyz=np.frombuffer(bytes(msg.data),dtype='<f4').reshape(-1,3)
    values['cloud']={'frame':msg.header.frame_id,'points':msg.width,'z_percentiles_m':np.percentile(xyz[:,2],[0,25,50,75,100]).tolist() if len(xyz) else []}
node.create_subscription(PointCloud2,'/points_navigation',cloud,qos_profile_sensor_data)
for name in ('map_server','controller_server','planner_server','behavior_server','bt_navigator','velocity_smoother','collision_monitor'):
    clients[name]=node.create_client(GetState,f'/{name}/get_state')
end=time.monotonic()+4
while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
for name,client in clients.items():
    if client.wait_for_service(timeout_sec=.2):
        future=client.call_async(GetState.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=2)
        values[name]=future.result().current_state.label if future.done() and future.result() else 'unavailable'
values['hardware_driver_publishers']=len(node.get_publishers_info_by_topic('/tracer/ready'))
values['hardware_cmd_publishers']=len(node.get_publishers_info_by_topic('/tracer/cmd_vel'))
(ROOT/'runtime_check.json').write_text(json.dumps(values,indent=2))
print(json.dumps(values,indent=2));node.destroy_node();rclpy.shutdown()
