"""Read-only check of a front test patch, not proof that an obstacle is absent."""
import json
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer,TransformListener
from common import ROOT,cloud_xyz,transform_matrix

rclpy.init();node=Node('front_obstacle_diagnostic');data={};counts=[]
buffer=Buffer();listener=TransformListener(buffer,node)
def cloud(msg):
    xyz=cloud_xyz(msg)
    patch=(xyz[:,0]>=1.1)&(xyz[:,0]<=1.9)&(abs(xyz[:,1])<=.4)&(xyz[:,2]>=.03)
    counts.append(int(patch.sum()))
node.create_subscription(PointCloud2,'/points_navigation',cloud,qos_profile_sensor_data)
node.create_subscription(OccupancyGrid,'/global_costmap/costmap',lambda m:data.update(grid=m),10)
deadline=time.monotonic()+5
while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
report={'read_only':True,'patch_base_m':{'x':[1.1,1.9],'y':[-.4,.4]},
        'cloud_frames':len(counts),'live_obstacle_points_min_max':[min(counts),max(counts)] if counts else None}
try:
    msg=data['grid'];t=buffer.lookup_transform(msg.header.frame_id,'base_link',rclpy.time.Time())
    matrix=transform_matrix(t.transform)
    xy=np.array([[x,y,0.] for x in np.arange(1.1,1.901,.08) for y in np.arange(-.4,.401,.08)])
    world=xy@matrix[:3,:3].T+matrix[:3,3]
    ix=np.floor((world[:,0]-msg.info.origin.position.x)/msg.info.resolution).astype(int)
    iy=np.floor((world[:,1]-msg.info.origin.position.y)/msg.info.resolution).astype(int)
    valid=(ix>=0)&(ix<msg.info.width)&(iy>=0)&(iy<msg.info.height)
    grid=np.array(msg.data).reshape(msg.info.height,msg.info.width)
    cells=set(zip(ix[valid],iy[valid]));costs=np.array([grid[y,x] for x,y in cells])
    report['costmap_patch']={'cells':len(cells),'lethal':int((costs==100).sum()),
        'inscribed':int((costs==99).sum()),'unknown':int((costs<0).sum()),
        'free':int((costs==0).sum()),'inflated':int(((costs>0)&(costs<99)).sum())}
except Exception as error:report['error']=str(error)
(ROOT/'front_obstacle_check.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
node.destroy_node();rclpy.shutdown()
