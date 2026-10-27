import json
import time
import numpy as np
from PIL import Image
from scipy.ndimage import label
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid,Odometry
from sensor_msgs.msg import PointCloud2
from common import ROOT,settings,lcm_types

rclpy.init();node=Node('nav2_costmap_diagnostic');data={}
node.create_subscription(OccupancyGrid,'/global_costmap/costmap',lambda m:data.update(grid=m),10)
node.create_subscription(Odometry,'/lio/odom',lambda m:data.update(odom=m),10)
node.create_subscription(PointCloud2,'/points_navigation',lambda m:data.update(cloud=m),qos_profile_sensor_data)
end=time.monotonic()+4
while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
report={}
if 'grid' in data:
    msg=data['grid'];grid=np.array(msg.data).reshape(msg.info.height,msg.info.width)
    colors=np.zeros((*grid.shape,3),dtype=np.uint8)+40
    colors[grid==0]=[225,240,230];colors[(grid>0)&(grid<99)]=[180,145,100];colors[grid>=99]=[220,65,50]
    Image.fromarray(np.flipud(colors)).resize((grid.shape[1]*2,grid.shape[0]*2)).save(ROOT/'live_costmap.png')
    np.save(ROOT/'live_costmap.npy',grid)
    for name,x,y in (('target',3.,2.5),):
        gx=int((x-msg.info.origin.position.x)/msg.info.resolution);gy=int((y-msg.info.origin.position.y)/msg.info.resolution)
        report[name]={'cell':[gx,gy],'cost':int(grid[gy,gx])}
    report['cost_values']={str(k):int(v) for k,v in zip(*np.unique(grid,return_counts=True))}
if 'cloud' in data:
    xyz=np.frombuffer(bytes(data['cloud'].data),dtype='<f4').reshape(-1,3)
    np.save(ROOT/'live_cloud_base.npy',xyz)
    floor=xyz[(xyz[:,2]>-.10)&(xyz[:,2]<.15)&(np.linalg.norm(xyz[:,:2],axis=1)<3)]
    rng=np.random.default_rng(0);best=None
    if len(floor)>30:
        for _ in range(300):
            sample=floor[rng.choice(len(floor),3,replace=False)]
            a=np.column_stack([sample[:,:2],np.ones(3)])
            if abs(np.linalg.det(a))<.01:continue
            fit=np.linalg.solve(a,sample[:,2])
            if np.linalg.norm(fit[:2])>.12 or abs(fit[2])>.10:continue
            residual=np.abs(floor[:,2]-np.column_stack([floor[:,:2],np.ones(len(floor))])@fit)
            mask=residual<.012
            if best is None or mask.sum()>best[0]:best=(int(mask.sum()),mask)
        if best:
            inliers=floor[best[1]];fit=np.linalg.lstsq(np.column_stack([inliers[:,:2],np.ones(len(inliers))]),inliers[:,2],rcond=None)[0]
            report['floor_fit']={'a':float(fit[0]),'b':float(fit[1]),'c':float(fit[2]),'inliers':best[0],'candidates':len(floor),
                                  'pitch_adjust_deg':float(np.degrees(np.arctan(fit[0])))}
(ROOT/'costmap_diagnosis.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
node.destroy_node();rclpy.shutdown()
