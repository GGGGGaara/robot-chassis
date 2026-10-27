"""One-shot runtime cache reset, only with fresh DISARMED gate and clear live near zone."""
import hashlib
import json
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.srv import ClearCostmapAroundRobot
from tf2_ros import Buffer,TransformListener
from common import ROOT, settings, cloud_xyz,transform_matrix

rclpy.init()
node=Node('reset_costmap_cache_disarmed')
state={}
cloud_counts=[]
map_series=[]
tf=Buffer();listener=TransformListener(tf,node)
def costmap(m,topic):
    try:
        matrix=transform_matrix(tf.lookup_transform('base_link',m.header.frame_id,rclpy.time.Time()).transform)
        grid=np.asarray(m.data).reshape(m.info.height,m.info.width)
        yy,xx=np.nonzero(grid>=99)
        xyz=np.column_stack([(xx+.5)*m.info.resolution+m.info.origin.position.x,
                             (yy+.5)*m.info.resolution+m.info.origin.position.y,np.zeros(len(xx))])
        base=xyz@matrix[:3,:3].T+matrix[:3,3]
        c=settings();inside=(abs(base[:,0])<c['robot']['length_m']/2+.10)&(abs(base[:,1])<c['robot']['width_m']/2+.10)
        lethal=grid[yy,xx]>=100
        map_series.append({'t':time.time(),'topic':topic,'occupied_in_padded_footprint':int(inside.sum()),
            'lethal_in_padded_footprint':int((inside&lethal).sum()),
            'nearest_lethal_distance_m':float(np.linalg.norm(base[lethal,:2],axis=1).min()) if lethal.any() else None})
    except Exception:pass
for topic in ('/local_costmap/costmap','/global_costmap/costmap'):
    node.create_subscription(OccupancyGrid,topic,lambda m,t=topic:costmap(m,t),10)
node.create_subscription(String,'/navigation/safety_status',lambda m:state.update(gate=(time.monotonic(),json.loads(m.data))),10)
def cloud(m):
    age=time.time()-m.header.stamp.sec-m.header.stamp.nanosec/1e9
    if m.header.frame_id!='base_link' or not 0<=age<.3:return
    xyz=cloud_xyz(m)
    n=int(((xyz[:,2]>=.03)&(np.linalg.norm(xyz[:,:2],axis=1)<1.)).sum())
    cloud_counts.append(n)
    state['cloud']=(time.monotonic(),n)
node.create_subscription(PointCloud2,'/points_navigation',cloud,qos_profile_sensor_data)
def safe():
    now=time.monotonic()
    stamp,gate=state.get('gate',(0,{}))
    if now-stamp>.4 or gate.get('armed') is not False or abs(gate.get('linear',1))>1e-8 or abs(gate.get('angular',1))>1e-8:
        raise RuntimeError('Fresh disarmed zero-output state required')
    stamp,n=state.get('cloud',(0,1))
    if now-stamp>.3 or n!=0:raise RuntimeError('Live near obstacle returns present or cloud stale')
report={'timestamp':time.time(),'no_arming':True,'no_velocity_publisher':True,'reset_distance_m':2.8,'services':[]}
source=ROOT.parent/'reconstruction_20261017_113920/raytrace/global_costmap.lcm'
before=hashlib.sha256(source.read_bytes()).hexdigest()
try:
    end=time.monotonic()+3
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    safe()
    if len(cloud_counts)<5 or max(cloud_counts[-5:])!=0:raise RuntimeError('Need consecutive clear live clouds')
    for topic in ('/local_costmap/clear_around_local_costmap','/global_costmap/clear_around_global_costmap'):
        client=node.create_client(ClearCostmapAroundRobot,topic)
        if not client.wait_for_service(timeout_sec=2):raise RuntimeError('Missing service '+topic)
        safe()
        request=ClearCostmapAroundRobot.Request();request.reset_distance=2.8
        future=client.call_async(request)
        deadline=time.monotonic()+3
        while not future.done() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02);safe()
        if not future.done() or future.result() is None:raise RuntimeError('Cache reset response failed '+topic)
        report['services'].append(topic)
    report['reset_completed_timestamp']=time.time()
    end=time.monotonic()+10
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.02)
        stamp,gate=state.get('gate',(0,{}))
        if gate.get('armed') is not False:raise RuntimeError('Gate changed during observation')
    safe()
    report['completed']=True
except Exception as error:
    report['completed']=False;report['error']=str(error)
finally:
    report['fixed_map_unchanged']=before==hashlib.sha256(source.read_bytes()).hexdigest()
    report['final_gate']=state.get('gate',(0,None))[1]
    report['cloud_count_range']=[min(cloud_counts),max(cloud_counts)] if cloud_counts else None
    report['map_series']=map_series
    (ROOT/'results/cache_reset_20261020.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
    node.destroy_node();rclpy.shutdown()
raise SystemExit(0 if report.get('completed') and report['fixed_map_unchanged'] else 1)
