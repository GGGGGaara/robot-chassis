"""Read-only map/footprint snapshot. No goals, arming or velocity publishers."""
import json
import math
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from common import ROOT, settings, cloud_xyz, transform_matrix
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.path import Path as PolygonPath

rclpy.init()
node = Node('start_collision_readonly')
buffer = Buffer()
listener = TransformListener(buffer, node)
data = {}
for topic in ('/map', '/local_costmap/costmap', '/global_costmap/costmap'):
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE) if topic == '/map' else 10
    node.create_subscription(OccupancyGrid, topic, lambda m, t=topic: data.update({t:m}), qos)
node.create_subscription(PointCloud2, '/points_navigation', lambda m:data.update(cloud=m), qos_profile_sensor_data)
node.create_subscription(String, '/navigation/safety_status', lambda m:data.update(safety=json.loads(m.data)), 10)
end = time.monotonic() + 6
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=.03)
out = ROOT/'results'/('start_collision_'+time.strftime('%Y%m%d_%H%M%S'))
out.mkdir()
report = {'timestamp':time.time(), 'read_only':True, 'safety':data.get('safety'), 'maps':{}}
c = settings()
length = c['robot']['length_m']/2 + .1
width = c['robot']['width_m']/2 + .1
footprint = np.array([[length,width], [length,-width], [-length,-width], [-length,width], [length,width]])
fig, axes = plt.subplots(1, 3, figsize=(15,5), layout='constrained')
cloud = cloud_xyz(data['cloud']) if 'cloud' in data else np.empty((0,3))
obstacles = cloud[(cloud[:,2]>=.03)&(cloud[:,2]<=c['robot']['height_m'])]
report['live_obstacles_within_1m'] = obstacles[np.linalg.norm(obstacles[:,:2],axis=1)<1].tolist()
selected = json.loads((ROOT.parent/'realtime_mapping_usage/path_preview_results/latest_path.json').read_text())
try:
    bfm = transform_matrix(buffer.lookup_transform('base_link','map',rclpy.time.Time()).transform)
    target = (bfm @ np.array([*selected['goal'],0.,1.]))[:2]
    report['target_base'] = target.tolist()
except Exception as error:
    report['target_error'] = str(error)
for ax, topic in zip(axes, ('/map', '/global_costmap/costmap', '/local_costmap/costmap')):
    ax.set(title=topic, xlabel='base x (m)', ylabel='base y (m)', xlim=(-1.4,2.0),ylim=(-1.6,1.6))
    ax.set_aspect('equal')
    if topic not in data:
        ax.text(0,0,'No map received'); continue
    msg = data[topic]
    grid = np.asarray(msg.data).reshape(msg.info.height, msg.info.width)
    np.save(out/(topic.strip('/').replace('/','_')+'.npy'), grid)
    try:
        base_from_frame = transform_matrix(buffer.lookup_transform('base_link', msg.header.frame_id, rclpy.time.Time()).transform)
        origin = msg.info.origin.position
        res = msg.info.resolution
        yy,xx = np.indices(grid.shape)
        world = np.column_stack([(xx.ravel()+.5)*res+origin.x,(yy.ravel()+.5)*res+origin.y,np.zeros(grid.size)])
        xy = (world @ base_from_frame[:3,:3].T + base_from_frame[:3,3])[:,:2]
        use=(xy[:,0]>-1.5)&(xy[:,0]<2.1)&(abs(xy[:,1])<1.7)
        vals=grid.ravel()[use]; points=xy[use]
        color=np.full((len(vals),3),.97)
        color[vals<0]=[.6,.6,.6]
        color[(vals>0)&(vals<99)]=[.97,.79,.48]
        color[vals==99]=[.92,.40,.55]
        color[vals>=100]=[.05,.05,.05]
        ax.scatter(points[:,0],points[:,1],c=color,s=16,marker='s',linewidths=0)
        info={'frame':msg.header.frame_id,'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
              'base_from_frame':base_from_frame.tolist(), 'resolution':res,
              'origin':[origin.x,origin.y], 'boundary_costs':{}}
        for degrees in (0,5,-5,10,-10,20,-20,30,-30,45,-45):
            theta=math.radians(degrees)
            rotation=np.array([[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]])
            boundary=np.concatenate([a[None,:]+np.linspace(0,1,50)[:,None]*(b-a)[None,:] for a,b in zip(footprint,footprint[1:])]) @ rotation.T
            frame_from_base=np.linalg.inv(base_from_frame)
            q=np.column_stack([boundary,np.zeros(len(boundary))]) @ frame_from_base[:3,:3].T+frame_from_base[:3,3]
            ix=np.floor((q[:,0]-origin.x)/res).astype(int)
            iy=np.floor((q[:,1]-origin.y)/res).astype(int)
            valid=(ix>=0)&(iy>=0)&(ix<msg.info.width)&(iy<msg.info.height)
            costs=grid[iy[valid],ix[valid]]
            info['boundary_costs'][str(degrees)]={'max':int(costs.max()) if len(costs) else None,'unknown':int((costs<0).sum()),'lethal_or_inscribed':int((costs>=99).sum())}
        occupied=(vals>=99)&(np.linalg.norm(points,axis=1)<1.)
        info['near_occupied_cell_centers_base'] = points[occupied].tolist()
        info['occupied_within_padded_footprint'] = int(((vals>=99)&PolygonPath(footprint).contains_points(points)).sum())
        report['maps'][topic]=info
    except Exception as error:
        report['maps'][topic]={'error':str(error)}
    ax.plot(footprint[:,0],footprint[:,1],color='#0877c9',lw=2,label='Padded footprint')
    ax.scatter(obstacles[:,0],obstacles[:,1],s=2,c='#d41414',alpha=.7,label='Live points >=3cm')
    ax.arrow(0,0,.3,0,width=.012,color='#0877c9')
axes[0].legend(fontsize=7)
fig.suptitle('READ ONLY: start collision diagnosis; black=occupied, pink=inscribed, red=live obstacles')
fig.savefig(out/'maps_and_footprint.png',dpi=160)
(out/'report.json').write_text(json.dumps(report,indent=2))
print(json.dumps({'folder':str(out),'safety':report['safety'],'live_near_obstacle_points':len(report['live_obstacles_within_1m']),
    'target_base':report.get('target_base'),'maps':{k:{'frame':v.get('frame'),'occupied_within_padded_footprint':v.get('occupied_within_padded_footprint'),'boundary_costs':v.get('boundary_costs'),'error':v.get('error')} for k,v in report['maps'].items()}},indent=2))
node.destroy_node()
rclpy.shutdown()
