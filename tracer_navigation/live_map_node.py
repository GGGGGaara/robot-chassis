"""Real-time clearance maps from raw lidar rays; reference is planning advice only."""
from array import array
import json
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener,TransformException
from common import settings,cloud_xyz,mount_matrix,transform_matrix
from live_grid import LiveGrid,fuse_reference


class LiveMaps(Node):
    def __init__(self):
        super().__init__('live_clearance_maps');self.config=settings();self.core=LiveGrid(self.config)
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.pending=None;self.source_stamp=None;self.source_wall=None;self.reference=None;self.map_transform=None
        self.origin_sensor=mount_matrix(self.config)[:3,3];self.mount=mount_matrix(self.config)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.live_pub=self.create_publisher(OccupancyGrid,'/navigation/live_map',qos)
        self.fused_pub=self.create_publisher(OccupancyGrid,'/map',qos)
        self.status=self.create_publisher(String,'/navigation/live_map_status',10)
        self.create_subscription(OccupancyGrid,'/navigation/reference_map',self.reference_map,qos)
        self.create_subscription(PointCloud2,'/lidar/points',lambda m:setattr(self,'pending',m),qos_profile_sensor_data)
        self.counter=0;self.create_timer(.1,self.tick)

    def reference_map(self,msg):
        if msg.header.frame_id!='map' or msg.info.resolution<=0 or msg.info.width*msg.info.height!=len(msg.data):return
        q=msg.info.origin.orientation
        if abs(q.x)+abs(q.y)+abs(q.z)>1e-6:return
        self.reference=msg

    def tick(self):
        now=time.monotonic();wall=time.time();cloud=self.pending
        error=None
        if cloud is not None:
            stamp=cloud.header.stamp.sec+cloud.header.stamp.nanosec*1e-9
            if cloud.header.frame_id!='mid360_link' or not 0<=wall-stamp<=.35:
                self.pending=None;error='raw_cloud_stale_or_wrong_frame'
            else:
                try:
                    t=self.tf.lookup_transform('odom','base_link',rclpy.time.Time.from_msg(cloud.header.stamp))
                    raw=self.tf.lookup_transform('map','odom',rclpy.time.Time.from_msg(cloud.header.stamp))
                    matrix=transform_matrix(t.transform);reg=transform_matrix(raw.transform)
                    xyz=cloud_xyz(cloud)
                    if len(xyz)<200:raise ValueError('insufficient_raw_returns')
                    base=np.einsum('ij,kj->ik',xyz,self.mount[:3,:3])+self.mount[:3,3]
                    pose=[matrix[0,3],matrix[1,3],math.atan2(matrix[1,0],matrix[0,0])]
                    self.core.update(base,self.origin_sensor,pose,now)
                    self.map_transform=np.array([[reg[0,0],reg[0,1],reg[0,3]],[reg[1,0],reg[1,1],reg[1,3]],[0.,0.,1.]])
                    self.source_stamp=cloud.header.stamp;self.source_wall=stamp;self.pending=None
                except TransformException:error='waiting_for_capture_time_tf'
                except (ValueError,TypeError) as exc:self.pending=None;error=str(exc)
        if self.core.lo is None:
            self.status.publish(String(data=json.dumps({'fresh':False,'error':error or 'awaiting_raw_cloud',
                'reference_received':self.reference is not None,'pending_points':int(self.pending.width) if self.pending is not None else 0})))
            return
        fresh=self.source_wall is not None and 0<=wall-self.source_wall<=.35
        cells=self.core.view(now) if fresh else np.full((self.core.n,self.core.n),-1,dtype=np.int8)
        origin=self.core.lo*self.core.res
        msg=OccupancyGrid();msg.header.frame_id='odom';msg.header.stamp=self.source_stamp
        msg.info.resolution=self.core.res;msg.info.width=self.core.n;msg.info.height=self.core.n
        msg.info.origin.position.x=float(origin[0]);msg.info.origin.position.y=float(origin[1]);msg.info.origin.orientation.w=1.
        msg.data=array('b',cells.tobytes());self.live_pub.publish(msg)
        self.counter+=1
        if self.reference is not None and self.map_transform is not None and self.counter%5==0:
            ref=self.reference;shape=(ref.info.height,ref.info.width)
            grid=np.asarray(ref.data,dtype=np.int8).reshape(shape)
            fused=fuse_reference(grid,ref.info.resolution,[ref.info.origin.position.x,ref.info.origin.position.y],
                                 cells,origin,self.core.res,self.map_transform,int(self.config['realtime_navigation']['reference_obstacle_cost']))
            out=OccupancyGrid();out.header.frame_id='map';out.header.stamp=self.source_stamp;out.info=ref.info
            out.data=array('b',fused.tobytes());self.fused_pub.publish(out)
        self.status.publish(String(data=json.dumps({'fresh':fresh,'source_stamp':self.source_wall,'reference_is_advisory':True,
            'live_free':int((cells==0).sum()),'live_obstacles':int((cells==100).sum()),'unknown':int((cells<0).sum()),
            'vertical_bands_required':self.core.bands,'error':error})))


def main():
    rclpy.init();node=LiveMaps()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
