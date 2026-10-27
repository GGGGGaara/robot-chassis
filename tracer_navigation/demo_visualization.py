"""Synthetic ROS visualization only. No CAN access or velocity publishers."""
import math
import os
import struct
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped,TransformStamped
from nav_msgs.msg import OccupancyGrid,Odometry,Path
from sensor_msgs.msg import PointCloud2,PointField
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster


class Demo(Node):
    def __init__(self):
        if os.environ.get('ROS_DOMAIN_ID')!='74' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
            raise RuntimeError('Demo requires isolated localhost ROS_DOMAIN_ID=74')
        super().__init__('tracer_offline_visual_demo')
        self.map_pub=self.create_publisher(OccupancyGrid,'/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.points_pub=self.create_publisher(PointCloud2,'/mapping/global_points',10)
        self.obstacles_pub=self.create_publisher(PointCloud2,'/points_navigation',qos_profile_sensor_data)
        self.path_pub=self.create_publisher(Path,'/plan',10)
        self.odom_pub=self.create_publisher(Odometry,'/lio/odom',10)
        self.status_pub=self.create_publisher(String,'/team/demo_status',10)
        self.tf=TransformBroadcaster(self)
        self.static_tf=StaticTransformBroadcaster(self)
        fixed=TransformStamped();fixed.header.stamp=self.get_clock().now().to_msg()
        fixed.header.frame_id='map';fixed.child_frame_id='odom';fixed.transform.rotation.w=1.
        self.static_tf.sendTransform(fixed)
        self.phase=0.
        self.create_timer(.2,self.tick)

    def tick(self):
        stamp=self.get_clock().now().to_msg();self.phase+=.02
        grid=OccupancyGrid();grid.header.frame_id='map';grid.header.stamp=stamp
        grid.info.resolution=.1;grid.info.width=60;grid.info.height=60
        grid.info.origin.position.x=-3.;grid.info.origin.position.y=-3.;grid.info.origin.orientation.w=1.
        grid.data=[100 if x in (0,59) or y in (0,59) or (28<=x<33 and 34<=y<40) else 0 for y in range(60) for x in range(60)]
        self.map_pub.publish(grid)
        points=[]
        for i in range(60):
            d=-3.+i*.1
            for z in (.1,.3,.5,.8,1.2):points.extend([(d,-3.,z),(d,2.9,z),(-3.,d,z),(2.9,d,z)])
        cloud=PointCloud2();cloud.header=grid.header;cloud.height=1;cloud.width=len(points)
        cloud.fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,k in enumerate('xyz')]
        cloud.is_bigendian=False;cloud.is_dense=True;cloud.point_step=12;cloud.row_step=12*len(points)
        cloud.data=b''.join(struct.pack('<fff',*p) for p in points);self.points_pub.publish(cloud)
        obstacles=PointCloud2();obstacles.header.frame_id='base_link';obstacles.header.stamp=stamp
        obstacles.height=1;obstacles.fields=cloud.fields;obstacles.point_step=12
        box=[(x,y,z) for x in (1.,1.2) for y in (.4,.6) for z in (.2,.4,.6,.8)]
        obstacles.width=len(box);obstacles.row_step=12*len(box);obstacles.is_dense=True
        obstacles.data=b''.join(struct.pack('<fff',*p) for p in box);self.obstacles_pub.publish(obstacles)
        path=Path();path.header=grid.header
        for i in range(25):
            pose=PoseStamped();pose.header=grid.header;pose.pose.position.x=-1.2+i*.1
            pose.pose.position.y=-1.;pose.pose.orientation.w=1.;path.poses.append(pose)
        self.path_pub.publish(path)
        odom=Odometry();odom.header.frame_id='odom';odom.header.stamp=stamp;odom.child_frame_id='base_link'
        odom.pose.pose.position.x=.4*math.sin(self.phase);odom.pose.pose.position.y=-1.
        odom.pose.pose.orientation.w=1.;self.odom_pub.publish(odom)
        moving=TransformStamped();moving.header=odom.header;moving.child_frame_id='base_link'
        moving.transform.translation.x=odom.pose.pose.position.x;moving.transform.translation.y=-1.
        moving.transform.rotation.w=1.;self.tf.sendTransform(moving)
        self.status_pub.publish(String(data='SYNTHETIC_VISUALIZATION_ONLY; not sensor data or navigation acceptance'))


def main():
    rclpy.init();node=None
    try:node=Demo();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
