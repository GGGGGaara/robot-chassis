#!/usr/bin/env python3
"""Subscribe to DimOS LCM and publish planar ROS TF/odom + live 3D observations."""
from collections import deque
import math
import re
import threading
import time

import lcm
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from scipy.spatial.transform import Rotation

from common import settings,lcm_types,mount_matrix,transform_matrix,planar,cloud_xyz,filter_cloud


class Bridge(Node):
    def __init__(self):
        super().__init__('dimos_ros_bridge')
        self.config=settings()
        self.TF,self.Cloud,_=lcm_types(self.config)
        self.mount=mount_matrix(self.config)
        self.lock=threading.Lock()
        self.poses=deque(maxlen=200)
        self.fix=None
        self.pending=deque(maxlen=20)
        self.last_pose=0.
        self.last_cloud=0.
        self.cloud_ok=False
        self.previous=None
        self.tf=TransformBroadcaster(self)
        self.static=StaticTransformBroadcaster(self)
        self.static.sendTransform(self.tf_msg(self.mount,'base_link','mid360_link',self.get_clock().now().to_msg()))
        self.odom=self.create_publisher(Odometry,'/lio/odom',20)
        self.points=self.create_publisher(PointCloud2,'/points_navigation',qos_profile_sensor_data)
        self.sensor_points=self.create_publisher(PointCloud2,'/points_navigation_sensor',qos_profile_sensor_data)
        self.raw_points=self.create_publisher(PointCloud2,'/lidar/points',qos_profile_sensor_data)
        self.loc=self.create_publisher(Bool,'/navigation/localization_valid',10)
        self.perception=self.create_publisher(Bool,'/navigation/perception_valid',10)
        self.create_timer(.02,self.tick)
        self.running=True
        threading.Thread(target=self.listen,daemon=True).start()
        self.get_logger().info('LCM subscribe-only bridge: approximate 45-degree forward mounting; base TF owns odom->base_link')

    def listen(self):
        bus=lcm.LCM()
        def tf(channel,raw):
            for item in self.TF.lcm_decode(raw).transforms:
                stamp=item.header.stamp.sec+item.header.stamp.nsec*1e-9
                if abs(time.time()-stamp)>.5: continue
                pair=(item.header.frame_id,item.child_frame_id)
                with self.lock:
                    if pair==('odom','mid360_link'):
                        self.poses.append((stamp,transform_matrix(item.transform),time.monotonic()))
                    elif pair==('odom','map'):
                        self.fix=(transform_matrix(item.transform),time.monotonic())
        def cloud(channel,raw):
            with self.lock: self.pending.append((raw,time.monotonic()))
        bus.subscribe(re.escape('/tf#tf2_msgs.TFMessage'),tf)
        bus.subscribe(re.escape('/lidar#sensor_msgs.PointCloud2'),cloud)
        while self.running:
            bus.handle_timeout(100)

    @staticmethod
    def tf_msg(matrix,parent,child,stamp):
        msg=TransformStamped();msg.header.stamp=stamp
        msg.header.frame_id=parent;msg.child_frame_id=child
        t=matrix[:3,3];q=Rotation.from_matrix(matrix[:3,:3]).as_quat()
        msg.transform.translation.x=float(t[0]);msg.transform.translation.y=float(t[1]);msg.transform.translation.z=float(t[2])
        msg.transform.rotation.x=float(q[0]);msg.transform.rotation.y=float(q[1]);msg.transform.rotation.z=float(q[2]);msg.transform.rotation.w=float(q[3])
        return msg

    def tick(self):
        now=time.monotonic()
        with self.lock:
            poses=list(self.poses);fix=self.fix;pending=None
            # Publish the original timestamp only after corresponding odometry
            # has had time to arrive and reach ROS TF consumers. Never restamp
            # an old cloud as new or extrapolate the robot pose into the future.
            while self.pending and now-self.pending[0][1]>=.08:
                pending=self.pending.popleft()
        good_pose=False
        if poses:
            stamp,raw,received=poses[-1]
            good_pose=now-received<.3 and fix is not None and now-fix[1]<.5
            if stamp!=self.last_pose:
                self.last_pose=stamp
                current=planar(raw@np.linalg.inv(self.mount))
                ros_stamp=self.get_clock().now().to_msg()
                ros_stamp.sec=int(stamp);ros_stamp.nanosec=int((stamp-int(stamp))*1e9)
                odom_tf=self.tf_msg(current,'odom','base_link',ros_stamp)
                transforms=[odom_tf]
                if good_pose:
                    placed=planar(np.linalg.inv(fix[0])@raw@np.linalg.inv(self.mount))
                    transforms.append(self.tf_msg(placed@np.linalg.inv(current),'map','odom',ros_stamp))
                self.tf.sendTransform(transforms)
                msg=Odometry();msg.header=odom_tf.header;msg.child_frame_id='base_link'
                msg.pose.pose.position.x=float(current[0,3]);msg.pose.pose.position.y=float(current[1,3]);msg.pose.pose.orientation=odom_tf.transform.rotation
                if self.previous:
                    old_stamp,old=self.previous;dt=stamp-old_stamp
                    if .005<dt<.5:
                        change=np.linalg.inv(old)@current
                        msg.twist.twist.linear.x=float(change[0,3]/dt)
                        msg.twist.twist.angular.z=float(math.atan2(change[1,0],change[0,0])/dt)
                self.previous=(stamp,current)
                self.odom.publish(msg)
        if pending:
            try:
                cloud=self.Cloud.lcm_decode(pending[0]);stamp=cloud.header.stamp.sec+cloud.header.stamp.nsec*1e-9
                xyz=cloud_xyz(cloud)
                # We only accept this known sensor frame, never relabel world points.
                self.cloud_ok=(cloud.header.frame_id=='mid360_link' and len(xyz)>=200 and abs(time.time()-stamp)<.35)
                if self.cloud_ok:
                    self.raw_points.publish(self.ros_cloud(xyz,'mid360_link',stamp))
                    base=filter_cloud(xyz,self.config)
                    msg=PointCloud2();msg.header.frame_id='base_link'
                    msg.header.stamp.sec=int(stamp);msg.header.stamp.nanosec=int((stamp-int(stamp))*1e9)
                    msg.height=1;msg.width=len(base);msg.is_bigendian=False;msg.is_dense=True
                    msg.fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,k in enumerate(('x','y','z'))]
                    msg.point_step=12;msg.row_step=12*len(base);msg.data=base.astype('<f4').tobytes()
                    self.points.publish(msg);self.last_cloud=now
                    # Costmaps receive the same filtered returns in the original
                    # sensor frame, preserving the true ray origin without a
                    # second MessageFilter target for the mounting transform.
                    sensor=(base-self.mount[:3,3])@self.mount[:3,:3]
                    sensor_msg=PointCloud2();sensor_msg.header.stamp=msg.header.stamp
                    sensor_msg.header.frame_id='mid360_link';sensor_msg.height=1
                    sensor_msg.width=len(sensor);sensor_msg.is_bigendian=False;sensor_msg.is_dense=True
                    sensor_msg.fields=msg.fields;sensor_msg.point_step=12;sensor_msg.row_step=12*len(sensor)
                    sensor_msg.data=sensor.astype('<f4').tobytes();self.sensor_points.publish(sensor_msg)
            except Exception as error:
                self.cloud_ok=False
                self.get_logger().error(f'Cloud rejected: {error}')
        self.loc.publish(Bool(data=good_pose))
        self.perception.publish(Bool(data=self.cloud_ok and now-self.last_cloud<.35))
    @staticmethod
    def ros_cloud(xyz,frame,stamp):
        msg=PointCloud2();msg.header.frame_id=frame
        msg.header.stamp.sec=int(stamp);msg.header.stamp.nanosec=int((stamp-int(stamp))*1e9)
        msg.height=1;msg.width=len(xyz);msg.is_dense=True
        msg.fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,k in enumerate(('x','y','z'))]
        msg.point_step=12;msg.row_step=12*len(xyz);msg.data=np.asarray(xyz,dtype='<f4').tobytes()
        return msg


def main():
    rclpy.init();node=Bridge()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.running=False;node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__': main()
