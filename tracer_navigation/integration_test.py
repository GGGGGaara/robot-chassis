#!/usr/bin/env python3
"""Real Nav2 nodes with synthetic sensors in isolated domain 43, no CAN driver."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from geometry_msgs.msg import TransformStamped,Twist,PoseStamped
from sensor_msgs.msg import PointCloud2,PointField
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
from common import ROOT,settings,mount_matrix
from scipy.spatial.transform import Rotation


class Probe(Node):
    def __init__(self):
        super().__init__('navigation_integration_probe')
        self.tf=TransformBroadcaster(self);self.static=StaticTransformBroadcaster(self)
        self.cloud=self.create_publisher(PointCloud2,'/points_navigation',qos_profile_sensor_data)
        self.sensor_cloud=self.create_publisher(PointCloud2,'/points_navigation_sensor',qos_profile_sensor_data)
        self.loc=self.create_publisher(Bool,'/navigation/localization_valid',10)
        self.perception=self.create_publisher(Bool,'/navigation/perception_valid',10)
        self.command=self.create_publisher(Twist,'/cmd_vel_nav',10)
        self.last_checked=None;self.last_preview=None
        self.create_subscription(Twist,'/cmd_vel_collision_checked',lambda m:setattr(self,'last_checked',(time.monotonic(),m.linear.x)),10)
        self.create_subscription(Twist,'/nav_cmd_preview',lambda m:setattr(self,'last_preview',(time.monotonic(),m.linear.x)),10)
        self.obstacle=None;self.cloud_on=True;self.loc_on=True;self.command_on=True
        self.previous_cloud=None
        self.mount=mount_matrix(settings())
        self.position=np.array([.1,2.08],dtype=float);self.heading=0.;self.velocity=(0.,0.)
        self.odom=self.create_publisher(Odometry,'/lio/odom',10)
        self.create_timer(.05,self.publish)
        static=self.transform(mount_matrix(settings()),'base_link','mid360_link');self.static.sendTransform(static)
        self.action=ActionClient(self,ComputePathToPose,'/compute_path_to_pose')
        self.arm=self.create_client(SetBool,'/navigation/arm')
        self.floor=np.array([[x,y,0.] for x in np.linspace(1.,5.,25) for y in np.linspace(-2.,2.,17)],dtype='<f4')

    def transform(self,matrix,parent,child):
        msg=TransformStamped();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id=parent;msg.child_frame_id=child
        q=Rotation.from_matrix(matrix[:3,:3]).as_quat()
        msg.transform.translation.x=float(matrix[0,3]);msg.transform.translation.y=float(matrix[1,3]);msg.transform.translation.z=float(matrix[2,3])
        msg.transform.rotation.x=float(q[0]);msg.transform.rotation.y=float(q[1]);msg.transform.rotation.z=float(q[2]);msg.transform.rotation.w=float(q[3])
        return msg

    def publish(self):
        if self.loc_on:
            pose=np.eye(4);pose[:2,3]=self.position
            pose[:3,:3]=Rotation.from_euler('z',self.heading).as_matrix()
            self.tf.sendTransform([self.transform(np.eye(4),'map','odom'),self.transform(pose,'odom','base_link')])
            odom=Odometry();odom.header.stamp=self.get_clock().now().to_msg();odom.header.frame_id='odom';odom.child_frame_id='base_link'
            odom.pose.pose.position.x=float(self.position[0]);odom.pose.pose.position.y=float(self.position[1])
            odom.pose.pose.orientation.z=math.sin(self.heading/2);odom.pose.pose.orientation.w=math.cos(self.heading/2)
            odom.twist.twist.linear.x=self.velocity[0];odom.twist.twist.angular.z=self.velocity[1];self.odom.publish(odom)
            self.loc.publish(Bool(data=True))
        if self.cloud_on:
            xyz=self.floor
            if self.obstacle is not None:
                box=np.array([[self.obstacle,y,z] for y in np.linspace(-.12,.12,7) for z in np.linspace(.1,.5,7)],dtype='<f4')
                xyz=np.concatenate([xyz,box])
            msg=PointCloud2();msg.header.frame_id='base_link';msg.header.stamp=self.get_clock().now().to_msg()
            msg.height=1;msg.width=len(xyz);msg.point_step=12;msg.row_step=12*len(xyz);msg.is_dense=True
            msg.fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,k in enumerate(('x','y','z'))]
            msg.data=xyz.tobytes();self.perception.publish(Bool(data=True))
            mount=self.mount;sensor=(xyz-mount[:3,3])@mount[:3,:3]
            sensor_msg=PointCloud2();sensor_msg.header.stamp=msg.header.stamp;sensor_msg.header.frame_id='mid360_link'
            sensor_msg.height=1;sensor_msg.width=len(sensor);sensor_msg.point_step=12;sensor_msg.row_step=12*len(sensor)
            sensor_msg.is_dense=True;sensor_msg.fields=msg.fields;sensor_msg.data=sensor.astype('<f4').tobytes()
            if self.previous_cloud:
                old_base,old_sensor=self.previous_cloud
                self.cloud.publish(old_base);self.sensor_cloud.publish(old_sensor)
            self.previous_cloud=(msg,sensor_msg)
        else:self.previous_cloud=None
        if self.command_on:
            msg=Twist();msg.linear.x=.12;self.command.publish(msg)

    def wait(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.02)

    def await_future(self,future,timeout=20):
        end=time.monotonic()+timeout
        while not future.done() and time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.02)
        if not future.done():raise TimeoutError('ROS action/service timeout')
        return future.result()

    def path(self):
        goal=ComputePathToPose.Goal();goal.planner_id='GridBased';goal.use_start=True
        for pose,x,y in ((goal.start,.1,2.08),(goal.goal,3.,2.5)):
            pose.header.frame_id='map';pose.header.stamp=self.get_clock().now().to_msg()
            pose.pose.position.x=x;pose.pose.position.y=y;pose.pose.orientation.w=1.
        handle=self.await_future(self.action.send_goal_async(goal))
        if not handle.accepted:raise RuntimeError('Planner rejected goal')
        result=self.await_future(handle.get_result_async())
        if result.status!=4 or not result.result.path.poses:raise RuntimeError('Planner did not produce a path')
        return [[p.pose.position.x,p.pose.position.y] for p in result.result.path.poses]

    def value(self,which='last_checked'):
        result=getattr(self,which)
        if result is None or time.monotonic()-result[0]>.3:raise RuntimeError('Velocity output missing/stale')
        return result[1]


def main():
    os.environ['ROS_DOMAIN_ID']='43'
    env=dict(os.environ,ROS_DOMAIN_ID='43',ROS_LOCALHOST_ONLY='1')
    log=(ROOT/'integration_launch.log').open('w')
    launch=subprocess.Popen(['ros2','launch',str(ROOT/'navigation.launch.py'),'bridge:=false'],env=env,
                            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=Probe();report={'hardware_driver_started':False,'ros_domain':43,'checks':{}}
    try:
        deadline=time.monotonic()+40
        while not node.action.wait_for_server(timeout_sec=.1) and time.monotonic()<deadline:node.wait(.1)
        node.wait(4.)
        path=node.path();report['checks']['static_map_planning']=len(path)>1
        assert node.arm.wait_for_service(timeout_sec=5.)
        request=SetBool.Request();request.data=True
        response=node.await_future(node.arm.call_async(request));assert response.success,response.message
        node.wait(1.5)
        clear=node.value();assert .01<clear<=settings()['robot']['max_linear_mps']+1e-6,clear
        report['checks']['clear_area_motion_preview']=clear
        node.obstacle=1.;node.wait(1.)
        slow=node.value();assert 0.<slow<clear*.5,slow
        report['checks']['slowdown_zone']=slow
        node.obstacle=.60;node.wait(.8)
        stopped=node.value();assert abs(stopped)<1e-8
        assert abs(node.value('last_preview'))<1e-8
        report['checks']['near_obstacle_immediate_stop']=True
        request=SetBool.Request();request.data=False
        assert node.await_future(node.arm.call_async(request)).success
        request.data=True
        blocked=node.await_future(node.arm.call_async(request));assert not blocked.success,blocked.message
        report['checks']['arming_refused_with_near_obstacle']=True
        node.obstacle=1.4;node.wait(2.5)
        assert node.await_future(node.arm.call_async(request)).success
        path2=node.path()
        # The Nav2 global obstacle layer changes the route around live returns.
        report['checks']['dynamic_obstacle_path_changed']=path2!=path
        assert path2!=path
        node.obstacle=None;node.wait(1.)
        node.cloud_on=False;node.wait(.8)
        assert abs(node.value('last_preview'))<1e-8
        report['checks']['pointcloud_loss_stop']=True
        node.cloud_on=True;node.loc_on=False;node.wait(.8)
        assert abs(node.value('last_preview'))<1e-8
        report['checks']['localization_loss_stop']=True
        node.loc_on=True;node.command_on=False;node.wait(.8)
        assert abs(node.value('last_preview'))<1e-8
        report['checks']['command_timeout_stop']=True
        report['passed']=True
    except Exception as error:
        report['passed']=False;report['error']=repr(error)
        raise
    finally:
        # Keep synthetic transforms alive while costmaps stop their worker threads.
        try:
            client=node.create_client(ManageLifecycleNodes,'/navigation_lifecycle/manage_nodes')
            if client.wait_for_service(timeout_sec=2.):
                request=ManageLifecycleNodes.Request();request.command=4
                report['orderly_shutdown']=node.await_future(client.call_async(request),20).success
        except Exception as error:
            report['shutdown_error']=repr(error)
        (ROOT/'integration_report.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2),flush=True)
        node.destroy_node();rclpy.shutdown()
        os.killpg(launch.pid,signal.SIGINT)
        try:launch.wait(timeout=15)
        except subprocess.TimeoutExpired:os.killpg(launch.pid,signal.SIGTERM);launch.wait(timeout=5)
        log.close()


if __name__=='__main__':main()
