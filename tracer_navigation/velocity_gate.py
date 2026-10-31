#!/usr/bin/env python3
"""Independent final gate. Dry-run topic by default; always starts disarmed."""
import json
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool,String
from std_srvs.srv import SetBool
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid,Odometry
import numpy as np
from rclpy.qos import qos_profile_sensor_data

from common import CommandGate,settings,acceptance_error,cloud_xyz,near_stop_count,hardware_fault_requires_rearm
from live_grid import swept_permission


class Gate(Node):
    def __init__(self):
        super().__init__('navigation_velocity_gate')
        self.declare_parameter('enable_drive',False)
        self.drive=bool(self.get_parameter('enable_drive').value)
        self.core=CommandGate(settings())
        self.command=(0.,0.);self.ages={k:float('-inf') for k in ('command','cloud','pose','feedback')}
        self.valid={k:False for k in ('cloud','pose','feedback')}
        self.create_subscription(Twist,'/cmd_vel_collision_checked',self.cmd,10)
        for topic,key in (('/navigation/perception_valid','cloud'),('/navigation/localization_valid','pose'),('/tracer/ready','feedback')):
            self.create_subscription(Bool,topic,lambda msg,k=key:self.health(k,msg.data),10)
        self.output=self.create_publisher(Twist,'/tracer/cmd_vel' if self.drive else '/nav_cmd_preview',10)
        self.status=self.create_publisher(String,'/navigation/safety_status',10)
        self.create_service(SetBool,'/navigation/arm',self.arm)
        self.create_service(SetBool,'/navigation/estop',self.estop)
        self.create_timer(.02,self.tick)
        self.counter=0
        self.latched_fault=None;self.arm_grace_deadline=0.;self.fault_context=None
        self.actual_cloud_time=float('-inf');self.near_points=0
        self.create_subscription(PointCloud2,'/points_navigation',self.cloud,qos_profile_sensor_data)
        self.live_enabled=self.core.config.get('realtime_navigation',{}).get('enabled',False)
        self.live_grid=None;self.live_pose=None;self.live_reason='live_grid_missing'
        if self.live_enabled:
            from rclpy.qos import QoSProfile,DurabilityPolicy
            self.create_subscription(OccupancyGrid,'/navigation/live_map',self.live_map,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(Odometry,'/lio/odom',self.live_odom,10)

    def live_map(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        q=msg.info.origin.orientation
        if (msg.header.frame_id!='odom' or msg.info.resolution<=0 or
            msg.info.width*msg.info.height!=len(msg.data) or abs(q.x)+abs(q.y)+abs(q.z)>1e-6 or abs(q.w-1.)>1e-6 or
            not 0<=time.time()-stamp<=.35 or not np.isin(np.asarray(msg.data),[-1,0,100]).all()):
            self.live_grid=None;return
        self.live_grid=(msg,time.monotonic())

    def live_odom(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        q=msg.pose.pose.orientation;p=msg.pose.pose.position
        if msg.header.frame_id!='odom' or msg.child_frame_id!='base_link' or not 0<=time.time()-stamp<=.35:
            self.live_pose=None;return
        if not np.isfinite([p.x,p.y,p.z,q.x,q.y,q.z,q.w]).all() or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>.01:
            self.live_pose=None;return
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.live_pose=([p.x,p.y,yaw],stamp,time.monotonic())

    def live_permission(self,v,w):
        now=time.monotonic()
        if not self.live_grid or now-self.live_grid[1]>.35:return 'live_grid_stale'
        if not self.live_pose or now-self.live_pose[2]>.35:return 'live_pose_stale'
        msg=self.live_grid[0];stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if not 0<=time.time()-stamp<=.35:return 'live_grid_stale'
        if abs(stamp-self.live_pose[1])>.25:return 'live_pose_misaligned'
        grid=np.asarray(msg.data,dtype=np.int8).reshape(msg.info.height,msg.info.width)
        return swept_permission(grid,[msg.info.origin.position.x,msg.info.origin.position.y],msg.info.resolution,
                                self.live_pose[0],(v,w),self.core.config)

    def cloud(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if msg.header.frame_id!='base_link' or abs(time.time()-stamp)>.25:return
        try:self.near_points=near_stop_count(cloud_xyz(msg),self.core.config);self.actual_cloud_time=time.monotonic()
        except (ValueError,TypeError):self.actual_cloud_time=float('-inf')

    def health(self,key,value):
        self.ages[key]=time.monotonic();self.valid[key]=bool(value)

    def cmd(self,msg):
        # Reject unsupported lateral/3D commands instead of silently forwarding.
        values=[msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z]
        valid=all(math.isfinite(v) for v in values) and all(abs(v)<1e-8 for v in (msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y))
        self.command=(msg.linear.x,msg.angular.z) if valid else (float('nan'),0.)
        self.ages['command']=time.monotonic()

    def arm(self,req,response):
        if not req.data:
            self.core.armed=False;response.success=True;response.message='Disarmed';return response
        config=self.core.config
        if self.core.estop:
            response.message='Reset latched estop first';return response
        if self.drive and (not config['mount']['verified'] or config['robot']['height_m'] is None):
            response.message='Measure mounting/whole robot height and verify robot.yaml first';return response
        if self.drive and acceptance_error(config):
            response.message='Site acceptance: '+acceptance_error(config);return response
        if not self.valid['pose'] or not self.valid['cloud'] or (self.drive and not self.valid['feedback']):
            response.message='Localization, pointcloud or CAN feedback not ready';return response
        now=time.monotonic();nav=config['navigation']
        if now-self.actual_cloud_time>.25:response.message='Actual pointcloud stale';return response
        if self.near_points>1:response.message=f'Clear near-stop zone first: {self.near_points} points';return response
        if self.live_enabled and (not self.live_grid or not self.live_pose):
            response.message='Live grid/odometry not ready';return response
        if self.live_enabled:
            check=self.live_permission(0.,0.)
            if check not in ('ok','live_unknown','live_obstacle') or self.count_publishers('/navigation/live_map')!=1:
                response.message='Live map not fresh or publisher ambiguous: '+check;return response
        for key,timeout in (('pose',nav['localization_timeout_s']),('cloud',nav['perception_timeout_s']),('feedback',nav['feedback_timeout_s'])):
            if key=='feedback' and not self.drive:continue
            if now-self.ages[key]>timeout:response.message=key+' heartbeat stale';return response
        self.latched_fault=None;self.arm_grace_deadline=now+2.;self.fault_context=None
        self.core.armed=True;response.success=True;response.message='Armed '+('hardware' if self.drive else 'dry-run only');return response

    def estop(self,req,response):
        self.core.estop=req.data;self.core.armed=False
        response.success=True;response.message='Estop latched' if req.data else 'Estop reset; still disarmed'
        return response

    def tick(self):
        now=time.monotonic();ages={k:now-v for k,v in self.ages.items()}
        healthy=self.valid['cloud'] and self.valid['pose'] and (self.valid['feedback'] or not self.drive)
        healthy=healthy and now-self.actual_cloud_time<=.25
        v,w,reason=self.core.evaluate(self.command,ages,healthy,self.drive)
        if reason=='ok' and self.near_points>1:v=w=0.;reason='near_obstacle';self.core.last=(0.,0.)
        if self.live_enabled and reason=='ok':
            self.live_reason=self.live_permission(v,w)
            if self.live_reason!='ok':v=w=0.;reason=self.live_reason;self.core.last=(0.,0.)
        if hardware_fault_requires_rearm(reason,self.core.armed,self.drive,now,self.arm_grace_deadline):
            self.fault_context={'timestamp':time.time(),'reason':reason,'inputs_valid':dict(self.valid),
                'actual_cloud_age_s':now-self.actual_cloud_time,'ages':dict(ages),'near_stop_points':self.near_points}
            self.core.armed=False;self.latched_fault=reason
        if self.latched_fault:
            v=w=0.;reason='latched_'+self.latched_fault
        msg=Twist();msg.linear.x=v;msg.angular.z=w
        try:self.output.publish(msg)
        except rclpy._rclpy_pybind11.RCLError:
            if rclpy.ok():raise
            return
        self.counter+=1
        if self.counter%5==0:
            self.status.publish(String(data=json.dumps({'mode':'hardware' if self.drive else 'dry-run','armed':self.core.armed,'reason':reason,'linear':v,'angular':w,'near_stop_points':self.near_points,
                'inputs_valid':dict(self.valid),'actual_cloud_age_s':now-self.actual_cloud_time if math.isfinite(self.actual_cloud_time) else None,
                'fault_context':self.fault_context,'live_evidence_required':self.live_enabled,'live_reason':self.live_reason,
                'ages':{k:round(a,2) if math.isfinite(a) else None for k,a in ages.items()}})))


if __name__=='__main__':
    rclpy.init();node=Gate()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.core.armed=False
        if rclpy.ok():node.tick()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
