"""Actual Humble/Nav2 + ideal synthetic lidar in domain 43; no CAN or hardware backend."""
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import threading
import numpy as np
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from common import ROOT,settings
from integration_test import Probe
from test_live_navigation import room_rays


class FloorProbe(Probe):
    def __init__(self):
        self.simulating=False;self.sim_command=(0.,0.);self.sim_stamp=0.;self.sim_step=time.monotonic()
        super().__init__();self.position=np.array([.1,2.1]);self.live=None;self.safety=None;self.map_status=None
        self.create_subscription(Twist,'/nav_cmd_preview',self.sim_receive,10)
        self.raw_cloud=self.create_publisher(PointCloud2,'/lidar/points',qos_profile_sensor_data)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid,'/navigation/live_map',lambda m:setattr(self,'live',m),qos)
        self.create_subscription(String,'/navigation/safety_status',lambda m:setattr(self,'safety',json.loads(m.data)),10)
        self.create_subscription(String,'/navigation/live_map_status',lambda m:setattr(self,'map_status',json.loads(m.data)),10)
        self.scene_config=settings();self.complete=self.ideal_room_scan()
        self.floor=self.complete.astype('<f4')

    def ideal_room_scan(self):
        # Fixed world room, dense ideal observations targeting voxel centres and
        # floor corners. Unlike a real MID360, this fixture has complete visibility.
        c=self.scene_config;res=c['realtime_navigation']['resolution_m'];band=c['realtime_navigation']['vertical_band_m']
        low=np.floor((self.position-np.array([.7,.7]))/res).astype(int)
        high=np.ceil((self.position+np.array([1.7,.7]))/res).astype(int)
        yy,xx=np.mgrid[low[1]:high[1],low[0]:high[0]]
        xy=(np.column_stack([xx.ravel(),yy.ravel()])+.5)*res
        heights=np.arange(math.ceil(c['robot']['height_m']/band))*band+band*.85
        targets=np.column_stack([np.repeat(xy,len(heights),axis=0),np.tile(heights,len(xy))])
        rotation=np.array([[math.cos(self.heading),-math.sin(self.heading)],[math.sin(self.heading),math.cos(self.heading)]])
        sensor=np.array([c['mount'][k] for k in ('x_m','y_m','z_m')]);sensor[:2]=rotation@sensor[:2]+self.position
        vectors=targets-sensor
        bounds=[(-4.9,5.1),(-2.9,7.1),(0.,2.8)]
        candidates=np.full((len(vectors),3),np.inf)
        for axis,(lo,hi) in enumerate(bounds):
            positive=vectors[:,axis]>1e-9;negative=vectors[:,axis]<-1e-9
            candidates[positive,axis]=(hi-sensor[axis])/vectors[positive,axis]
            candidates[negative,axis]=(lo-sensor[axis])/vectors[negative,axis]
        endpoints=sensor+vectors*candidates.min(axis=1)[:,None]
        ground=np.concatenate([np.column_stack([xy+np.array([x,y])*res,np.zeros(len(xy))])
            for x,y in ((-.48,-.48),(-.48,.48),(.48,-.48),(.48,.48),(0,0))])
        # Grid cells can straddle the body boundary. Observe their exposed floor
        # slivers explicitly rather than inventing support underneath the robot.
        hx=c['robot']['length_m']/2+.00001;hy=c['robot']['width_m']/2+.00001
        edge=np.vstack([np.column_stack([np.full(80,x),np.linspace(-hy,hy,80)]) for x in (-hx,hx)]+
                       [np.column_stack([np.linspace(-hx,hx,80),np.full(80,y)]) for y in (-hy,hy)])
        edge_world=np.einsum('ij,kj->ik',edge,rotation)+self.position
        ground=np.vstack([ground,np.column_stack([edge_world,np.zeros(len(edge_world))])])
        points=np.vstack([endpoints,ground]);points[:,:2]=np.einsum('ij,jk->ik',points[:,:2]-self.position,rotation)
        return points

    def publish(self):
        now=time.monotonic();dt=min(.1,now-self.sim_step);self.sim_step=now
        if self.simulating:
            v,w=self.sim_command if now-self.sim_stamp<.25 else (0.,0.)
            self.position+=np.array([math.cos(self.heading),math.sin(self.heading)])*v*dt
            self.heading+=w*dt;self.velocity=(v,w)
            self.floor=self.ideal_room_scan().astype('<f4')
        previous=self.previous_cloud
        super().publish()
        if self.cloud_on and previous:self.raw_cloud.publish(previous[1])

    def sim_receive(self,msg):
        self.sim_command=(msg.linear.x,msg.angular.z);self.sim_stamp=time.monotonic()

    def until(self,predicate,seconds=20):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            time.sleep(.02)
            if predicate():return
        raise TimeoutError('Condition timed out; latest safety='+str(self.safety)+'; live_map='+str(self.map_status))

    def wait(self,seconds):time.sleep(seconds)

    def await_future(self,future,timeout=20):
        self.until(future.done,timeout)
        return future.result()


def main():
    if subprocess.run(['pgrep','-f','[/]tracer_safe_base( |$)|[f]astlio2_native'],capture_output=True).returncode==0:
        raise SystemExit('Real hardware/backend process found; refuse offline test')
    os.environ['ROS_DOMAIN_ID']='43';os.environ['ROS_LOCALHOST_ONLY']='1'
    folder=ROOT/'results/floor_navigation_20261031';folder.mkdir(parents=True,exist_ok=True)
    log=(folder/'nav2_live_launch.log').open('w')
    launch=subprocess.Popen(['ros2','launch',str(ROOT/'navigation.launch.py'),'bridge:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=FloorProbe();report={'offline_only':True,'hardware_started':False,'domain':43,'checks':{}}
    executor=rclpy.executors.SingleThreadedExecutor();executor.add_node(node)
    worker=threading.Thread(target=executor.spin,daemon=True);worker.start()
    try:
        node.until(lambda:node.live is not None and node.safety is not None and node.action.server_is_ready(),40)
        node.wait(2.)
        req=SetBool.Request();req.data=True
        if not node.arm.wait_for_service(timeout_sec=3):raise RuntimeError('Gate not available')
        armed=node.await_future(node.arm.call_async(req));assert armed.success,armed.message
        node.until(lambda:node.safety and node.safety.get('linear',0)>.001,12)
        report['checks']['fresh_full_height_rays_allow_dry_run']=True
        path=node.path();report['checks']['actual_nav2_tentative_route']=len(path)>2
        node.obstacle=.75
        node.until(lambda:node.safety and node.safety.get('reason') in ('live_obstacle','near_obstacle') and node.safety.get('linear')==0.,8)
        report['checks']['new_obstacle_stops_output']=True
        node.obstacle=None
        node.until(lambda:node.safety and node.safety.get('linear',0)>.001,8)
        report['checks']['removed_obstacle_clears_only_after_new_rays']=True
        node.floor=node.complete[np.abs(node.complete[:,2])<.01].astype('<f4')
        node.until(lambda:node.safety and node.safety.get('reason')=='live_unknown' and node.safety.get('linear')==0.,8)
        report['checks']['ground_without_current_height_coverage_stops']=True
        node.floor=node.complete.astype('<f4')
        node.until(lambda:node.safety and node.safety.get('linear',0)>.001,8)
        report['checks']['fresh_coverage_recovers_dry_run']=True
        node.cloud_on=False
        node.until(lambda:node.safety and node.safety.get('linear')==0.,3)
        report['checks']['input_loss_stops']=True
        report['checks']['no_physical_publishers']=all(node.count_publishers(t)==0 for t in ('/tracer/cmd_vel','/tracer/ready'))
        node.cloud_on=True;node.command_on=False
        req.data=False;node.await_future(node.arm.call_async(req));node.wait(1.)
        client=ActionClient(node,NavigateToPose,'/navigate_to_pose')
        if not client.wait_for_server(timeout_sec=4):raise RuntimeError('Navigator unavailable')
        target=NavigateToPose.Goal();target.pose.header.frame_id='map';target.pose.header.stamp=node.get_clock().now().to_msg()
        goal=node.position+np.array([.6,0.]);target.pose.pose.position.x=float(goal[0]);target.pose.pose.position.y=float(goal[1]);target.pose.pose.orientation.w=1.
        handle=node.await_future(client.send_goal_async(target));assert handle.accepted
        result=handle.get_result_async();node.simulating=True
        node.until(lambda:node.safety and (node.safety.get('actual_cloud_age_s') or 99)<.15,5)
        req.data=True;armed=node.await_future(node.arm.call_async(req));assert armed.success,armed.message
        node.until(lambda:result.done(),75)
        error=float(np.linalg.norm(node.position-goal));node.wait(.5)
        report['model_arrival_error_m']=error;report['model_action_status']=result.result().status
        report['checks']['nav2_closed_loop_arrival_model']=result.result().status==4 and error<=.2
        report['checks']['nav2_model_stops_after_arrival']=abs(node.sim_command[0])<.001 and abs(node.sim_command[1])<.001
        req.data=False;node.await_future(node.arm.call_async(req))
        report['passed']=all(report['checks'].values())
    except Exception as exc:
        report.update(passed=False,error=repr(exc),last_safety=node.safety)
        if node.live is not None:
            np.savez(folder/'last_live_grid.npz',grid=np.asarray(node.live.data).reshape(node.live.info.height,node.live.info.width),
                     origin=[node.live.info.origin.position.x,node.live.info.origin.position.y],pose=[*node.position,node.heading],
                     resolution=node.live.info.resolution)
    finally:
        if launch.poll() is None:
            launch.send_signal(signal.SIGINT)
            try:launch.wait(timeout=15)
            except subprocess.TimeoutExpired:os.killpg(launch.pid,signal.SIGKILL);launch.wait()
        executor.shutdown(timeout_sec=2);worker.join(timeout=3)
        node.destroy_node();rclpy.shutdown();log.close()
        (folder/'ros_live_integration.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2));raise SystemExit(0 if report.get('passed') else 1)


if __name__=='__main__':main()
