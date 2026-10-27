"""Real Nav2 + simulated odometry/pointcloud + kinematic robot. No CAN driver."""
import json
import math
import os
import signal
import subprocess
import time
import argparse
import socket
import struct
import numpy as np
import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from std_srvs.srv import SetBool
from ament_index_python.packages import get_package_prefix
from common import ROOT,require_virtual_can,settings
from integration_test import Probe
from coordinate_geometry import relative_goal


class Robot(Probe):
    def __init__(self,vcan=False):
        super().__init__();self.command_on=False;self.last_step=time.monotonic();self.input_command=(0.,0.);self.input_time=0.
        self.trace=[];self.phase='approach'
        self.bus=None;self.can_frames=0;self.last_system_feedback=0.
        if vcan:
            require_virtual_can()
            self.bus=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
            self.bus.bind(('vcan_navtest',));self.bus.setblocking(False)
            self.can_command=self.create_publisher(Twist,'/vcan_test/cmd_vel',10)
        self.create_subscription(Twist,'/nav_cmd_preview',self.drive,10)

    def drive(self,msg):
        if self.bus is not None:self.can_command.publish(msg);return
        self.input_command=(msg.linear.x,msg.angular.z);self.input_time=time.monotonic()

    def publish(self):
        now=time.monotonic();dt=min(.1,now-self.last_step);self.last_step=now
        if self.bus is not None:
            while True:
                try:cid,dlc,data=struct.unpack('=IB3x8s',self.bus.recv(16))
                except BlockingIOError:break
                if cid==0x111:
                    v,w,_,_=struct.unpack('>hhhh',data);self.input_command=(v/1000.,w/1000.);self.input_time=now;self.can_frames+=1
        v,w=self.input_command if now-self.input_time<.25 else (0.,0.)
        self.velocity=(v,w)
        self.position+=np.array([math.cos(self.heading),math.sin(self.heading)])*v*dt
        self.heading+=w*dt
        if self.bus is not None:
            if now-self.last_system_feedback>=.2:
                self.bus.send(struct.pack('=IB3x8s',0x211,8,bytes.fromhex('00010A5000000000')));self.last_system_feedback=now
            self.bus.send(struct.pack('=IB3x8s',0x221,8,struct.pack('>hhhh',int(v*1000),int(w*1000),0,0)))
        self.trace.append({'t':now,'x':float(self.position[0]),'y':float(self.position[1]),'yaw':self.heading,'v':v,'w':w,'phase':self.phase})
        super().publish()


def main():
    p=argparse.ArgumentParser();p.add_argument('--selected',action='store_true');p.add_argument('--vcan',action='store_true')
    p.add_argument('--relative',nargs=2,type=float,metavar=('FORWARD','LEFT'))
    p.add_argument('--soak-s',type=float,default=0.);args=p.parse_args()
    os.environ['ROS_DOMAIN_ID']='43';rclpy.init();node=Robot(args.vcan)
    if args.relative is not None and args.selected:raise ValueError('Choose relative or selected goal')
    suffix=('_vcan' if args.vcan else '')+('_relative' if args.relative is not None else '')
    log=(ROOT/f'closed_loop{suffix}_launch.log').open('w')
    launch=subprocess.Popen(['ros2','launch',str(ROOT/'navigation.launch.py'),'bridge:=false'],
        env=dict(os.environ,ROS_DOMAIN_ID='43'),stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    report={'simulation_only':True,'hardware_driver_started':False,'virtual_can_driver':args.vcan,'checks':{}};handle=None;driver=None;driver_log=None
    try:
        target=[.95,2.08];goal_yaw=0.
        if args.relative is not None:
            target=relative_goal([float(v) for v in node.position]+[node.heading],args.relative)
            goal_yaw=math.atan2(target[1]-node.position[1],target[0]-node.position[0])
        if args.selected:
            selected=json.loads((ROOT.parent/'realtime_mapping_usage/path_preview_results/latest_path.json').read_text())
            saved=json.loads((ROOT/'results/latest_nav2_path.json').read_text())
            target=selected['goal'];node.position=np.array(saved['points'][0]);node.heading=math.radians(-5.)
            goal_yaw=math.atan2(target[1]-node.position[1],target[0]-node.position[0])
        if args.vcan:
            executable=get_package_prefix('tracer_navigation_driver')+'/lib/tracer_navigation_driver/tracer_safe_base'
            driver_log=(ROOT/'closed_loop_vcan_driver.log').open('w')
            driver=subprocess.Popen([executable,'--ros-args','-p','enable_drive:=true','-p','can_interface:=vcan_navtest',
                '-p',f'max_linear_mps:={settings()["robot"]["max_linear_mps"]}',
                '-r','/tracer/cmd_vel:=/vcan_test/cmd_vel','-r','/tracer/ready:=/vcan_test/ready',
                '-r','/tracer_status:=/vcan_test/status','-r','/tracer/wheel_odom:=/vcan_test/odom',
                '-r','/tracer/driver_diagnostics:=/vcan_test/diagnostics'],
                env=dict(os.environ,ROS_DOMAIN_ID='43'),stdout=driver_log,stderr=subprocess.STDOUT)
        action=ActionClient(node,NavigateToPose,'/navigate_to_pose')
        deadline=time.monotonic()+40
        while not action.wait_for_server(timeout_sec=.1) and time.monotonic()<deadline:node.wait(.1)
        node.wait(3.)
        request=SetBool.Request();request.data=True
        assert node.arm.wait_for_service(timeout_sec=5.)
        response=node.await_future(node.arm.call_async(request));assert response.success,response.message
        goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=node.get_clock().now().to_msg()
        goal.pose.pose.position.x=target[0];goal.pose.pose.position.y=target[1]
        goal.pose.pose.orientation.z=math.sin(goal_yaw/2);goal.pose.pose.orientation.w=math.cos(goal_yaw/2)
        report['start']=[float(v) for v in node.position];report['goal']=target
        handle=node.await_future(action.send_goal_async(goal));assert handle.accepted
        result=handle.get_result_async();start=time.monotonic();inserted=None;removed=None
        while not result.done() and time.monotonic()-start<180:
            node.wait(.1);elapsed=time.monotonic()-start
            if inserted is None and np.linalg.norm(node.position-np.array(report['start']))>.12:
                node.obstacle=.60;node.phase='obstacle_stop';inserted=time.monotonic()
            if inserted is not None and removed is None and time.monotonic()-inserted>4:
                stopped=[p for p in node.trace if p['phase']=='obstacle_stop' and p['t']>inserted+1]
                report['checks']['obstacle_holds_zero']=bool(stopped) and all(abs(p['v'])<1e-6 and abs(p['w'])<1e-6 for p in stopped)
                assert report['checks']['obstacle_holds_zero']
                node.obstacle=None;node.phase='resume';removed=time.monotonic()
        assert result.done(),'Navigation action timed out'
        response=result.result();report['action_status']=response.status
        report['final']=[float(v) for v in node.position]
        report['position_error_m']=float(np.linalg.norm(node.position-np.array(report['goal'])))
        report['duration_s']=time.monotonic()-start
        report['checks']['arrived']=response.status==4 and report['position_error_m']<=.22
        report['checks']['moved_from_start']=bool(np.linalg.norm(node.position-np.array(report['start']))>.4)
        report['checks']['resumed_after_removal']=removed is not None and any(p['phase']=='resume' and p['v']>.005 for p in node.trace)
        limits=settings()['robot']
        report['checks']['speed_limits']=all(-1e-7<=p['v']<=limits['max_linear_mps']+1e-6 and abs(p['w'])<=limits['max_angular_rps']+1e-6 for p in node.trace)
        assert all(report['checks'].values()),report
        node.wait(.7)
        report['checks']['zero_after_arrival']=abs(node.velocity[0])<1e-6 and abs(node.velocity[1])<1e-6
        report['passed']=all(report['checks'].values())
        if args.soak_s:
            plans=0;deadline=time.monotonic()+args.soak_s
            while time.monotonic()<deadline:
                node.wait(min(5.,max(0.,deadline-time.monotonic())));node.path();plans+=1
            report['soak']={'duration_s':args.soak_s,'successful_plans':plans}
        report['virtual_can_frames']=node.can_frames
        if args.vcan:
            from common import driver_fingerprint,software_fingerprint
            report.update(system_feedback_period_s=.2,driver_sha256=driver_fingerprint(),software_sha256=software_fingerprint())
    except Exception as error:
        report['passed']=False;report['error']=repr(error)
    finally:
        if handle is not None:
            try:node.await_future(handle.cancel_goal_async(),3)
            except Exception:pass
        try:
            request=SetBool.Request();request.data=False;node.await_future(node.arm.call_async(request),3)
            client=node.create_client(ManageLifecycleNodes,'/navigation_lifecycle/manage_nodes')
            if client.wait_for_service(timeout_sec=1):
                request=ManageLifecycleNodes.Request();request.command=4
                report['orderly_shutdown']=node.await_future(client.call_async(request),10).success
        except Exception as error:report['shutdown_error']=repr(error)
        if driver is not None:
            driver.send_signal(signal.SIGINT);driver.wait(timeout=3);report['virtual_driver_exit_code']=driver.returncode;driver_log.close()
        (ROOT/f'closed_loop{suffix}_report.json').write_text(json.dumps(report,indent=2))
        (ROOT/f'closed_loop{suffix}_trace.json').write_text(json.dumps(node.trace))
        print(json.dumps(report,indent=2),flush=True)
        node.destroy_node();rclpy.shutdown();os.killpg(launch.pid,signal.SIGINT)
        try:launch.wait(timeout=15)
        except subprocess.TimeoutExpired:os.killpg(launch.pid,signal.SIGTERM);launch.wait(timeout=5)
        log.close()
        if node.bus is not None:node.bus.close()
    raise SystemExit(0 if report.get('passed') else 1)


if __name__=='__main__':main()
