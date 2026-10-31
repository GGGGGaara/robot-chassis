"""Plan by default; explicit --execute runs a bounded, monitored Nav2 mission."""
import argparse
import hashlib
import json
import math
import time
import fcntl
import os
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose,NavigateToPose
from std_msgs.msg import Bool,String
from std_srvs.srv import SetBool
from tracer_msgs.msg import TracerStatus
from tf2_ros import Buffer,TransformListener
from common import ROOT,settings,acceptance_error
from navctl import show_in_rerun
from coordinate_geometry import relative_goal, finite_vector, distance_metrics, validate_anchor, motion_metrics,validate_route_length


class Mission(Node):
    def __init__(self):
        super().__init__('tracer_ab_mission');self.safety=None;self.flags={};self.wheel=None;self.distance=None;self.driver_diagnostic=None
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.create_subscription(String,'/navigation/safety_status',lambda m:setattr(self,'safety',(time.monotonic(),json.loads(m.data))),10)
        for topic,key in (('/navigation/localization_valid','pose'),('/navigation/perception_valid','cloud'),('/tracer/ready','driver')):
            self.create_subscription(Bool,topic,lambda m,k=key:self.flags.update({k:(time.monotonic(),m.data)}),10)
        self.create_subscription(TracerStatus,'/tracer_status',lambda m:setattr(self,'wheel',(time.monotonic(),m)),10)
        self.create_subscription(String,'/tracer/driver_diagnostics',lambda m:setattr(self,'driver_diagnostic',(time.monotonic(),json.loads(m.data))),10)
        self.arm=self.create_client(SetBool,'/navigation/arm')

    def wait(self,future,seconds=15):
        deadline=time.monotonic()+seconds
        while not future.done() and time.monotonic()<deadline:rclpy.spin_once(self,timeout_sec=.03)
        if not future.done():raise TimeoutError('ROS response timed out')
        result=future.result()
        if result is None:raise RuntimeError('ROS response missing')
        return result

    def pause(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.02)

    def pose(self):
        t=self.tf.lookup_transform('map','base_link',rclpy.time.Time())
        q=t.transform.rotation
        return [t.transform.translation.x,t.transform.translation.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]

    def set_arm(self,value):
        if not self.arm.wait_for_service(timeout_sec=2):raise RuntimeError('Navigation gate unavailable')
        req=SetBool.Request();req.data=value;response=self.wait(self.arm.call_async(req),3)
        if not response.success:raise RuntimeError(response.message)

    def healthy(self,execute=False):
        now=time.monotonic()
        if not self.safety or now-self.safety[0]>.5:raise RuntimeError('Safety status stale')
        for key in ('pose','cloud','driver') if execute else ('pose','cloud'):
            stamp,valid=self.flags.get(key,(0.,False))
            if not valid or now-stamp>.5:
                detail=''
                if key=='driver' and self.driver_diagnostic and now-self.driver_diagnostic[0]<2:
                    d=self.driver_diagnostic[1];detail=f': {d.get("reason")}, errno={d.get("send_errno")}, system_age={d.get("system_age_s")}, wheel_age={d.get("motion_age_s")}'
                raise RuntimeError(key+' not ready'+detail)
        if execute:
            if self.safety[1].get('mode')!='hardware':raise RuntimeError('Hardware mission requires hardware mode; preview does not move the robot')
            if self.count_publishers('/tracer/cmd_vel')!=1 or self.count_publishers('/tracer/ready')!=1:
                raise RuntimeError('Unexpected number of hardware command publishers/drivers')


def goal_from_selection():
    selected=json.loads((ROOT.parent/'realtime_mapping_usage/path_preview_results/latest_path.json').read_text())
    if selected.get('valid') is False or selected.get('live_start') is False:
        raise ValueError('Select a valid live-start target; offline demo targets cannot drive hardware')
    source=json.loads((ROOT/'generated/map_source.json').read_text())
    actual=hashlib.sha256(Path(source['source']).read_bytes()).hexdigest()
    if selected.get('map_sha256')!=source['sha256'] or actual!=source['sha256']:raise ValueError('Selected B point/map checksum mismatch')
    goal=selected['goal']
    if len(goal)!=2 or not all(math.isfinite(v) for v in goal):raise ValueError('Invalid B point')
    return goal,actual


def current_map_sha():
    source=json.loads((ROOT/'generated/map_source.json').read_text())
    actual=hashlib.sha256(Path(source['source']).read_bytes()).hexdigest()
    if actual!=source['sha256']:raise ValueError('Generated map/source checksum mismatch')
    return actual


def resolve_target(args,start):
    if args.relative is not None:
        return relative_goal(start,args.relative),current_map_sha(),{'kind':'relative','offset_m':args.relative,'anchor_pose':start}
    if args.xy is not None:
        return finite_vector(args.xy,2,'Map coordinates'),current_map_sha(),{'kind':'map_coordinates'}
    if args.target_file is not None:
        selected=json.loads(args.target_file.read_text(encoding='utf-8'))
        actual=current_map_sha()
        if selected.get('frame')!='map' or selected.get('units')!='m' or selected.get('map_sha256')!=actual:
            raise ValueError('Coordinate target frame/units/map checksum mismatch')
        if args.execute and selected.get('require_anchor'):
            validate_anchor(start,selected['anchor_pose'])
        return finite_vector(selected['goal'],2,'Coordinate target'),actual,selected
    target,actual=goal_from_selection()
    return target,actual,{'kind':'page_selection'}


def main():
    p=argparse.ArgumentParser(description='Selected B point in fixed map; default plan only, no arming.')
    p.add_argument('--execute',action='store_true');p.add_argument('--max-route-m',type=float)
    p.add_argument('--floor',action='store_true',help='Whole-floor tentative route; mandatory fresh live-clearance profile')
    target_args=p.add_mutually_exclusive_group()
    target_args.add_argument('--relative',nargs=2,type=float,metavar=('FORWARD_M','LEFT_M'),help='Offset from current chassis centre/heading; positive left')
    target_args.add_argument('--xy',nargs=2,type=float,metavar=('X_M','Y_M'),help='Absolute map coordinates in metres')
    target_args.add_argument('--target-file',type=Path,help='Frozen coordinate target with map checksum')
    p.add_argument('--timeout-s',type=float)
    p.add_argument('--save-target',type=Path,help='Save resolved target for a subsequent guarded execution')
    args=p.parse_args()
    config=settings();live=config.get('realtime_navigation',{})
    if args.floor and not live.get('enabled'):raise SystemExit('Floor mission requires realtime navigation profile')
    bound=float(live.get('max_floor_route_m',200.)) if args.floor else 15.
    duration=float(live.get('max_floor_duration_s',3600.)) if args.floor else 600.
    if not 0<bound<=1000 or not 5<=duration<=7200:raise SystemExit('Invalid configured floor bounds')
    if args.max_route_m is None:args.max_route_m=bound if args.floor else float(config['navigation'].get('max_route_m',3.))
    if args.timeout_s is None:args.timeout_s=duration if args.floor else 180.
    if not 0<args.max_route_m<=bound or not 5<=args.timeout_s<=duration:raise SystemExit('Invalid mission length/time bound')
    # Protect the shared navigator/gate from simultaneous task clients.
    mission_lock=(ROOT/'results'/f'.mission_{os.environ.get("ROS_DOMAIN_ID","42")}.lock').open('w')
    try:fcntl.flock(mission_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise SystemExit('Another mission is already running; cancel it first')
    rclpy.init();node=Mission();handle=None;report={'execute':args.execute,'timestamp':time.time(),'trace':[],
        'floor_mode':args.floor,'reference_route_is_tentative':bool(live.get('enabled')),'reachability':'unconfirmed'};exit_code=1
    folder=ROOT/'results';folder.mkdir(exist_ok=True)
    try:
        node.pause(2.);node.healthy(args.execute)
        if args.execute:
            error=acceptance_error(settings())
            if error:raise RuntimeError(error)
            if node.safety[1].get('armed'):raise RuntimeError('Disarm before starting a new mission')
        start=node.pose();target,mapsha,target_context=resolve_target(args,start)
        report.update(start=start,goal=target,map_sha256=mapsha,target_context=target_context)
        pose=PoseStamped();pose.header.frame_id='map';pose.header.stamp=node.get_clock().now().to_msg()
        pose.pose.position.x=target[0];pose.pose.position.y=target[1]
        yaw=math.atan2(target[1]-start[1],target[0]-start[0]);pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2)
        planner=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
        if not planner.wait_for_server(timeout_sec=5):raise RuntimeError('Planner unavailable')
        req=ComputePathToPose.Goal();req.goal=pose;req.planner_id='GridBased';req.use_start=False
        plan_handle=node.wait(planner.send_goal_async(req));response=node.wait(plan_handle.get_result_async(),20)
        if response.status!=4:raise RuntimeError('Planner did not succeed')
        points=[[v.pose.position.x,v.pose.position.y] for v in response.result.path.poses]
        if len(points)<2:raise RuntimeError('No route returned')
        report.update(distance_metrics(start,target,points));length=report['planned_length_m'];report['path']=points
        validate_route_length(length,args.max_route_m,bound)
        if math.dist(points[0],start[:2])>.35:raise RuntimeError('Path start does not match current robot position')
        # Face the end of the planned route instead of forcing global yaw=0.
        tail=next((v for v in reversed(points[:-1]) if math.dist(v,points[-1])>.10),points[0])
        yaw=math.atan2(target[1]-tail[1],target[0]-tail[0]);pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2)
        report['goal_yaw_rad']=yaw;show_in_rerun(points)
        print(f'Straight A-B: {report["straight_distance_m"]:.2f} m; planned route including detours: {length:.2f} m; B={target}; end heading {math.degrees(yaw):.1f} degrees.',flush=True)
        if report['detour_ratio'] and report['detour_ratio']>1.5:
            print('Long detour: check the route and obstacles. Route length is not straight-line distance or a scale calibration.',flush=True)
        if args.save_target:
            frozen={'frame':'map','units':'m','goal':target,'map_sha256':mapsha,'created_at':time.time(),
                    'anchor_pose':start,'require_anchor':args.relative is not None,'target_context':target_context}
            args.save_target.write_text(json.dumps(frozen,indent=2),encoding='utf-8')
        if not args.execute:
            report['planned_only']=True;report['passed']=True;exit_code=0;return
        nav=ActionClient(node,NavigateToPose,'/navigate_to_pose')
        if not nav.wait_for_server(timeout_sec=5):raise RuntimeError('Navigator unavailable')
        req=NavigateToPose.Goal();req.pose=pose
        handle=node.wait(nav.send_goal_async(req,feedback_callback=lambda m:setattr(node,'distance',m.feedback.distance_remaining)))
        if not handle.accepted:raise RuntimeError('Navigation goal rejected')
        result=handle.get_result_async();node.set_arm(True);begin=time.monotonic();last_progress=begin;last_xy=node.pose()[:2];last_record=0.;no_wheel_response=None
        print('Hardware mission armed. Use physical emergency stop if motion is unexpected.',flush=True)
        while not result.done():
            node.pause(.05);node.healthy(True);now=time.monotonic();state=node.safety[1]
            if now-begin>.5 and not state.get('armed'):raise RuntimeError('Gate disarmed: '+state.get('reason','unknown'))
            if now-begin>args.timeout_s:raise TimeoutError('Mission duration exceeded')
            current=node.pose()
            if math.dist(current[:2],last_xy)>.05:last_progress=now;last_xy=current[:2]
            if now-last_progress>45:raise RuntimeError('No position progress for 45 seconds')
            if now-begin>.5 and (not node.wheel or now-node.wheel[0]>.5):raise RuntimeError('Wheel feedback stale')
            requested=abs(state.get('linear',0))>.01 or abs(state.get('angular',0))>.03
            wheel_still=node.wheel and abs(node.wheel[1].linear_velocity)<.002 and abs(node.wheel[1].angular_velocity)<.01
            if requested and wheel_still:
                if no_wheel_response is None:no_wheel_response=now
                elif now-no_wheel_response>3:raise RuntimeError('Commanded motion but wheels did not respond for 3 seconds')
            else:no_wheel_response=None
            if now-last_record>.1:
                report['trace'].append({'elapsed_s':now-begin,'pose':current,'remaining_m':node.distance,'safety':state,
                    'wheel':[node.wheel[1].linear_velocity,node.wheel[1].angular_velocity] if node.wheel else None})
                last_record=now
        response=result.result();report['action_status']=response.status;report['final']=node.pose()
        report['position_error_m']=math.dist(report['final'][:2],target)
        report['duration_s']=time.monotonic()-begin
        if response.status!=4:raise RuntimeError('Navigation action did not succeed: '+str(response.status))
        report['passed']=report['position_error_m']<=.20
        if not report['passed']:raise RuntimeError('Final position outside acceptance tolerance')
        report['reachability']='arrived_observed'
        print(f'Arrived: position error {report["position_error_m"]:.3f} m. Disarming.',flush=True);exit_code=0
    except (Exception,KeyboardInterrupt) as error:
        report['passed']=False;report['error']=repr(error);print('Mission stopped: '+str(error),flush=True)
    finally:
        report['driver_diagnostic']=node.driver_diagnostic[1] if node.driver_diagnostic else None
        if args.execute:
            try:node.set_arm(False)
            except Exception as error:report['disarm_error']=str(error)
            if handle is not None:
                try:node.wait(handle.cancel_goal_async(),3)
                except Exception as error:report['cancel_error']=str(error)
            node.pause(.5)
            report['final_gate']=node.safety[1] if node.safety else None
            stopped=bool(node.wheel and time.monotonic()-node.wheel[0]<.5 and abs(node.wheel[1].linear_velocity)<.01 and abs(node.wheel[1].angular_velocity)<.02)
            report['final_wheel_stop_confirmed']=stopped
            if not stopped or report.get('disarm_error') or report.get('cancel_error'):
                report['passed']=False;exit_code=1
                report['reachability']='arrival_or_stop_not_confirmed'
                print('Physical stop not fully confirmed; check the vehicle and use physical estop if needed.',flush=True)
        if report['trace']:
            report.update(motion_metrics(report['trace']))
            if not report.get('passed'):
                last=report['trace'][-1]['pose'][:2]
                report['remaining_straight_m']=math.dist(last,report['goal'])
                print(f'Incomplete task: wheel-estimated travel {report["wheel_travelled_m"]:.2f} m; LIO travel {report["lio_travelled_m"]:.2f} m; straight distance still to B {report["remaining_straight_m"]:.2f} m. This is not an arrival.',flush=True)
        (folder/'latest_mission.json').write_text(json.dumps(report,indent=2))
        (folder/f'mission_{int(report["timestamp"])}.json').write_text(json.dumps(report,indent=2))
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__=='__main__':main()
