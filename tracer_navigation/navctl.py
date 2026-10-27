#!/usr/bin/env python3
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose,NavigateToPose
from std_srvs.srv import SetBool
from action_msgs.srv import CancelGoal
from common import ROOT,settings,lcm_types


def show_in_rerun(points):
    import lcm
    lcm_types(settings())
    from lcm_msgs import nav_msgs,geometry_msgs,std_msgs
    now=time.time()
    header=std_msgs.Header(stamp=std_msgs.Time(sec=int(now),nsec=int((now-int(now))*1e9)),frame_id='map')
    poses=[geometry_msgs.PoseStamped(header=header,pose=geometry_msgs.Pose(
        position=geometry_msgs.Point(x=x,y=y,z=0.),orientation=geometry_msgs.Quaternion(x=0.,y=0.,z=0.,w=1.))) for x,y in points]
    message=nav_msgs.Path(poses_length=len(poses),header=header,poses=poses)
    bus=lcm.LCM()
    for _ in range(3):
        bus.publish('/navigation_plan#nav_msgs.Path',message.lcm_encode());time.sleep(.1)


def wait(node,future,timeout=20):
    rclpy.spin_until_future_complete(node,future,timeout_sec=timeout)
    if not future.done():raise RuntimeError('ROS response timed out')
    return future.result()


def main():
    parser=argparse.ArgumentParser(description='Nav2 goals and stop controls; coordinates in map metres.')
    parser.add_argument('command',choices=['plan','navigate','plan-selected','navigate-selected','arm','disarm','estop','reset-estop','cancel'])
    parser.add_argument('coordinates',nargs='*',type=float)
    args=parser.parse_args();rclpy.init();node=Node('tracer_navigation_control')
    try:
        if args.command.endswith('-selected'):
            selected=json.loads((ROOT.parent/'realtime_mapping_usage/path_preview_results/latest_path.json').read_text())
            source=json.loads((ROOT/'generated/map_source.json').read_text())
            if selected.get('map_sha256')!=source['sha256']:raise ValueError('Selected target belongs to a different static map')
            args.coordinates=selected['goal'];args.command=args.command.removesuffix('-selected')
        if args.command in ('arm','disarm','estop','reset-estop'):
            service='/navigation/arm' if args.command in ('arm','disarm') else '/navigation/estop'
            client=node.create_client(SetBool,service)
            if not client.wait_for_service(timeout_sec=5):raise RuntimeError('Navigation gate not running')
            request=SetBool.Request();request.data=args.command in ('arm','estop')
            result=wait(node,client.call_async(request));print(result.message)
            if not result.success:raise RuntimeError('Command not accepted')
        elif args.command=='cancel':
            client=node.create_client(CancelGoal,'/navigate_to_pose/_action/cancel_goal')
            if not client.wait_for_service(timeout_sec=5):raise RuntimeError('Navigator unavailable')
            result=wait(node,client.call_async(CancelGoal.Request()));print('Cancellation response:',result.return_code)
        else:
            if len(args.coordinates) not in (2,3) or not all(math.isfinite(v) for v in args.coordinates):
                raise ValueError('Use plan/navigate X Y [YAW_DEGREES]')
            x,y=args.coordinates[:2];yaw=math.radians(args.coordinates[2] if len(args.coordinates)==3 else 0)
            kind=ComputePathToPose if args.command=='plan' else NavigateToPose
            action='/compute_path_to_pose' if args.command=='plan' else '/navigate_to_pose'
            client=ActionClient(node,kind,action)
            if not client.wait_for_server(timeout_sec=10):raise RuntimeError('Nav2 action unavailable')
            goal=kind.Goal();pose=goal.goal if args.command=='plan' else goal.pose
            pose.header.frame_id='map';pose.header.stamp=node.get_clock().now().to_msg()
            pose.pose.position.x=x;pose.pose.position.y=y;pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2)
            if args.command=='plan':goal.planner_id='GridBased';goal.use_start=False
            handle=wait(node,client.send_goal_async(goal))
            if not handle.accepted:raise RuntimeError('Goal rejected')
            if args.command=='navigate':
                print('Navigation goal accepted. Gate remains in its current arm/disarm mode.');return
            response=wait(node,handle.get_result_async())
            if response.status!=4 or not response.result.path.poses:raise RuntimeError('No valid route')
            points=[[p.pose.position.x,p.pose.position.y] for p in response.result.path.poses]
            result={'frame':'map','points':points,'length_m':sum(math.dist(a,b) for a,b in zip(points,points[1:])),'goal':[x,y],'timestamp':time.time()}
            output=ROOT/'results';output.mkdir(exist_ok=True)
            (output/'latest_nav2_path.json').write_text(json.dumps(result,indent=2))
            show_in_rerun(points)
            print(f'Nav2 route: {len(points)} points, {result["length_m"]:.2f} m; saved to {output}/latest_nav2_path.json')
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
