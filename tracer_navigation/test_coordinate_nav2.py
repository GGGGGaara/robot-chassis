"""Real Nav2 plan service + relative CLI, isolated domain 43, no CAN driver."""
import json
import os
import signal
import subprocess
import time

import rclpy
from nav2_msgs.srv import ManageLifecycleNodes
from common import ROOT
from integration_test import Probe


def main():
    os.environ['ROS_DOMAIN_ID']='43';rclpy.init();node=Probe();node.command_on=False
    folder=ROOT/'results/coordinate_mapping_20261023';folder.mkdir(parents=True,exist_ok=True)
    report={'simulation_only':True,'hardware_driver_started':False,'checks':{}}
    log=(folder/'coordinate_nav2_launch.log').open('w')
    launch=subprocess.Popen(['ros2','launch',str(ROOT/'navigation.launch.py'),'bridge:=false'],
        env=dict(os.environ,ROS_DOMAIN_ID='43'),stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    def run(script,args):
        child=subprocess.Popen([str(ROOT/'.ros_venv/bin/python'),str(ROOT/script),*args],
            stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,env=dict(os.environ,ROS_DOMAIN_ID='43'))
        end=time.monotonic()+35
        while child.poll() is None and time.monotonic()<end:node.wait(.05)
        if child.poll() is None:child.kill();child.wait();raise TimeoutError('CLI did not finish')
        return child.returncode,child.communicate()[0]
    try:
        deadline=time.monotonic()+40
        while not node.action.wait_for_server(timeout_sec=.1) and time.monotonic()<deadline:node.wait(.1)
        node.wait(3.)
        target=folder/'integration_target.json'
        code,out=run('mission.py',['--relative','.8','0','--save-target',str(target)])
        report['relative_plan_stdout']=out
        assert code==0,out
        saved=json.loads(target.read_text());mission=json.loads((ROOT/'results/latest_mission.json').read_text())
        report['checks']['forward_080_goal_correct']=abs(saved['goal'][0]-.9)<.01 and abs(saved['goal'][1]-2.08)<.01
        report['checks']['straight_distance_080']=abs(mission['straight_distance_m']-.8)<.01
        code,out=run('mission.py',['--target-file',str(target)])
        report['checks']['frozen_goal_replans']=code==0
        code,out=run('coordinate_task.py',['.8','0'])
        report['coordinate_cli_stdout']=out;report['checks']['operator_cli_plan_only']=code==0
        code,out=run('coordinate_task.py',['.8','0','--execute'])
        report['checks']['execute_refused_without_acceptance']=code!=0 and 'Site acceptance' in out
        report['checks']['no_physical_driver_or_cmd_publisher']=node.count_publishers('/tracer/ready')==0 and node.count_publishers('/tracer/cmd_vel')==0
        report['passed']=all(report['checks'].values());assert report['passed'],report
    except Exception as error:report['passed']=False;report['error']=repr(error)
    finally:
        try:
            client=node.create_client(ManageLifecycleNodes,'/navigation_lifecycle/manage_nodes')
            if client.wait_for_service(timeout_sec=1):
                request=ManageLifecycleNodes.Request();request.command=4;node.await_future(client.call_async(request),10)
        except Exception as error:report['shutdown_error']=str(error)
        node.destroy_node();rclpy.shutdown();os.killpg(launch.pid,signal.SIGINT)
        try:launch.wait(timeout=12)
        except subprocess.TimeoutExpired:os.killpg(launch.pid,signal.SIGTERM);launch.wait(timeout=5)
        log.close();(folder/'nav2_coordinate_tests.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
    return 0 if report.get('passed') else 1


if __name__=='__main__':raise SystemExit(main())
