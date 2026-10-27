"""Check mission CLI planning and refusal to actuate without hardware/site acceptance."""
import json
import os
import signal
import subprocess
import time
import rclpy
from nav2_msgs.srv import ManageLifecycleNodes
from common import ROOT
from integration_test import Probe

os.environ['ROS_DOMAIN_ID']='43';rclpy.init();node=Probe();node.command_on=False
log=(ROOT/'mission_offline_launch.log').open('w')
launch=subprocess.Popen(['ros2','launch',str(ROOT/'navigation.launch.py'),'bridge:=false'],
    env=dict(os.environ,ROS_DOMAIN_ID='43'),stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
report={'simulation_only':True,'hardware_driver_started':False,'checks':{}}
def run(args):
    child=subprocess.Popen([str(ROOT/'.ros_venv/bin/python'),str(ROOT/'mission.py'),*args],
        stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,env=dict(os.environ,ROS_DOMAIN_ID='43'))
    end=time.monotonic()+35
    while child.poll() is None and time.monotonic()<end:node.wait(.05)
    if child.poll() is None:child.kill();raise TimeoutError('Mission CLI hung')
    return child.returncode,child.communicate()[0]
try:
    deadline=time.monotonic()+35
    while not node.action.wait_for_server(timeout_sec=.1) and time.monotonic()<deadline:node.wait(.1)
    node.wait(3.)
    code,output=run([]);report['plan_stdout']=output
    report['checks']['default_plan_only_succeeds']=code==0
    plan=json.loads((ROOT/'results/latest_mission.json').read_text());report['plan_result']=plan
    assert code==0 and plan.get('planned_only'),output
    code,output=run(['--execute']);report['execute_refusal_stdout']=output
    report['checks']['execute_refused_without_hardware']=code!=0
    assert code!=0,output
    report['checks']['no_hardware_commands_published']=node.count_publishers('/tracer/cmd_vel')==0
    assert report['checks']['no_hardware_commands_published']
    report['passed']=True
except Exception as error:report['passed']=False;report['error']=repr(error)
finally:
    client=node.create_client(ManageLifecycleNodes,'/navigation_lifecycle/manage_nodes')
    try:
        if client.wait_for_service(timeout_sec=1):
            request=ManageLifecycleNodes.Request();request.command=4;node.await_future(client.call_async(request),10)
    except Exception as error:report['shutdown_error']=str(error)
    (ROOT/'mission_offline_report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
    node.destroy_node();rclpy.shutdown();os.killpg(launch.pid,signal.SIGINT)
    try:launch.wait(timeout=12)
    except subprocess.TimeoutExpired:os.killpg(launch.pid,signal.SIGTERM);launch.wait(timeout=5)
    log.close()
raise SystemExit(0 if report['passed'] else 1)
