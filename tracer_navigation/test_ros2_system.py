"""Installed ROS2 bringup + synthetic LCM backend -> ROS clouds -> CDR/PLY. No hardware."""
import copy
import json
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path
import lcm
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from lifecycle_msgs.srv import GetState
from common import ROOT,settings,lcm_types,cloud_xyz
from test_coordinates_and_models import cloud


def main():
    os.environ['ROS_DOMAIN_ID']='43';rclpy.init();node=Node('ros2_system_offline_probe')
    TF,_,_=lcm_types(settings())
    from dimos_lcm.geometry_msgs.TransformStamped import TransformStamped
    bus=lcm.LCM();state={};checks={};folder=ROOT/'results/ros2_unification_20261023';folder.mkdir(exist_ok=True)
    output=folder/('model_test_'+str(time.time_ns()));output.mkdir()
    lifecycle_names=('map_server','controller_server','planner_server','behavior_server','bt_navigator','velocity_smoother','collision_monitor')
    states={};clients={name:node.create_client(GetState,'/'+name+'/get_state') for name in lifecycle_names};waiting=set()
    def query_states():
        for name,client in clients.items():
            if name not in waiting and client.service_is_ready():
                waiting.add(name)
                def done(future,key=name):
                    waiting.discard(key)
                    if future.result():states[key]=future.result().current_state.label
                client.call_async(GetState.Request()).add_done_callback(done)
    node.create_timer(1.,query_states)
    node.create_subscription(PointCloud2,'/mapping/global_points',lambda m:state.update(model=m),10)
    node.create_subscription(PointCloud2,'/lidar/points',lambda m:state.update(raw=m),qos_profile_sensor_data)
    node.create_subscription(Bool,'/navigation/localization_valid',lambda m:state.update(localized=m.data),10)
    log=(folder/'system_launch.log').open('w')
    process=subprocess.Popen(['ros2','launch','tracer_bringup','system.launch.py',f'project_root:={ROOT}',
                              'start_backend:=false','navigation:=true',f'output_dir:={output}'],
                              stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    def publish():
        stamp=time.time();tf=TF();tf.transforms=[]
        for child,x in (('map',5.),('mid360_link',6.)):
            item=copy.deepcopy(TransformStamped());item.header.frame_id='odom';item.child_frame_id=child
            item.header.stamp.sec=int(stamp);item.header.stamp.nsec=int((stamp%1)*1e9)
            item.transform.translation.x=x;item.transform.rotation.w=1.;tf.transforms.append(item)
        tf.transforms_length=2;bus.publish('/tf#tf2_msgs.TFMessage',tf.lcm_encode())
        msg=cloud([[6.,0.,.2],[7.,0.,.2]],stamp)
        bus.publish('/global_map#sensor_msgs.PointCloud2',msg.lcm_encode())
        msg=cloud(np.tile([1.,0.,0.],(300,1)),stamp);msg.header.frame_id='mid360_link'
        bus.publish('/lidar#sensor_msgs.PointCloud2',msg.lcm_encode())
    node.create_timer(.1,publish)
    report={'offline_only':True,'hardware_driver_started':False}
    try:
        deadline=time.monotonic()+40
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if (output/'latest.json').exists() and state.get('localized') and state.get('raw') is not None and all(states.get(n)=='active' for n in lifecycle_names):break
            if process.poll() is not None:raise RuntimeError('Unified launch exited: inspect system_launch.log')
        checks['ros_model_received']='model' in state
        checks['full_map_transform_correct']=bool('model' in state and np.allclose(cloud_xyz(state['model'])[:,0],[1.,2.]))
        checks['raw_sensor_cloud_on_ros']='raw' in state and state['raw'].header.frame_id=='mid360_link'
        checks['ros_localization_valid']=bool(state.get('localized'))
        pointer=json.loads((output/'latest.json').read_text());snapshot=output/pointer['snapshot']
        checks['ros_cdr_and_ply_saved']=(snapshot/'model_map.ply').exists() and (snapshot/'global_points.cdr').exists()
        from rclpy.serialization import deserialize_message
        saved=deserialize_message((snapshot/'global_points.cdr').read_bytes(),PointCloud2)
        checks['cdr_roundtrip_metres']=saved.header.frame_id=='map' and np.allclose(cloud_xyz(saved)[:,0],[1.,2.])
        checks['no_hardware_command_publishers']=node.count_publishers('/tracer/cmd_vel')==0
        checks['all_seven_nav2_nodes_active']=all(states.get(n)=='active' for n in lifecycle_names)
        assert all(checks.values()),checks
        # Exercise fail-closed OnProcessExit with a real child exit.
        backend=subprocess.check_output(['pgrep','-f',str(ROOT/'ros_mapping_backend.py')],text=True).split()
        assert len(backend)==1,backend
        os.kill(int(backend[0]),signal.SIGTERM)
        process.wait(timeout=25);checks['component_exit_stops_launch']=True
        report['passed']=all(checks.values())
    except Exception as error:report['passed']=False;report['error']=repr(error)
    finally:
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=25)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=10)
        node.destroy_node();rclpy.shutdown();log.close()
        report['checks']=checks;(folder/'integration.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
    return 0 if report.get('passed') else 1


if __name__=='__main__':raise SystemExit(main())
