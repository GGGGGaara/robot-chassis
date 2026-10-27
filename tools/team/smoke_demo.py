"""Receive real ROS messages from the installed isolated demo launch. No hardware."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid,Odometry,Path as RosPath
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage

root=Path(__file__).resolve().parents[2]
if os.environ.get('ROS_DOMAIN_ID')!='74':raise SystemExit('Requires ROS_DOMAIN_ID=74')
rclpy.init();node=rclpy.create_node('team_demo_smoke');received={}
node.create_subscription(OccupancyGrid,'/map',lambda m:received.update(map=m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
node.create_subscription(PointCloud2,'/mapping/global_points',lambda m:received.update(cloud=m),10)
node.create_subscription(PointCloud2,'/points_navigation',lambda m:received.update(obstacles=m),qos_profile_sensor_data)
node.create_subscription(RosPath,'/plan',lambda m:received.update(path=m),10)
node.create_subscription(Odometry,'/lio/odom',lambda m:received.update(odom=m),10)
node.create_subscription(TFMessage,'/tf',lambda m:received.update(tf=m),10)
node.create_subscription(TFMessage,'/tf_static',lambda m:received.update(static_tf=m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
folder=root/'tracer_navigation/results/team_alignment_20261027';folder.mkdir(parents=True,exist_ok=True)
log=(folder/'demo_launch.log').open('w')
proc=subprocess.Popen(['bash',str(root/'tools/team/demo.sh'),'rviz:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
report={'offline_only':True,'hardware_started':False,'domain':74,'checks':{}}
try:
    end=time.monotonic()+20
    while time.monotonic()<end and len(received)<7:
        rclpy.spin_once(node,timeout_sec=.1)
        if proc.poll() is not None:raise RuntimeError('Demo launch exited: inspect demo_launch.log')
    c=report['checks']
    c['received_map_cloud_path_odom_tf']=len(received)==7
    c['metric_map']=bool('map' in received and received['map'].header.frame_id=='map' and abs(received['map'].info.resolution-.1)<1e-6 and len(received['map'].data)==3600)
    c['xyz_pointcloud']=bool('cloud' in received and received['cloud'].point_step==12 and received['cloud'].width==1200 and len(received['cloud'].data)==14400)
    c['synthetic_obstacle_sensor_qos']=bool('obstacles' in received and received['obstacles'].header.frame_id=='base_link' and received['obstacles'].width==16)
    c['route_in_map']=bool('path' in received and received['path'].header.frame_id=='map' and len(received['path'].poses)==25)
    c['odom_base_link']=bool('odom' in received and received['odom'].header.frame_id=='odom' and received['odom'].child_frame_id=='base_link')
    c['no_velocity_publishers']=all(node.count_publishers(t)==0 for t in ('/cmd_vel','/cmd_vel_nav','/tracer/cmd_vel','/tracer/ready'))
    c['no_navigation_actions']=not any(n in ('planner_server','controller_server','tracer_safe_base','mapping_backend') for n in node.get_node_names())
    report['passed']=all(c.values())
except Exception as e:
    report.update(passed=False,error=repr(e))
finally:
    if proc.poll() is None:
        # Signal launch once; launch itself forwards SIGINT to its child nodes.
        proc.send_signal(signal.SIGINT)
        try:proc.wait(timeout=8)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
    log.close();node.destroy_node();rclpy.shutdown()
    report['launch_exit_code']=proc.returncode
    report['checks']['clean_shutdown']=proc.returncode in (0,130) and 'Traceback (most recent call last)' not in (folder/'demo_launch.log').read_text()
    report['passed']=bool(report.get('passed') and all(report['checks'].values()))
    (folder/'demo_smoke.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
raise SystemExit(0 if report.get('passed') else 1)
