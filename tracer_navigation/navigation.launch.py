from pathlib import Path
import yaml
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from common import acceptance_error,settings
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT=Path(__file__).resolve().parent


def start(context):
    drive=LaunchConfiguration('enable_drive').perform(context).lower()=='true'
    bridge=LaunchConfiguration('bridge').perform(context).lower()=='true'
    c=settings()
    if drive and (not c['mount']['verified'] or c['robot']['height_m'] is None):
        raise RuntimeError('Hardware launch requires successful forward/left tests and commission.sh accept; do not bypass by editing YAML')
    if drive and acceptance_error(c):raise RuntimeError('Hardware acceptance: '+acceptance_error(c))
    params=str(ROOT/'generated/nav2.yaml')
    nodes=[]
    if bridge:
        nodes.append(ExecuteProcess(cmd=[str(ROOT/'.ros_venv/bin/python'),str(ROOT/'dimos_ros_bridge.py')],output='screen'))
        nodes.append(ExecuteProcess(cmd=[str(ROOT/'.ros_venv/bin/python'),str(ROOT/'live_scan_view.py')],output='screen'))
    nodes.append(ExecuteProcess(cmd=[str(ROOT/'.ros_venv/bin/python'),str(ROOT/'velocity_gate.py'),
                  '--ros-args','-p',f'enable_drive:={str(drive).lower()}'],output='screen'))
    live=c.get('realtime_navigation',{}).get('enabled',False)
    if live:
        nodes.append(ExecuteProcess(cmd=[str(ROOT/'.ros_venv/bin/python'),str(ROOT/'live_map_node.py')],output='screen'))
    if drive:
        nodes.append(Node(package='tracer_navigation_driver',executable='tracer_safe_base',name='tracer_safe_base',
                          parameters=[{'can_interface':c['robot']['can_interface'],'enable_drive':True,
                            'max_linear_mps':c['robot']['max_linear_mps'],
                            'can_tx_period_s':c['robot']['can_tx_period_s'],
                            'system_feedback_timeout_s':c['robot']['system_feedback_timeout_s']}],output='screen'))
    for package,executable,name,remaps in (
        ('nav2_map_server','map_server','map_server',[('map','/navigation/reference_map')] if live else []),
        ('nav2_controller','controller_server','controller_server',[('cmd_vel','/cmd_vel_nav')]),
        ('nav2_planner','planner_server','planner_server',[]),
        ('nav2_behaviors','behavior_server','behavior_server',[]),
        ('nav2_bt_navigator','bt_navigator','bt_navigator',[]),
        ('nav2_velocity_smoother','velocity_smoother','velocity_smoother',[('cmd_vel','/cmd_vel_nav'),('cmd_vel_smoothed','/cmd_vel_smoothed')]),
        ('nav2_collision_monitor','collision_monitor','collision_monitor',[])):
        nodes.append(Node(package=package,executable=executable,name=name,parameters=[params],remappings=remaps,output='screen'))
    nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='navigation_lifecycle',output='screen',
        parameters=[{'autostart':True,'node_names':['map_server','controller_server','planner_server','behavior_server','bt_navigator','velocity_smoother','collision_monitor'],'bond_timeout':4.0}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('enable_drive',default_value='false'),
                              DeclareLaunchArgument('bridge',default_value='true'),OpaqueFunction(function=start)])
