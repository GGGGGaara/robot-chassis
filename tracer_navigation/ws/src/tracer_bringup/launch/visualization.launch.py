"""Visualization launch; real data by default, isolated synthetic data on request."""
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    root=Path(LaunchConfiguration('project_root').perform(context)).resolve()
    demo=LaunchConfiguration('demo').perform(context).lower()=='true'
    rviz=LaunchConfiguration('rviz').perform(context).lower()=='true'
    actions=[]
    if demo:
        if os.environ.get('ROS_DOMAIN_ID')!='74' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
            raise RuntimeError('Synthetic demo is restricted to localhost domain 74')
        actions.append(Node(package='tracer_bringup',executable='visual_demo',
                            additional_env={'TRACER_PROJECT_ROOT':str(root)},output='screen'))
    if rviz:
        actions.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(root/'rviz/team.rviz')],output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('project_root',default_value=os.environ.get('TRACER_PROJECT_ROOT','')),
        DeclareLaunchArgument('demo',default_value='false'),DeclareLaunchArgument('rviz',default_value='true'),OpaqueFunction(function=start)])
