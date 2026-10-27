"""Single ROS2 process tree. Unexpected component exit shuts down the system."""
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,IncludeLaunchDescription,RegisterEventHandler,EmitEvent
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    root=Path(LaunchConfiguration('project_root').perform(context)).resolve()
    if not (root/'generated/nav2.yaml').is_file():raise RuntimeError('Run start_ros2.sh to generate configuration first')
    env={'TRACER_PROJECT_ROOT':str(root)}
    backend=LaunchConfiguration('start_backend').perform(context).lower()=='true'
    navigation=LaunchConfiguration('navigation').perform(context).lower()=='true'
    drive=LaunchConfiguration('enable_drive').perform(context).lower()=='true'
    if drive and not navigation:raise RuntimeError('enable_drive requires navigation')
    nodes=[Node(package='tracer_bringup',executable='mapping_backend',parameters=[{'start_backend':backend}],additional_env=env,output='screen'),
           Node(package='tracer_bringup',executable='lio_bridge',additional_env=env,output='screen'),
           Node(package='tracer_bringup',executable='model_adapter',additional_env=env,output='screen'),
           Node(package='tracer_bringup',executable='model_recorder',parameters=[{'output_dir':LaunchConfiguration('output_dir').perform(context),'save_interval_s':10.}],additional_env=env,output='screen')]
    # Applies to backend, adapters, recorder and every Nav2/driver process in the included launch.
    actions=[RegisterEventHandler(OnProcessExit(on_exit=[EmitEvent(event=Shutdown(reason='A system component exited; fail closed'))]))]
    actions+=nodes
    if navigation:
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(root/'navigation.launch.py')),
                      launch_arguments={'enable_drive':str(drive).lower(),'bridge':'false'}.items()))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('project_root',default_value=os.environ.get('TRACER_PROJECT_ROOT','')),
        DeclareLaunchArgument('start_backend',default_value='true'),
        DeclareLaunchArgument('navigation',default_value='true'),
        DeclareLaunchArgument('enable_drive',default_value='false'),
        DeclareLaunchArgument('output_dir',default_value=''),OpaqueFunction(function=start)])
