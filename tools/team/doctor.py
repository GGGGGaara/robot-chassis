"""Read-only environment audit. Never opens sensors/CAN or launches a robot."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import sys

root=Path(__file__).resolve().parents[2]
p=argparse.ArgumentParser();p.add_argument('--profile',choices=('demo','integration'),default='demo');args=p.parse_args()
checks={}
def check(name,condition,detail):checks[name]={'ok':bool(condition),'detail':str(detail)}
check('ros_humble',os.environ.get('ROS_DISTRO')=='humble',os.environ.get('ROS_DISTRO'))
check('python310',sys.version_info[:2]==(3,10),sys.version.split()[0])
check('ros_python',importlib.util.find_spec('rclpy') is not None,sys.executable)
check('project_venv',(root/'tracer_navigation/.ros_venv/bin/python').is_file(),'Create with --system-site-packages')
from ament_index_python.packages import get_package_share_directory
for package in ('tracer_bringup','rviz2','rviz_default_plugins','nav2_bringup','nav2_rviz_plugins','tf2_tools','rqt_graph'):
    try:location=get_package_share_directory(package);check(package,True,location)
    except LookupError:check(package,False,'Install tooling / build tracer_bringup')
check('rviz_config',(root/'tracer_navigation/rviz/team.rviz').is_file(),'map + TF + cloud + plan + odometry')
if args.profile=='integration':
    sys.path.insert(0,str(root/'tracer_navigation'))
    from common import settings
    config=settings();nav=config['navigation']
    for key in ('static_map_lcm','lcm_types_dir'):check(key,Path(nav[key]).exists(),nav[key])
    dimos=Path(os.environ.get('TRACER_DIMOS_ROOT',str(root/'ku/dimos-main')))
    for relative in ('pyproject.toml','dimos/hardware/sensors/lidar/fastlio2/cpp/result/bin/fastlio2_native','target/release/voxel_ray_tracing'):
        check('dimos/'+relative,(dimos/relative).is_file(),dimos/relative)
    venv=Path(os.environ.get('TRACER_MAPPING_VENV',str(Path.home()/'venvs/dimos-mapping')))
    check('mapping_python312',(venv/'bin/python').is_file(),venv/'bin/python (check version separately)')
    for package in ('tracer_msgs','tracer_navigation_driver','nav2_collision_monitor'):
        try:check(package,True,get_package_share_directory(package))
        except LookupError:check(package,False,'Full vendor/driver build required')
report={'profile':args.profile,'offline_only':True,'passed':all(c['ok'] for c in checks.values()),'checks':checks}
print(json.dumps(report,indent=2,ensure_ascii=False))
raise SystemExit(0 if report['passed'] else 1)
