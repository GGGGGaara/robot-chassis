"""Audit local readiness without connecting sensors or sending CAN frames."""
import hashlib
import json
import py_compile
import subprocess
import time
import xml.etree.ElementTree as ET
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_prefix
from common import ROOT,settings,config_fingerprint,software_fingerprint,driver_fingerprint,acceptance_error,localization_map_path

c=settings();checks={};errors=[]
def check(name,fn):
    try:checks[name]=fn()
    except Exception as error:checks[name]=False;errors.append(name+': '+str(error))
def syntax():
    for path in ROOT.glob('*.py'):py_compile.compile(str(path),doraise=True)
    for path in ROOT.glob('*.sh'):subprocess.run(['bash','-n',str(path)],check=True,capture_output=True)
    for path in ROOT.glob('*.xml'):ET.parse(path)
    return True
check('python_shell_xml_syntax',syntax)
def dependencies():
    return {name:get_package_prefix(name) for name in ('nav2_map_server','nav2_controller','nav2_planner','nav2_behaviors',
        'nav2_regulated_pure_pursuit_controller','nav2_collision_monitor','nav2_velocity_smoother','tracer_navigation_driver')}
check('installed_packages',dependencies)

def transport():
    import os
    assert os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE')==str(ROOT/'fastdds_loopback.xml')
    for port in ('17910','18160'):
        route=subprocess.check_output(['ip','-4','route','get','127.0.0.1','ipproto','udp','dport',port],text=True)
        assert ' dev lo ' in route,route
    return {'dds':'UDPv4 loopback only','domains':[42,43]}
check('ros_local_udp_route',transport)
def maps():
    source=Path(c['navigation']['static_map_lcm']);sha=hashlib.sha256(source.read_bytes()).hexdigest()
    metadata=json.loads((ROOT/'generated/map_source.json').read_text())
    assert sha==metadata['sha256']
    fixed=localization_map_path(c);cloud_sha=hashlib.sha256(fixed.read_bytes()).hexdigest()
    if source.resolve()==(ROOT.parent/'reconstruction_20261017_113920/raytrace/global_costmap.lcm').resolve():
        assert sha=='084292d8a9a6dc0cc8047c6c4ea902b8e533f57bb8c9962e6345e1f25e19399b'
        assert cloud_sha=='5583f9970567a74c69c51181273c0c119f12c4d508cc52c9bbd2d741ab339e05'
    else:
        manifest=json.loads((source.parent.parent/'floor_reference.json').read_text())
        assert manifest['frame']=='map' and manifest['units']=='m' and manifest['reference_only_not_live_clearance']
        assert manifest['sha256']['global_costmap.lcm']==sha and manifest['sha256']['global_map.pc2.lcm']==cloud_sha
    return {'reference_costmap_sha256':sha,'reference_pointcloud_sha256':cloud_sha,'assets_verified':True}
check('maps_unchanged',maps)
def limits():
    assert c['robot']['height_m']==1.2 and c['robot']['max_linear_mps']<=.10 and c['robot']['max_angular_rps']<=.15
    assert c['navigation']['max_route_m']==15.
    assert c['robot']['length_m']==.702 and c['robot']['width_m']==.610
    assert c['robot']['can_tx_period_s']==.05 and c['robot']['system_feedback_timeout_s']==.6
    nav=yaml.safe_load((ROOT/'generated/nav2.yaml').read_text())
    assert nav['controller_server']['ros__parameters']['FollowPath']['allow_reversing'] is False
    assert nav['behavior_server']['ros__parameters']['behavior_plugins']==['wait']
    if c.get('realtime_navigation',{}).get('enabled'):
        local=nav['local_costmap']['local_costmap']['ros__parameters'];glob=nav['global_costmap']['global_costmap']['ros__parameters']
        assert local['evidence_layer']['map_topic']=='/navigation/live_map' and 'static_layer' not in local['plugins']
        assert glob['evidence_layer']['map_topic']=='/map' and glob['trinary_costmap'] is False
        assert local['track_unknown_space'] and local['unknown_cost_value']==255
    else:assert nav['global_costmap']['global_costmap']['ros__parameters']['static_layer']['subscribe_to_updates'] is False
    return True
check('conservative_limits_and_fixed_map',limits)
check('compiled_driver_sha256',driver_fingerprint)
report={'timestamp':time.time(),'offline_only':True,'physical_can_present':Path('/sys/class/net/can0').exists(),
    'checks':checks,'errors':errors,'passed':not errors,'config_sha256':config_fingerprint(c),
    'software_sha256':software_fingerprint(),'hardware_acceptance':acceptance_error(c) or 'accepted'}
(ROOT/'offline_check_report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
raise SystemExit(0 if report['passed'] else 1)
