"""Geometry and independent command gate shared by nodes and tests."""
import importlib.util
import math
from pathlib import Path
import sys
import hashlib
import json
import copy
import os

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

ROOT = Path(__file__).resolve().parent


def settings():
    config=yaml.safe_load((ROOT/'robot.yaml').read_text(encoding='utf-8'))
    nav=config['navigation']
    source=Path(os.environ.get('TRACER_STATIC_MAP_LCM',nav['static_map_lcm'])).expanduser()
    nav['static_map_lcm']=str(source if source.is_absolute() else ROOT.parent/source)
    override=os.environ.get('TRACER_LCM_TYPES_DIR')
    if override:
        nav['lcm_types_dir']=str(Path(override).expanduser())
    elif nav.get('lcm_types_dir')=='auto':
        spec=importlib.util.find_spec('dimos_lcm')
        if spec and spec.submodule_search_locations:
            nav['lcm_types_dir']=next(iter(spec.submodule_search_locations))
        else:
            venv=Path(os.environ.get('TRACER_MAPPING_VENV',str(Path.home()/'venvs/dimos-mapping'))).expanduser()
            candidates=sorted(venv.glob('lib/python*/site-packages/dimos_lcm'))
            nav['lcm_types_dir']=str(candidates[0]) if candidates else str(venv/'MISSING_dimos_lcm')
    return config


def config_fingerprint(config):
    value=copy.deepcopy(config)
    value['mount'].pop('verified',None);value['mount'].pop('source',None)
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':')).encode()).hexdigest()


def localization_map_path(config):
    default=Path(config['navigation']['static_map_lcm']).with_name('global_map.pc2.lcm')
    return Path(os.environ.get('DIMOS_PREMAP',os.environ.get('TRACER_PREMAP',str(default)))).expanduser()


def driver_fingerprint():
    runtime=Path(os.environ.get('TRACER_RUNTIME_ROOT',str(Path.home()/'tracer_nav_runtime'))).expanduser()
    path=runtime/'ws/install/tracer_navigation_driver/lib/tracer_navigation_driver/tracer_safe_base'
    return hashlib.sha256(path.read_bytes()).hexdigest()


def software_fingerprint():
    names=['common.py','dimos_ros_bridge.py','velocity_gate.py','generate_config.py','navigation.launch.py',
           'navigate.xml','navigate_through.xml','ground_test_pulse.py','mission.py','coordinate_geometry.py','coordinate_task.py',
           'ros_mapping_backend.py','ros_mapping_bridge.py','ros_model_recorder.py',
           'ws/src/tracer_bringup/launch/system.launch.py','ws/src/tracer_bringup/tracer_bringup/entry.py','start_ros2.sh',
           'ws/src/tracer_navigation_driver/src/tracer_safe_base.cpp','generated/nav2.yaml',
           'ros_env.sh','project_env.sh','live_grid.py','live_map_node.py','floor_task.py',
           'fastdds_loopback.xml','prepare_ros_loopback.sh','ground_test.sh','start_hardware.sh','commission.py']
    digest=hashlib.sha256()
    for name in names:digest.update(name.encode());digest.update((ROOT/name).read_bytes())
    return digest.hexdigest()


def acceptance_error(config):
    import time
    try:
        evidence=json.loads((ROOT/'site_acceptance.json').read_text())
        if not config['mount']['verified']:return 'mount_not_accepted'
        if evidence['config_sha256']!=config_fingerprint(config):return 'configuration_changed_after_acceptance'
        if evidence['driver_sha256']!=driver_fingerprint():return 'driver_changed_after_acceptance'
        if evidence.get('software_sha256')!=software_fingerprint():return 'software_changed_after_acceptance'
        if evidence.get('map_sha256')!=hashlib.sha256(Path(config['navigation']['static_map_lcm']).read_bytes()).hexdigest():return 'map_changed_after_acceptance'
        if evidence.get('localization_map_sha256')!=hashlib.sha256(localization_map_path(config).read_bytes()).hexdigest():return 'localization_map_changed_after_acceptance'
        age=time.time()-evidence['timestamp']
        if not 0<=age<=18*3600:return 'site_acceptance_expired'
        if not evidence.get('map_aligned') or not evidence.get('motion_observed'):return 'site_checks_missing'
    except (OSError,KeyError,ValueError):return 'site_acceptance_missing'
    return None


def pulse_profile(direction):
    return {'forward':(.03,0.),'left':(0.,.10),'right':(0.,-.10)}[direction]


def hardware_fault_requires_rearm(reason,armed,drive,now,grace_deadline):
    return bool(drive and armed and reason not in ('ok','stopped','near_obstacle','live_unknown','live_obstacle') and
                (reason!='command_timeout' or now>grace_deadline))


def require_virtual_can(name='vcan_navtest'):
    import subprocess
    info=json.loads(subprocess.check_output(['ip','-j','-details','link','show',name],text=True))
    if not info or info[0].get('linkinfo',{}).get('info_kind')!='vcan':raise RuntimeError('Offline tests require a genuine vcan interface')


def lcm_types(config):
    # Load ONLY pure-Python generated messages, never Python 3.12 binary packages.
    if 'dimos_lcm' not in sys.modules:
        path = Path(config['navigation']['lcm_types_dir'])
        spec = importlib.util.spec_from_file_location('dimos_lcm',path/'__init__.py',submodule_search_locations=[str(path)])
        module = importlib.util.module_from_spec(spec)
        sys.modules['dimos_lcm'] = module
        spec.loader.exec_module(module)
    from dimos_lcm.tf2_msgs.TFMessage import TFMessage
    from dimos_lcm.sensor_msgs.PointCloud2 import PointCloud2
    from dimos_lcm.nav_msgs.OccupancyGrid import OccupancyGrid
    return TFMessage, PointCloud2, OccupancyGrid


def mount_matrix(config):
    mount = config['mount']
    result = np.eye(4)
    result[:3,:3] = Rotation.from_euler('xyz',[mount[k] for k in ('roll_deg','pitch_deg','yaw_deg')],degrees=True).as_matrix()
    result[:3,3] = [mount[k] for k in ('x_m','y_m','z_m')]
    return result


def transform_matrix(t):
    m = np.eye(4)
    m[:3,:3] = Rotation.from_quat([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]).as_matrix()
    m[:3,3] = [t.translation.x,t.translation.y,t.translation.z]
    return m


def planar(m):
    yaw = math.atan2(m[1,0],m[0,0])
    result = np.eye(4)
    result[:3,:3] = Rotation.from_euler('z',yaw).as_matrix()
    result[:2,3] = m[:2,3]
    return result


def ground_correction(a,b,c):
    normal=np.array([-a,-b,1.]);normal/=np.linalg.norm(normal)
    axis=np.cross(normal,[0.,0.,1.])
    angle=math.atan2(np.linalg.norm(axis),normal[2])
    rotation=Rotation.from_rotvec(axis/np.linalg.norm(axis)*angle) if np.linalg.norm(axis)>1e-12 else Rotation.identity()
    result=np.eye(4);result[:3,:3]=rotation.as_matrix();result[2,3]=-c/np.linalg.norm([-a,-b,1.])
    return result


def cloud_xyz(cloud):
    fields = {f.name:f for f in cloud.fields}
    if any(k not in fields or fields[k].datatype != 7 for k in ('x','y','z')):
        raise ValueError('Expected float32 XYZ cloud')
    dtype = np.dtype({'names':['x','y','z'], 'formats':[('>' if cloud.is_bigendian else '<')+'f4']*3,
                      'offsets':[fields[k].offset for k in ('x','y','z')], 'itemsize':cloud.point_step})
    raw = bytes(cloud.data)
    rows = []
    for row in range(cloud.height):
        values = np.frombuffer(raw,dtype=dtype,count=cloud.width,offset=row*cloud.row_step)
        rows.append(np.column_stack([values[k] for k in ('x','y','z')]))
    xyz = np.concatenate(rows) if rows else np.empty((0,3),dtype=np.float32)
    return xyz[np.isfinite(xyz).all(axis=1)]


def filter_cloud(xyz, config):
    mount = mount_matrix(config)
    base = xyz @ mount[:3,:3].T + mount[:3,3]
    robot = config['robot']
    self_mask = ((np.abs(base[:,0]) < robot['length_m']/2+.03) &
                 (np.abs(base[:,1]) < robot['width_m']/2+.03))
    maximum = robot['height_m'] or config['navigation']['provisional_obstacle_height_m']
    keep = ~self_mask & (base[:,2]>=-.10) & (base[:,2]<=maximum) & (np.linalg.norm(base[:,:2],axis=1)<=8)
    return np.asarray(base[keep],dtype=np.float32)


def stop_polygon(config):
    """Eight-vertex near-stop area; radius is measured from chassis centre."""
    radius=float(config['navigation'].get('near_stop_radius_m',.75))
    if not math.isfinite(radius) or radius<=0:
        raise ValueError('Near-stop radius must be finite and positive')
    angle=np.arange(8)*math.pi/4
    polygon=radius*np.column_stack([np.cos(angle),np.sin(angle)])
    robot=config['robot']
    corners=np.array([[x,y] for x in (-robot['length_m']/2-.10,robot['length_m']/2+.10)
                      for y in (-robot['width_m']/2-.10,robot['width_m']/2+.10)])
    for a,b in zip(polygon,np.roll(polygon,-1,axis=0)):
        edge=b-a;relative=corners-a
        if np.any(edge[0]*relative[:,1]-edge[1]*relative[:,0]<-1e-9):
            raise ValueError('Near-stop area must contain the padded chassis footprint')
    return polygon


def near_stop_count(xyz,config):
    """Independent check of the same convex stop polygon as CollisionMonitor."""
    height=config['robot']['height_m'] or config['navigation']['provisional_obstacle_height_m']
    q=xyz[(xyz[:,2]>=.03)&(xyz[:,2]<=height),:2]
    polygon=stop_polygon(config)
    inside=np.ones(len(q),dtype=bool)
    for a,b in zip(polygon,np.roll(polygon,-1,axis=0)):
        edge=b-a;relative=q-a;inside&=(edge[0]*relative[:,1]-edge[1]*relative[:,0]>=-1e-9)
    return int(inside.sum())


class CommandGate:
    def __init__(self,config):
        self.config = config
        self.armed = False
        self.estop = False
        self.last = (0.,0.)

    def evaluate(self, command, ages, healthy=True, drive=False, dt=.02):
        nav, robot = self.config['navigation'],self.config['robot']
        reason = None
        if self.estop: reason='estop'
        elif not self.armed: reason='disarmed'
        elif drive and (not self.config['mount']['verified'] or robot['height_m'] is None): reason='calibration_unverified'
        elif not healthy: reason='unhealthy_inputs'
        elif ages['command']>nav['command_timeout_s']: reason='command_timeout'
        elif ages['cloud']>nav['perception_timeout_s']: reason='cloud_timeout'
        elif ages['pose']>nav['localization_timeout_s']: reason='localization_timeout'
        elif drive and ages['feedback']>nav['feedback_timeout_s']: reason='feedback_timeout'
        elif not all(math.isfinite(v) for v in command): reason='invalid_command'
        if reason:
            self.last=(0.,0.)
            return 0.,0.,reason
        v = max(0.,min(robot['max_linear_mps'],command[0]))
        w = max(-robot['max_angular_rps'],min(robot['max_angular_rps'],command[1]))
        # Collision stop must not be delayed by acceleration smoothing.
        if v==0 and w==0:
            self.last=(0.,0.)
            return 0.,0.,'stopped'
        dv, dw = robot['max_accel_mps2']*dt,robot['max_angular_accel_rps2']*dt
        v = min(v,self.last[0]+dv) if v>self.last[0] else v
        w = max(self.last[1]-dw,min(self.last[1]+dw,w))
        self.last=(v,w)
        return v,w,'ok'
