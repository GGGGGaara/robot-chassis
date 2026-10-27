"""Offline route/unit audit, or subscribe-only manually measured travel audit."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np
import yaml

from common import ROOT,settings,lcm_types,mount_matrix
from coordinate_geometry import distance_metrics,motion_metrics


def offline_audit():
    config=settings();_,_,Grid=lcm_types(config)
    path=Path(config['navigation']['static_map_lcm']);grid=Grid.lcm_decode(path.read_bytes())
    generated=yaml.safe_load((ROOT/'generated/static_map.yaml').read_text())
    resolution=float(grid.info.resolution)
    records=[];travel_records=[]
    for file in sorted((ROOT/'results').glob('mission_*.json')):
        record=json.loads(file.read_text())
        if record.get('path') and record.get('start') and record.get('goal'):
            records.append({'file':file.name,'execute':bool(record.get('execute')),
                            **distance_metrics(record['start'],record['goal'],record['path'])})
        if record.get('execute') and len(record.get('trace',[]))>2:
            travel_records.append({'file':file.name,'planned_length_m':record.get('planned_length_m'),
                                   'passed':bool(record.get('passed')),'error':record.get('error'),**motion_metrics(record['trace'])})
    rotation=mount_matrix(config)[:3,:3]
    checks={'source_generated_resolution_equal':abs(resolution-generated['resolution'])<1e-9,
            'resolution_positive_finite':math.isfinite(resolution) and resolution>0,
            'mount_is_rigid_no_scale':bool(np.allclose(rotation.T@rotation,np.eye(3)) and np.isclose(np.linalg.det(rotation),1))}
    return {'offline_only':True,'checks':checks,'passed':all(checks.values()),'resolution_m_per_cell':resolution,
            'map_extent_m':[grid.info.width*resolution,grid.info.height*resolution],
            'source_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'route_records':records,'travel_records':travel_records,
            'physical_scale_verified':False,'scale_correction_applied':False,
            'meaning':'Grid and transform units audited. Physical distance requires an independently measured straight segment; no 1/3 rescaling is inferred.'}


def compare_measurement(displacement,path_length,measured):
    if not all(math.isfinite(v) for v in (displacement,path_length,measured)) or measured<.5:
        raise ValueError('Use a measured straight segment of at least 0.5 m')
    ratio=displacement/measured
    return {'measured_straight_m':measured,'lio_displacement_m':displacement,'lio_path_m':path_length,
            'lio_to_measured_ratio':ratio,'relative_error':abs(ratio-1),
            'physical_scale_verified':.9<=ratio<=1.1 and path_length<=1.15*max(displacement,.01),
            'scale_correction_applied':False}


def capture(seconds,measured):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from tf2_ros import Buffer,TransformListener
    rclpy.init();node=Node('distance_audit_subscribe_only');buffer=Buffer();listener=TransformListener(buffer,node)
    state={};samples=[]
    node.create_subscription(String,'/navigation/safety_status',lambda m:state.update(gate=(time.monotonic(),json.loads(m.data))),10)
    def unarmed():
        stamp,gate=state.get('gate',(0,{}))
        if time.monotonic()-stamp>.5 or gate.get('armed') is not False:
            raise RuntimeError('Fresh DISARMED navigation required; this recorder never sends motion commands')
    try:
        end=time.monotonic()+3
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.03)
        unarmed();initial=buffer.lookup_transform('map','base_link',rclpy.time.Time())
        print(f'START: record {seconds:g} s. Move manually along the measured straight segment, then stop before the end. No autonomous command is sent.',flush=True)
        begin=time.monotonic()
        while time.monotonic()-begin<seconds:
            rclpy.spin_once(node,timeout_sec=.04);unarmed()
            tf=buffer.lookup_transform('map','base_link',rclpy.time.Time())
            stamp=tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9
            if not 0<=time.time()-stamp<.5:raise RuntimeError('Pose stale or invalid timestamp')
            if not samples or stamp>samples[-1][0]:
                p=tf.transform.translation;samples.append([stamp,p.x,p.y])
        if len(samples)<20:raise RuntimeError('Insufficient fresh position samples')
        first=np.array([initial.transform.translation.x,initial.transform.translation.y])
        last=np.median(np.array(samples)[-min(10,len(samples)):,1:],axis=0)
        displacement=float(np.linalg.norm(last-first))
        length=sum(math.dist(a[1:],b[1:]) for a,b in zip(samples,samples[1:]))
        report={'subscribe_only':True,'seconds':seconds,'samples':samples,**compare_measurement(displacement,length,measured)}
        file=ROOT/'results'/f'measured_distance_{time.time_ns()}.json'
        file.write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(json.dumps({k:v for k,v in report.items() if k!='samples'},indent=2));print(file)
    finally:
        node.destroy_node();rclpy.shutdown()


def main():
    p=argparse.ArgumentParser(description='No motion publisher, no automatic scale correction')
    p.add_argument('--record-seconds',type=float);p.add_argument('--measured-m',type=float)
    args=p.parse_args()
    if args.record_seconds is None:
        if args.measured_m is not None:p.error('--measured-m requires --record-seconds')
        report=offline_audit();(ROOT/'results/distance_audit_20261023.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2));return
    if not math.isfinite(args.record_seconds) or not 10<=args.record_seconds<=180:p.error('Record 10..180 seconds')
    if args.measured_m is None or not math.isfinite(args.measured_m) or args.measured_m<.5:p.error('Specify a measured segment >=0.5 m')
    capture(args.record_seconds,args.measured_m)


if __name__=='__main__':main()
