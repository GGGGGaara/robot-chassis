"""Subscribe-only snapshot of a standalone mapping backend's complete pointcloud."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import sys
import time
import numpy as np
import lcm

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tracer_navigation'))
from common import settings,lcm_types,cloud_xyz
from model_recorder import write_ply


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',required=True,type=Path)
    parser.add_argument('--seconds',type=float,default=15.)
    args=parser.parse_args()
    if not 5<=args.seconds<=600:parser.error('--seconds must be 5..600')
    if args.output_dir.exists():raise SystemExit('Output exists; refusing to overwrite floor capture')
    _,Cloud,_=lcm_types(settings());latest=[];bus=lcm.LCM()
    def receive(channel,raw):
        cloud=Cloud.lcm_decode(raw);stamp=cloud.header.stamp.sec+cloud.header.stamp.nsec*1e-9
        if cloud.header.frame_id in ('odom','map') and 0<=time.time()-stamp<5:
            latest[:]=[(raw,cloud,stamp)]
    bus.subscribe('^'+re.escape('/global_map#sensor_msgs.PointCloud2')+'$',receive)
    print('Subscribe-only floor snapshot. Keep standalone mapping running until this command finishes.',flush=True)
    end=time.monotonic()+args.seconds
    while time.monotonic()<end:bus.handle_timeout(100)
    if not latest:raise SystemExit('No fresh complete model received; no output created')
    raw,cloud,stamp=latest[0];xyz=cloud_xyz(cloud)
    if len(xyz)<1000:raise SystemExit('Too few model points for a floor reference; no output created')
    args.output_dir.mkdir(parents=True)
    (args.output_dir/'source_cloud.pc2.lcm').write_bytes(raw)
    write_ply(args.output_dir/'source_cloud.ply',xyz)
    metadata={'frame':cloud.header.frame_id,'units':'m','points':len(xyz),'source_stamp':stamp,
        'z_percentiles_m':np.percentile(xyz[:,2],[0,5,25,50,95,100]).tolist(),
        'source_sha256':hashlib.sha256(raw).hexdigest(),'no_motion_commands':True,
        'note':'Merged pointcloud snapshot only. Retain the original lidar/IMU/pose recording separately for drift/loop-closure checks.'}
    (args.output_dir/'capture.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print(json.dumps(metadata,indent=2))


if __name__=='__main__':main()
