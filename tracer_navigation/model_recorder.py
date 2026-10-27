"""Subscribe-only DimOS model recorder. No ROS actions or velocity publishers."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import time

import lcm
import numpy as np

from common import settings,lcm_types,cloud_xyz,transform_matrix


def registered_points(cloud,fix):
    if cloud.header.frame_id!='odom':raise ValueError('Model must be in odom frame')
    if fix.shape!=(4,4) or not np.isfinite(fix).all():raise ValueError('Invalid map transform')
    matrix=np.linalg.inv(fix)
    xyz=cloud_xyz(cloud)
    return np.asarray(xyz@matrix[:3,:3].T+matrix[:3,3],dtype='<f4')


def write_ply(path,xyz):
    xyz=np.asarray(xyz,dtype='<f4')
    if xyz.ndim!=2 or xyz.shape[1]!=3 or not np.isfinite(xyz).all():raise ValueError('Invalid model points')
    header=('ply\nformat binary_little_endian 1.0\ncomment coordinates: map frame, metres\n'
            f'element vertex {len(xyz)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n').encode()
    with path.open('wb') as stream:stream.write(header);stream.write(xyz.tobytes())


class Recorder:
    def __init__(self,output,interval):
        self.output=output;self.interval=interval;self.bus=lcm.LCM();self.running=True
        self.TF,self.Cloud,_=lcm_types(settings());self.fix=None;self.latest=None;self.last_save=0.;self.last_stamp=0.;self.saved=[]
        output.mkdir(parents=True,exist_ok=True)
        self.bus.subscribe(re.escape('/tf#tf2_msgs.TFMessage'),self.transforms)
        self.bus.subscribe(re.escape('/global_map#sensor_msgs.PointCloud2'),self.cloud)

    def transforms(self,channel,raw):
        for item in self.TF.lcm_decode(raw).transforms:
            stamp=item.header.stamp.sec+item.header.stamp.nsec*1e-9
            if (item.header.frame_id,item.child_frame_id)==('odom','map') and abs(time.time()-stamp)<.5:
                self.fix=(transform_matrix(item.transform),stamp,time.monotonic())

    def cloud(self,channel,raw):
        msg=self.Cloud.lcm_decode(raw);stamp=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9
        if msg.header.frame_id=='odom' and msg.width*msg.height>0 and abs(time.time()-stamp)<2.:
            # Freeze the alignment associated with this observation. Never apply a later session's transform.
            if self.fix and time.monotonic()-self.fix[2]<.5:
                self.latest=(raw,msg,stamp,self.fix)

    def save(self):
        if self.latest is None or self.latest[2]==self.last_stamp:return
        raw,msg,stamp,(fix,fix_stamp,_)=self.latest
        xyz=registered_points(msg,fix)
        # A complete snapshot is committed by renaming its own directory; manifest is the final pointer.
        name=f'model_{int(stamp*1e9)}';temp=self.output/(name+'.tmp');final=self.output/name
        temp.mkdir()
        (temp/'global_map.pc2.lcm').write_bytes(raw)
        write_ply(temp/'model_map.ply',xyz)
        metadata={'timestamp':stamp,'alignment_timestamp':fix_stamp,'frame':'map','units':'m',
                  'points':len(xyz),'odom_T_map':fix.tolist(),'raw_cloud_frame':'odom',
                  'sha256':hashlib.sha256(raw).hexdigest(),'no_motion_commands':True}
        (temp/'metadata.json').write_text(json.dumps(metadata,indent=2),encoding='utf-8')
        os.replace(temp,final)
        pointer=self.output/'latest.json.tmp';pointer.write_text(json.dumps({'snapshot':name,**metadata},indent=2),encoding='utf-8')
        os.replace(pointer,self.output/'latest.json')
        # Keep just current and previous snapshots to bound disk use during long runs.
        self.saved.append(final)
        if len(self.saved)>2:
            old=self.saved.pop(0)
            if old.resolve().parent!=self.output.resolve():raise RuntimeError('Snapshot cleanup path mismatch')
            for item in old.iterdir():item.unlink()
            old.rmdir()
        self.last_stamp=stamp;self.last_save=time.monotonic()
        print(f'Model saved: {final / "model_map.ply"} ({len(xyz)} points, metres)',flush=True)

    def run(self):
        for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:setattr(self,'running',False))
        try:
            while self.running:
                self.bus.handle_timeout(100)
                if time.monotonic()-self.last_save>=self.interval:self.save()
        finally:self.save()


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path);p.add_argument('--interval-s',type=float,default=10.)
    args=p.parse_args()
    if not 2<=args.interval_s<=300:p.error('Save interval must be 2..300 seconds')
    Recorder(args.output,args.interval_s).run()


if __name__=='__main__':main()
