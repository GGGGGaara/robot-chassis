"""Require fresh live sensors and a map alignment before starting Nav2."""
import re
import time
import lcm
from common import settings,lcm_types

TF,Cloud,_=lcm_types(settings());bus=lcm.LCM();seen={};started=time.monotonic();last_notice=0.
def transforms(channel,raw):
    for t in TF.lcm_decode(raw).transforms:
        if abs(time.time()-(t.header.stamp.sec+t.header.stamp.nsec*1e-9))<.5:
            if (t.header.frame_id,t.child_frame_id) in (('odom','map'),('odom','mid360_link')):
                seen[t.child_frame_id]=time.monotonic()
def cloud(channel,raw):
    m=Cloud.lcm_decode(raw)
    if m.header.frame_id=='mid360_link' and m.width*m.height>=200 and abs(time.time()-(m.header.stamp.sec+m.header.stamp.nsec*1e-9))<.35:
        seen['cloud']=time.monotonic()
bus.subscribe(re.escape('/tf#tf2_msgs.TFMessage'),transforms)
bus.subscribe(re.escape('/lidar#sensor_msgs.PointCloud2'),cloud)
while True:
    bus.handle_timeout(100);now=time.monotonic()
    if all(now-seen.get(key,float('-inf'))<.35 for key in ('map','mid360_link','cloud')):
        print('Fresh LiDAR, odometry and fixed-map alignment ready; starting navigation preview.',flush=True);break
    if now-last_notice>10:
        print('Waiting for live LiDAR and fixed-map relocalization; Nav2 not started yet.',flush=True);last_notice=now
    if now-started>180:raise SystemExit('Localization readiness timed out; keep robot stopped and check DimOS.')
