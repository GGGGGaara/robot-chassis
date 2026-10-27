"""Visualization-only relay: latest LiDAR frame, no accumulation or robot commands."""
import json
import re
import time

import lcm
from common import ROOT,settings,lcm_types


def main():
    _,Cloud,_=lcm_types(settings())
    bus=lcm.LCM();state={'last':0.,'last_sent':0.,'template':None,'frames':0,'points':0}
    output='/live_scan#sensor_msgs.PointCloud2'

    def receive(channel,raw):
        now=time.monotonic()
        if now-state['last_sent']<.2:return
        msg=Cloud.lcm_decode(raw)
        stamp=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9
        if abs(time.time()-stamp)>.5:return
        # Keep the original sensor frame and time; Rerun attaches it using TF.
        bus.publish(output,raw)
        state.update(last=now,last_sent=now,template=msg,frames=state['frames']+1,points=msg.width*msg.height)

    bus.subscribe(re.escape('/lidar#sensor_msgs.PointCloud2'),receive)
    last_report=0.
    print('Visualization only: world/live_scan, latest scan at up to 5 Hz.',flush=True)
    while True:
        bus.handle_timeout(100)
        now=time.monotonic()
        if state['template'] is not None and now-state['last']>.5:
            msg=state['template'];msg.width=0;msg.row_step=0;msg.data_length=0;msg.data=b''
            bus.publish(output,msg.lcm_encode());state['template']=None;state['points']=0
        if now-last_report>2:
            report={'visualization_only':True,'entity':'world/live_scan','frames':state['frames'],
                    'points':state['points'],'fresh':now-state['last']<=.5}
            (ROOT/'live_scan_status.json').write_text(json.dumps(report,indent=2))
            last_report=now


if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:pass
