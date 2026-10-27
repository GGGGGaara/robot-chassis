"""Measure CAN delivery independently of ROS; receive only, no CAN writes."""
import argparse
import json
import socket
import struct
import time
from pathlib import Path
import numpy as np

p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=15);p.add_argument('--label',default='driver_running')
args=p.parse_args();bus=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
bus.setsockopt(socket.SOL_CAN_RAW,socket.CAN_RAW_FILTER,b''.join(struct.pack('=II',i,0xC00007FF) for i in (0x211,0x221)))
bus.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,1048576)
bus.setsockopt(socket.SOL_SOCKET,35,1)  # SO_TIMESTAMPNS, kernel receive wall time
bus.bind(('can0',));bus.settimeout(.1)
start=time.monotonic();rows={0x211:[],0x221:[]};latencies=[]
try:
    while time.monotonic()-start<args.seconds:
        try:data,anc,flags,addr=bus.recvmsg(16,128)
        except socket.timeout:continue
        ident,dlc,payload=struct.unpack('=IB3x8s',data)
        if ident not in rows:continue
        now=time.time();stamp=now
        for level,kind,value in anc:
            if level==socket.SOL_SOCKET and kind==35:
                sec,nsec=struct.unpack('=qq',value[:16]);stamp=sec+nsec*1e-9
        rows[ident].append([stamp,now,payload[:dlc].hex()]);latencies.append(now-stamp)
finally:bus.close()
stats={}
for ident,values in rows.items():
    intervals=np.diff([v[0] for v in values])
    stats[hex(ident)]={'frames':len(values),'frequency_hz':len(values)/args.seconds,
        'kernel_gap_percentiles_s':np.percentile(intervals,[50,95,99,100]).tolist() if len(intervals) else [],
        'gaps_over_300ms':int(np.count_nonzero(intervals>.3))}
report={'read_only':True,'label':args.label,'timestamp':time.time(),'duration_s':args.seconds,
    'statistics':stats,'userspace_latency_percentiles_s':np.percentile(latencies,[50,99,100]).tolist() if latencies else [],'frames':rows}
file=Path(__file__).with_name('can_capture_'+args.label+'.json');file.write_text(json.dumps(report))
report.pop('frames');print(json.dumps(report,indent=2))
