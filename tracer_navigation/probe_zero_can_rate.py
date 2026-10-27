"""Physical CAN rate comparison with hardcoded ZERO speed frames only."""
import json
import socket
import struct
import subprocess
import time
from pathlib import Path
import numpy as np

if subprocess.run(['pgrep','-f','/tracer_safe_base( |$)'],stdout=subprocess.DEVNULL).returncode==0:
    raise SystemExit('Stop the hardware driver before the zero-only probe')
if 'LISTEN-ONLY' in subprocess.check_output(['ip','-details','link','show','can0'],text=True):
    raise SystemExit('CAN is listen-only: rate comparison cannot transmit and would be invalid')
ZERO=struct.pack('=IB3x8s',0x111,8,bytes(8))
bus=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
bus.setsockopt(socket.SOL_CAN_RAW,socket.CAN_RAW_FILTER,b''.join(struct.pack('=II',i,0xC00007FF) for i in (0x211,0x221)))
bus.setsockopt(socket.SOL_SOCKET,35,1);bus.bind(('can0',));bus.settimeout(.002)
reports=[]
try:
    for hz in (10,20,50):
        start=time.monotonic();next_tx=start;received={0x211:[],0x221:[]};errors=[];tx=0
        while time.monotonic()-start<10:
            now=time.monotonic()
            if now>=next_tx:
                try:bus.send(ZERO);tx+=1
                except OSError as error:errors.append(str(error))
                next_tx=now+1/hz
            try:data,anc,flags,addr=bus.recvmsg(16,128)
            except socket.timeout:continue
            ident,length,payload=struct.unpack('=IB3x8s',data)
            stamp=time.time()
            for level,kind,value in anc:
                if level==socket.SOL_SOCKET and kind==35:
                    sec,nsec=struct.unpack('=qq',value[:16]);stamp=sec+nsec*1e-9
            if ident in received:received[ident].append(stamp)
        stats={}
        for ident,stamps in received.items():
            gaps=np.diff(stamps)
            stats[hex(ident)]={'frames':len(stamps),'gap_percentiles_s':np.percentile(gaps,[50,95,99,100]).tolist() if len(gaps) else [],
                'gaps_over_300ms':int(np.count_nonzero(gaps>.3))}
        record={'zero_only':True,'command_hz':hz,'duration_s':10,'sent_zero_frames':tx,'send_errors':errors,'statistics':stats}
        record['valid_transmission_test']=not errors and tx>=hz*9
        reports.append(record)
        print(json.dumps({**record,'send_errors':{'count':len(errors),'first':errors[:2]}}),flush=True)
finally:
    for _ in range(3):
        try:bus.send(ZERO)
        except OSError:pass
        time.sleep(.02)
    bus.close()
    Path(__file__).with_name('zero_can_rate_report.json').write_text(json.dumps(reports,indent=2))
