"""Read-only ROS/CAN capture: no publishers and no CAN writes."""
import json
import argparse
import socket
import struct
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tracer_msgs.msg import TracerStatus
from common import settings,driver_fingerprint,software_fingerprint


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--seconds',type=float,default=15)
    args=parser.parse_args()
    if not 5<=args.seconds<=120:raise SystemExit('duration must be 5..120 seconds')
    rclpy.init(); node = Node('driver_readonly_diagnosis')
    sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.setsockopt(socket.SOL_CAN_RAW,socket.CAN_RAW_FILTER,b''.join(struct.pack('=II',v,0xC00007FF) for v in (0x211,0x221)))
    sock.setsockopt(socket.SOL_SOCKET,35,1)
    sock.bind(('can0',)); sock.setblocking(False)
    started = time.monotonic(); last = {}; kernel_last={};gaps = {}; samples = []; counts = {}
    status = {}; gate = {}; previous = None;last_ready=0.;last_status=0.;diagnostics=[]
    def ready(msg):
        nonlocal previous,last_ready
        now = time.monotonic()
        last_ready=now
        counts[str(msg.data)] = counts.get(str(msg.data), 0) + 1
        if not msg.data or previous != msg.data:
            samples.append({'elapsed_s': now-started, 'ready': msg.data,
                            'raw_ages_s': {hex(k): now-v for k,v in last.items()},
                            'status': dict(status), 'gate': dict(gate)})
        previous = msg.data
    def chassis(msg):
        nonlocal last_status
        last_status=time.monotonic()
        status.update(mode=msg.control_mode, error=msg.error_code,
                      linear=msg.linear_velocity, angular=msg.angular_velocity)
    node.create_subscription(Bool, '/tracer/ready', ready, 100)
    node.create_subscription(TracerStatus, '/tracer_status', chassis, 100)
    node.create_subscription(String, '/navigation/safety_status', lambda m: gate.update(json.loads(m.data)), 10)
    node.create_subscription(String, '/tracer/driver_diagnostics',lambda m:diagnostics.append(json.loads(m.data)),100)
    try:
        while time.monotonic()-started < args.seconds:
            while True:
                try: data, ancillary, flags, address = sock.recvmsg(16,128)
                except BlockingIOError: break
                ident, length, payload = struct.unpack('=IB3x8s', data)
                if ident not in (0x211, 0x221): continue
                stamp=None
                for level,kind,value in ancillary:
                    if level==socket.SOL_SOCKET and kind==35:
                        seconds,nanos=struct.unpack('=qq',value[:16]);stamp=seconds+nanos*1e-9
                if stamp is None:continue
                now = time.monotonic()
                if ident in kernel_last:gaps[hex(ident)] = max(gaps.get(hex(ident), 0), stamp-kernel_last[ident])
                last[ident] = now;kernel_last[ident]=stamp
                if ident == 0x211: status['raw_system'] = payload[:length].hex()
            rclpy.spin_once(node, timeout_sec=.005)
        now=time.monotonic();wall_now=time.time();config=settings()
        stable=bool(counts.get('True',0)>args.seconds*25 and counts.get('False',0)==0 and now-last_ready<.3 and now-last_status<.3
            and status.get('mode')==1 and status.get('error')==0
            and wall_now-kernel_last.get(0x211,0)<config['robot']['system_feedback_timeout_s']
            and wall_now-kernel_last.get(0x221,0)<.3)
        report = {'read_only': True, 'timestamp':wall_now,'duration_s': time.monotonic()-started,'stable_ready':stable,
                  'ready_counts': counts, 'max_raw_receive_gap_s': gaps,
                  'gap_clock':'kernel receive timestamp','driver_faults':[v for v in diagnostics if not v.get('ready')],
                  'events': samples, 'final_status': status, 'final_gate': gate,
                  'driver_sha256':driver_fingerprint(),'software_sha256':software_fingerprint()}
        path = Path(__file__).with_name('driver_readonly_diagnosis.json')
        path.write_text(json.dumps(report, indent=2))
        folder=Path(__file__).with_name('results');folder.mkdir(exist_ok=True)
        (folder/f'driver_diagnosis_{int(wall_now)}.json').write_text(json.dumps(report,indent=2))
        print(json.dumps({'read_only': True, 'duration_s': report['duration_s'],
                          'ready_counts': counts, 'max_raw_receive_gap_s': gaps,
                          'stable_ready': stable,'driver_faults':report['driver_faults'],
                          'final_status': status, 'final_gate': gate,
                          'detail_file': str(path)}, indent=2))
    finally:
        sock.close(); node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
