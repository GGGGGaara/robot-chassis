"""Exercise missing/one-sided/stale/healthy feedback using virtual CAN only."""
import json
import socket
import struct
import threading
import time
from pathlib import Path
from check_can_feedback import receive
from common import require_virtual_can

require_virtual_can()
checks = {}
samples = []


def scenario(ids, send_for=1.1):
    stop = threading.Event()

    def produce():
        with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
            bus.bind(('vcan_navtest',))
            end = time.monotonic() + send_for
            while not stop.is_set() and time.monotonic() < end:
                for ident in ids:
                    bus.send(struct.pack('=IB3x8s', ident, 8, bytes(8)))
                stop.wait(.03)
    worker = threading.Thread(target=produce)
    worker.start()
    try:
        result = receive('vcan_navtest', 1.)
        samples.append(result)
        return result
    finally:
        stop.set()
        worker.join()


checks['no_frames_rejected'] = not scenario(())['passed']
checks['wheel_only_rejected'] = not scenario((0x221,))['passed']
checks['state_only_rejected'] = not scenario((0x211,))['passed']
checks['early_frames_then_silence_rejected'] = not scenario((0x211, 0x221), .15)['passed']
checks['both_fresh_accepted'] = scenario((0x211, 0x221))['passed']
checks['extended_frames_rejected'] = not scenario((0x80000211, 0x80000221))['passed']
report = {'interface': 'vcan_navtest', 'physical_can_touched': False,
          'checks': checks, 'samples': samples, 'passed': all(checks.values())}
Path(__file__).with_name('can_feedback_test_report.json').write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
raise SystemExit(0 if report['passed'] else 1)
