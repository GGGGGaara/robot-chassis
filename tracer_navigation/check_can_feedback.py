"""Receive-only CAN preflight. Success requires fresh state AND wheel frames."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import socket
import struct
import time


def receive(interface, seconds):
    ids = (0x211, 0x221)
    counts = {hex(i): 0 for i in ids}
    last = {}
    start = time.monotonic()
    with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
        bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                       b''.join(struct.pack('=II', i, 0xC00007FF) for i in ids))
        bus.bind((interface,))
        bus.settimeout(.1)
        while time.monotonic() - start < seconds:
            try:
                data = bus.recv(16)
            except socket.timeout:
                continue
            ident, dlc, payload = struct.unpack('=IB3x8s', data)
            if ident not in ids or dlc != 8:
                continue
            counts[hex(ident)] += 1
            last[hex(ident)] = time.monotonic()
    end = time.monotonic()
    ages = {hex(i): end - last[hex(i)] if hex(i) in last else None for i in ids}
    # These are only a link-presence check, not a replacement for the 30 s
    # zero-only driver test and its kernel-timestamp watchdogs.
    passed = all(counts[hex(i)] >= 2 and ages[hex(i)] <= .6 for i in ids)
    return {'timestamp': datetime.now(timezone.utc).isoformat(),
            'read_only': True, 'interface': interface, 'duration_s': end - start,
            'counts': counts, 'last_receive_age_s': ages, 'passed': passed,
            'meaning': 'Feedback present only; not hardware motion acceptance'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', default='can0')
    parser.add_argument('--seconds', type=float, default=4.)
    parser.add_argument('--output', type=Path,
                        default=Path(__file__).with_name('can_feedback_preflight.json'))
    args = parser.parse_args()
    if not 1 <= args.seconds <= 60:
        parser.error('--seconds must be between 1 and 60')
    try:
        report = receive(args.interface, args.seconds)
    except OSError as error:
        report = {'timestamp': datetime.now(timezone.utc).isoformat(),
                  'read_only': True, 'interface': args.interface,
                  'passed': False, 'error': str(error)}
    args.output.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))
    if not report['passed']:
        print('CAN feedback missing or stale. Stop here; inspect power, cable, USB attachment and CAN settings.')
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
