# -*- coding: utf-8 -*-
"""
CoDroid 雷达 CAN 抓包工具 (candleLight USB-CAN 适配器)
用法:
    python can_capture.py scan                       # 自动探测波特率
    python can_capture.py capture 500000 10 out.csv  # 500k 抓 10 秒存 out.csv
"""

import sys
import time
import argparse

# ---- Windows 下让 pyusb 使用 libusb-package 自带的 libusb-1.0.dll ----
import usb.backend.libusb1 as _libusb1
import libusb_package

_orig_get_backend = _libusb1.get_backend

def _patched_get_backend(find_library=None):
    if find_library is None:
        find_library = libusb_package.find_library
    return _orig_get_backend(find_library=find_library)

_libusb1.get_backend = _patched_get_backend

import can


# 常见雷达 CAN 波特率，按可能性排序
CANDIDATE_BITRATES = [500000, 250000, 1000000, 125000, 800000, 100000, 50000]


def open_bus(bitrate):
    return can.Bus(interface="gs_usb", channel=0, bitrate=bitrate)


def scan(listen_seconds=2.0):
    print(f"开始扫描 {len(CANDIDATE_BITRATES)} 个常见波特率，每个监听 {listen_seconds}s ...\n")
    results = []
    for br in CANDIDATE_BITRATES:
        count = 0
        sample_ids = []
        try:
            bus = open_bus(br)
            time.sleep(0.3)  # 等待设备稳定
            deadline = time.time() + listen_seconds
            while time.time() < deadline:
                msg = bus.recv(timeout=0.2)
                if msg is not None and msg.is_rx:
                    count += 1
                    if len(sample_ids) < 5:
                        sample_ids.append(hex(msg.arbitration_id))
            bus.shutdown()
        except Exception as e:
            print(f"  {br:>8} bps : 打开失败 ({e})")
            continue
        results.append((br, count))
        flag = "  <<< 有数据" if count > 0 else ""
        ids = "  ID: " + ", ".join(sample_ids) if sample_ids else ""
        print(f"  {br:>8} bps : {count:>6} 帧{flag}{ids}")

    print()
    detected = [br for br, c in results if c > 0]
    if detected:
        print(f"检测到有数据的波特率: {', '.join(str(b) for b in detected)}")
        print(f"推荐用: python can_capture.py capture {detected[0]} 10 out.csv")
    else:
        print("所有波特率都没有收到帧。请确认:")
        print("  1. 雷达已通电，并持续在 CAN 上广播")
        print("  2. CAN_H / CAN_L 接线正确，总线两端有 120Ω 终端电阻")
        print("  3. 雷达的波特率不在常见列表里")
    return detected


def capture(bitrate, seconds, outfile):
    print(f"开始抓取 @ {bitrate} bps，时长 {seconds}s，输出 {outfile} ...")
    bus = open_bus(bitrate)
    n = 0
    seen_ids = set()
    with open(outfile, "w", encoding="utf-8") as f:
        f.write("timestamp,arbitration_id,is_extended,dlc,data_hex\n")
        deadline = time.time() + seconds
        while time.time() < deadline:
            msg = bus.recv(timeout=0.2)
            if msg is None or not msg.is_rx:
                continue
            n += 1
            seen_ids.add(msg.arbitration_id)
            data_hex = "".join(f"{b:02X}" for b in msg.data)
            f.write(f"{msg.timestamp:.6f},0x{msg.arbitration_id:X},{msg.is_extended_id},{msg.dlc},{data_hex}\n")
            f.flush()
    bus.shutdown()
    print(f"完成: 抓到 {n} 帧, 唯一 CAN ID {len(seen_ids)} 个")
    if seen_ids:
        print("出现的 CAN ID:", ", ".join(f"0x{i:X}" for i in sorted(seen_ids)))
    return n


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd")

    s = sub.add_parser("scan")
    s.add_argument("--seconds", type=float, default=2.0)

    c = sub.add_parser("capture")
    c.add_argument("bitrate", type=int)
    c.add_argument("seconds", type=float)
    c.add_argument("outfile")

    args = ap.parse_args()
    if args.cmd == "scan":
        scan(args.seconds)
    elif args.cmd == "capture":
        capture(args.bitrate, args.seconds, args.outfile)
    else:
        ap.print_help()
