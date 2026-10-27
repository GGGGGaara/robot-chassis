# -*- coding: utf-8 -*-
"""
RPLIDAR (思岚 SLAMTEC) 激光雷达点云抓取脚本
协议: UART 串口, 标准扫描模式
用法:
    python rplidar_capture.py detect                    # 列出所有串口, 找到 RPLIDAR
    python rplidar_capture.py capture COM5 10 out.csv   # 抓 10 秒点云存 out.csv
    python rplidar_capture.py capture COM5 10 out.csv --baud 256000   # A3/S1 用 256000
"""

import sys
import time
import math
import argparse

import serial
from serial.tools import list_ports


# ---- RPLIDAR 协议常量 ----
SCAN_CMD = b"\xA5\x20"        # 标准扫描
FORCE_SCAN_CMD = b"\xA5\x25"  # 强制扫描(连续)
DESCRIPTOR_LEN = 7            # 响应描述符长度


def detect():
    ports = list_ports.comports()
    if not ports:
        print("没有找到任何串口。请确认 RPLIDAR 已通过 USB 线接到电脑。")
        return []
    print(f"{'端口':<10} {'描述':<40} {'硬件ID'}")
    for p in ports:
        print(f"{p.device:<10} {p.description:<40} {p.hwid}")
    return ports


def read_response(ser):
    """读取 7 字节响应描述符, 校验 A5 5A 头"""
    buf = ser.read(DESCRIPTOR_LEN)
    if len(buf) < DESCRIPTOR_LEN:
        return None
    if buf[0] != 0xA5 or buf[1] != 0x5A:
        return None
    return buf


def decode_packet(pkt):
    """解析 5 字节数据包 -> (quality, angle_deg, distance_mm, start_flag)"""
    b0, b1, b2, b3, b4 = pkt
    quality = b0 >> 2
    angle_q6 = b1 | (b2 << 8)
    distance_q2 = b3 | (b4 << 8)
    angle = angle_q6 / 64.0
    distance = distance_q2 / 4.0
    start = b0 & 0x01
    return quality, angle, distance, start


def capture(port, seconds, outfile, baud=115200):
    print(f"打开 {port} @ {baud} ...")
    ser = serial.Serial(port, baud, timeout=1)
    # 让电机转起来, 发送强制扫描
    ser.write(FORCE_SCAN_CMD)
    resp = read_response(ser)
    if resp is None:
        print("未收到有效响应描述符。请确认:")
        print("  1. 波特率是否正确 (A1/A2 默认 115200, A3/S1/S2 用 --baud 256000)")
        print("  2. 这个端口确实是 RPLIDAR")
        ser.close()
        return 0
    print(f"响应: {resp.hex().upper()}, 开始抓取 {seconds}s ...")

    n = 0
    start_time = time.time()
    with open(outfile, "w", encoding="utf-8") as f:
        f.write("angle_deg,distance_mm,quality,x_mm,y_mm\n")
        while time.time() - start_time < seconds:
            # 数据包 5 字节
            pkt = ser.read(5)
            if len(pkt) < 5:
                continue
            quality, angle, distance, start = decode_packet(pkt)
            if distance == 0:  # 无回波点
                continue
            rad = math.radians(angle)
            x = distance * math.cos(rad)
            y = distance * math.sin(rad)
            f.write(f"{angle:.2f},{distance:.1f},{quality},{x:.1f},{y:.1f}\n")
            n += 1
    ser.close()
    print(f"完成: 抓到 {n} 个有效点, 已存 {outfile}")
    return n


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd")

    d = sub.add_parser("detect")

    c = sub.add_parser("capture")
    c.add_argument("port")
    c.add_argument("seconds", type=float)
    c.add_argument("outfile")
    c.add_argument("--baud", type=int, default=115200)

    args = ap.parse_args()
    if args.cmd == "detect":
        detect()
    elif args.cmd == "capture":
        capture(args.port, args.seconds, args.outfile, args.baud)
    else:
        ap.print_help()
