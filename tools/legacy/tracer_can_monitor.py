"""Read-only TRACER 2.0 CAN decoder for captured CSV files.

Does not transmit CAN frames. Protocol fields follow the TRACER 2.0 manual:
0x211 status, 0x221 body velocity, 0x311 wheel odometry, 0x251/0x252 motor RPM.
All multi-byte values are Motorola/big-endian.
"""
import argparse
import csv
import struct


def s16(b):
    return struct.unpack(">h", bytes(b))[0]


def s32(b):
    return struct.unpack(">i", bytes(b))[0]


def decode(row):
    aid = int(row["arbitration_id"], 16)
    d = bytes.fromhex(row["data_hex"])
    if aid == 0x211:
        return f"STATUS state=0x{d[0]:02X} mode=0x{d[1]:02X} battery={int.from_bytes(d[2:4], 'big')/10:.1f}V fault=0x{int.from_bytes(d[4:6], 'big'):04X} count={d[7]}"
    if aid == 0x221:
        return f"VELOCITY linear={s16(d[0:2])/1000:.3f}m/s angular={s16(d[2:4])/1000:.3f}rad/s"
    if aid == 0x311:
        return f"ODOM left={s32(d[0:4])}mm right={s32(d[4:8])}mm"
    if aid in (0x251, 0x252):
        return f"MOTOR id=0x{aid:X} rpm={s16(d[0:2])}"
    if aid in (0x261, 0x262):
        return f"MOTOR_STATUS id=0x{aid:X} status=0x{d[5]:02X}"
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv_file")
    ap.add_argument("--every", type=int, default=20, help="print every N decoded rows")
    args = ap.parse_args()
    n = 0
    with open(args.csv_file, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            msg = decode(row)
            if msg is not None:
                n += 1
                if n == 1 or n % args.every == 0:
                    print(f"{row['timestamp']} {msg}")
    print(f"decoded_rows={n}")


if __name__ == "__main__":
    main()
