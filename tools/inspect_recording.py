#!/usr/bin/env python3
import struct
import sys
import zlib
from collections import defaultdict

HEADER_FMT = "<IHHIQQQII"
HEADER_SIZE = struct.calcsize(HEADER_FMT)
MAGIC = 0x41564452

SENSOR_NAMES = {
    1: "CAMERA_4K",
    2: "LIDAR_128CH",
    3: "RADAR",
    4: "IMU",
    5: "VEHICLE_CAN",
    6: "GPS",
}

def inspect_log(filepath: str) -> None:
    stats = defaultdict(lambda: {"count": 0, "bytes": 0, "crc_err": 0, "latencies_us": []})

    with open(filepath, "rb") as f:
        while True:
            hdr_bytes = f.read(HEADER_SIZE)
            if not hdr_bytes:
                break
            if len(hdr_bytes) < HEADER_SIZE:
                print("[WARN] Truncated packet header at EOF")
                break

            magic, ver, stype, sid, seq, hw_ts, ingest_ts, plen, crc = struct.unpack(
                HEADER_FMT, hdr_bytes
            )
            if magic != MAGIC:
                print(f"[ERROR] Invalid magic number: {hex(magic)}")
                sys.exit(1)

            payload = f.read(plen)
            if len(payload) < plen:
                print("[WARN] Truncated payload at EOF")
                break

            calc_crc = zlib.crc32(payload) & 0xFFFFFFFF
            s_entry = stats[stype]
            s_entry["count"] += 1
            s_entry["bytes"] += HEADER_SIZE + plen
            if calc_crc != crc:
                s_entry["crc_err"] += 1
            if ingest_ts >= hw_ts:
                s_entry["latencies_us"].append((ingest_ts - hw_ts) / 1000.0)

    print(f"\n=== Python Fleet Log Analyzer: {filepath} ===")
    for stype, data in sorted(stats.items()):
        name = SENSOR_NAMES.get(stype, f"UNKNOWN({stype})")
        lats = data["latencies_us"]
        avg_lat = sum(lats) / len(lats) if lats else 0.0
        max_lat = max(lats) if lats else 0.0
        print(
            f"  {name:<14} | Packets: {data['count']:<5} | "
            f"Size: {data['bytes'] / (1024*1024):6.2f} MB | "
            f"Avg Ingest Lat: {avg_lat:6.2f} us | Max: {max_lat:6.2f} us | "
            f"CRC Errors: {data['crc_err']}"
        )

if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else "av_fleet_capture.bin"
    inspect_log(path)
