#!/usr/bin/env python3
"""Convert a GrassVIO RealSense rosbag2 .db3 file to TUM-VI style files.

Reads the rosbag2 sqlite schema directly (tables: topics, messages) and
decodes sensor_msgs/msg/Image and sensor_msgs/msg/Imu payloads with the
layout used by the recorder (no ROS installation needed):

  * Header is serialized WITHOUT the `seq` field: int32 sec @4,
    uint32 nsec @8, then a length-prefixed frame_id string.
  * Imu messages: frame_id string data field is 12 bytes wide, so the
    37 float64s start at offset 28. Total message = 324 bytes.
  * Image messages: frame_id string data padded to 4, then height,
    width, encoding, is_bigendian, step, data.

Output layout (one directory per take):
  rgb/         <timestamp>.png   (RGB8 color frames)
  depth/       <timestamp>.png   (16-bit depth, raw units, 0.001 m per unit)
  rgb.txt      <timestamp> rgb/<timestamp>.png
  depth.txt    <timestamp> depth/<timestamp>.png
  imu.txt      <timestamp> gx gy gz ax ay az   (TUM-VI order)
  info.json    counts, rates, timestamp ranges, pairing stats

Timestamps are the sensor header stamps in seconds. Gyro runs at
~200 Hz and accel at ~100 Hz on these bags, so accel is linearly
interpolated onto the gyro timestamps for the combined imu.txt.
Depth frames are paired to color frames by nearest header timestamp;
pairs with |offset| > 20 ms are dropped and counted.

Usage:
    python3 convert_bag.py <bag.db3> <outdir> [--max-offset-ms 20]
"""

import argparse
import json
import os
import sqlite3
import struct
import sys

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None

GYRO = "/device_0/sensor_2/Gyro_0/imu/data"
ACCEL = "/device_0/sensor_2/Accel_0/imu/data"
DEPTH = "/device_0/sensor_0/Depth_0/image/data"
COLOR = "/device_0/sensor_1/Color_0/image/data"


def parse_header(blob):
    sec, nsec = struct.unpack("<iI", blob[4:12])
    ln = struct.unpack("<I", blob[12:16])[0]
    frame_id = blob[16:16 + ln - 1].decode("utf-8", "replace")
    return sec * 1_000_000_000 + nsec, frame_id


def parse_imu(blob):
    assert len(blob) == 324, "unexpected Imu blob size %d" % len(blob)
    stamp_ns, frame_id = parse_header(blob)
    d = struct.unpack("<37d", blob[28:324])
    return stamp_ns, np.array(d[13:16]), np.array(d[25:28])


def parse_image(blob):
    stamp_ns, frame_id = parse_header(blob)
    ln = struct.unpack("<I", blob[12:16])[0]
    off = 16 + ((ln + 3) // 4) * 4
    height, width = struct.unpack("<2I", blob[off:off + 8])
    off += 8
    eln = struct.unpack("<I", blob[off:off + 4])[0]
    off += 4
    encoding = blob[off:off + eln - 1].decode("utf-8", "replace")
    off += eln
    is_bigendian = blob[off]
    off += 1
    while off % 4:
        off += 1
    step = struct.unpack("<I", blob[off:off + 4])[0]
    off += 4
    dlen = struct.unpack("<I", blob[off:off + 4])[0]
    off += 4
    data = blob[off:off + dlen]
    assert off + dlen == len(blob), "image size mismatch"
    return stamp_ns, height, width, encoding, data


def read_all(db, topic):
    tid = db.execute(
        "SELECT id FROM topics WHERE name=?", (topic,)).fetchone()
    if tid is None:
        raise SystemExit("topic not found: %s" % topic)
    rows = db.execute(
        "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (tid[0],)).fetchall()
    return [r[0] for r in rows]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("outdir")
    ap.add_argument("--max-offset-ms", type=float, default=20.0)
    args = ap.parse_args()

    if cv2 is None:
        raise SystemExit("opencv (cv2) is required to write PNGs")

    os.makedirs(os.path.join(args.outdir, "rgb"), exist_ok=True)
    os.makedirs(os.path.join(args.outdir, "depth"), exist_ok=True)
    db = sqlite3.connect("file:%s?mode=ro" % args.bag, uri=True)

    info = {"bag": os.path.basename(args.bag)}

    # ---- IMU: interpolate accel onto gyro timestamps ----
    gyro = [parse_imu(b) for b in read_all(db, GYRO)]
    accel = [parse_imu(b) for b in read_all(db, ACCEL)]
    g_ts = np.array([g[0] for g in gyro], dtype=np.float64)
    g_w = np.array([g[1] for g in gyro])
    a_ts = np.array([a[0] for a in accel], dtype=np.float64)
    a_a = np.array([a[2] for a in accel])
    a_interp = np.column_stack([
        np.interp(g_ts, a_ts, a_a[:, k]) for k in range(3)])
    keep = (g_ts >= a_ts[0]) & (g_ts <= a_ts[-1])
    info["gyro_n"] = len(gyro)
    info["accel_n"] = len(accel)
    info["imu_n"] = int(keep.sum())
    info["imu_t0_s"] = float(g_ts[keep][0] / 1e9)
    info["imu_t1_s"] = float(g_ts[keep][-1] / 1e9)
    info["gyro_rate_hz"] = float(len(gyro) / ((g_ts[-1] - g_ts[0]) / 1e9))
    info["accel_rate_hz"] = float(len(accel) / ((a_ts[-1] - a_ts[0]) / 1e9))
    with open(os.path.join(args.outdir, "imu.txt"), "w") as f:
        for i in np.nonzero(keep)[0]:
            f.write("%.9f %.9f %.9f %.9f %.9f %.9f %.9f\n" % (
                g_ts[i] / 1e9,
                g_w[i, 0], g_w[i, 1], g_w[i, 2],
                a_interp[i, 0], a_interp[i, 1], a_interp[i, 2]))

    # ---- Images: pair depth to color by nearest timestamp ----
    color = [parse_image(b) for b in read_all(db, COLOR)]
    depth = [parse_image(b) for b in read_all(db, DEPTH)]
    d_ts = np.array([d[0] for d in depth], dtype=np.float64)
    info["color_n"] = len(color)
    info["depth_n"] = len(depth)
    info["color_encoding"] = color[0][3]
    info["depth_encoding"] = depth[0][3]
    info["color_hw"] = [color[0][1], color[0][2]]
    info["depth_hw"] = [depth[0][1], depth[0][2]]
    info["color_t0_s"] = float(color[0][0] / 1e9)
    info["color_t1_s"] = float(color[-1][0] / 1e9)

    rgb_lines, depth_lines = [], []
    offsets_ms, dropped = [], 0
    for (ts, h, w, enc, data) in color:
        j = int(np.argmin(np.abs(d_ts - ts)))
        off_ms = abs(d_ts[j] - ts) / 1e6
        if off_ms > args.max_offset_ms:
            dropped += 1
            continue
        offsets_ms.append(off_ms)
        rgb_arr = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3)
        d_ts_j, dh, dw, denc, ddata = depth[j]
        dep_arr = np.frombuffer(ddata, dtype="<u2").reshape(dh, dw)
        name = "%.6f.png" % (ts / 1e9)
        cv2.imwrite(os.path.join(args.outdir, "rgb", name),
                    cv2.cvtColor(rgb_arr, cv2.COLOR_RGB2BGR))
        cv2.imwrite(os.path.join(args.outdir, "depth", name), dep_arr)
        rgb_lines.append("%.6f rgb/%s\n" % (ts / 1e9, name))
        depth_lines.append("%.6f depth/%s\n" % (ts / 1e9, name))
    with open(os.path.join(args.outdir, "rgb.txt"), "w") as f:
        f.writelines(rgb_lines)
    with open(os.path.join(args.outdir, "depth.txt"), "w") as f:
        f.writelines(depth_lines)
    info["paired_n"] = len(rgb_lines)
    info["dropped_pairs"] = dropped
    info["pair_offset_ms_mean"] = float(np.mean(offsets_ms))
    info["pair_offset_ms_max"] = float(np.max(offsets_ms))
    with open(os.path.join(args.outdir, "info.json"), "w") as f:
        json.dump(info, f, indent=1)
    print(json.dumps(info, indent=1))


if __name__ == "__main__":
    main()
