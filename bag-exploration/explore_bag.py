#!/usr/bin/env python3
"""Explore a ROS 2 sqlite (.db3) RealSense recording for GrassVIO.

Reads the rosbag2 sqlite schema directly (tables: topics, messages) and
decodes sensor_msgs/msg/Image and sensor_msgs/msg/Imu payloads with the
layout actually used by the recorder. No ROS installation needed.

Recorder layout quirks (verified empirically on the Sep-26 bags):
  * std_msgs/Header is serialized WITHOUT the `seq` field:
    int32 sec @4, uint32 nsec @8, then a length-prefixed frame_id string.
  * The string padding bytes are uninitialized garbage (not zeros) -- the
    parser skips padding without validating it.
  * Imu messages: frame_id string data field is 12 bytes wide, so the 37
    float64s start at offset 28. Total message = 324 bytes.
  * Image messages: frame_id string data padded to 4 as usual, then
    height, width, encoding, is_bigendian, step, data.

Usage:
    python3 explore_bag.py <bag.db3> [--out report.json] [--depth-samples N]
"""

import argparse
import json
import math
import os
import sqlite3
import struct

import numpy as np

GYRO = "/device_0/sensor_2/Gyro_0/imu/data"
ACCEL = "/device_0/sensor_2/Accel_0/imu/data"
DEPTH = "/device_0/sensor_0/Depth_0/image/data"
COLOR = "/device_0/sensor_1/Color_0/image/data"
DATA_TOPICS = (GYRO, ACCEL, DEPTH, COLOR)


# --------------------------------------------------------------------------
# Decoders for the recorder's actual layout
# --------------------------------------------------------------------------
def parse_header(blob):
    sec, nsec = struct.unpack("<iI", blob[4:12])
    ln = struct.unpack("<I", blob[12:16])[0]
    frame_id = blob[16:16 + ln - 1].decode("utf-8", "replace")
    return sec * 1_000_000_000 + nsec, frame_id


def parse_imu(blob):
    """Returns dict with stamp_ns, frame_id, orientation, ang_vel, lin_acc."""
    assert len(blob) == 324, f"unexpected Imu blob size {len(blob)}"
    stamp_ns, frame_id = parse_header(blob)
    d = struct.unpack("<37d", blob[28:324])
    return {
        "stamp_ns": stamp_ns, "frame_id": frame_id,
        "orientation": d[0:4],
        "angular_velocity": d[13:16],
        "linear_acceleration": d[25:28],
    }


def parse_image(blob):
    """Returns dict with stamp_ns, frame_id, height, width, encoding, step, data."""
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
    assert off + dlen == len(blob), f"image size mismatch {off}+{dlen}!={len(blob)}"
    return {
        "stamp_ns": stamp_ns, "frame_id": frame_id,
        "height": height, "width": width, "encoding": encoding,
        "is_bigendian": is_bigendian, "step": step, "data": data,
    }


def parse_ros_string(blob):
    ln = struct.unpack("<I", blob[4:8])[0]
    return blob[8:8 + ln - 1].decode("utf-8", "replace")


# --------------------------------------------------------------------------
# Stats helpers
# --------------------------------------------------------------------------
def ts_stats(ts_ns):
    ts = np.asarray(ts_ns, dtype=np.float64)
    n = len(ts)
    if n == 0:
        return None
    span_s = (ts[-1] - ts[0]) / 1e9 if n > 1 else 0.0
    out = {
        "count": int(n),
        "span_s": round(float(span_s), 4),
        "mean_rate_hz": round(float(n / span_s), 2) if span_s > 0 else None,
    }
    if n > 1:
        dt = np.diff(ts) / 1e9
        med = float(np.median(dt))
        out.update({
            "dt_median_ms": round(med * 1000, 4),
            "dt_mean_ms": round(float(np.mean(dt)) * 1000, 4),
            "dt_std_ms": round(float(np.std(dt)) * 1000, 4),
            "dt_min_ms": round(float(np.min(dt)) * 1000, 4),
            "dt_max_ms": round(float(np.max(dt)) * 1000, 4),
            "median_rate_hz": round(float(1.0 / med), 2) if med > 0 else None,
            "n_negative_dt": int(np.sum(dt < 0)),
            "n_duplicate_ts": int(np.sum(dt == 0)),
        })
        gap_thr = 2.5 * med
        big = np.where(dt > gap_thr)[0]
        out["n_gaps_over_2_5x_median"] = int(len(big))
        out["largest_gap_ms"] = round(float(np.max(dt)) * 1000, 3)
        out["gap_start_times_s"] = [round(float(ts[i] / 1e9), 3) for i in big[:10]]
    return out


def allan_deviation(x, fs, taus_s=(0.05, 0.1, 0.25, 0.5, 1.0)):
    """Overlapping Allan deviation of a 1-D series (cheap O(n) per tau)."""
    x = np.asarray(x, dtype=np.float64)
    res = {}
    for tau in taus_s:
        m = int(round(tau * fs))
        if m < 1 or 2 * m >= len(x):
            res[str(tau)] = None
            continue
        c = np.cumsum(np.insert(x, 0, 0.0))
        k = len(x) - 2 * m
        y1 = (c[m:m + k] - c[:k]) / m
        y2 = (c[2 * m:2 * m + k] - c[m:m + k]) / m
        res[str(tau)] = round(float(np.sqrt(0.5 * np.mean((y2 - y1) ** 2))), 6)
    return res


def imu_report(vecs, fs, unit, label):
    x = np.asarray(vecs, dtype=np.float64)
    rep = {"n": int(len(x)), "est_rate_hz": round(float(fs), 2), "unit": unit}
    mag = np.linalg.norm(x, axis=1)
    rep["magnitude"] = {
        "mean": round(float(np.mean(mag)), 5), "std": round(float(np.std(mag)), 6),
        "min": round(float(np.min(mag)), 5), "max": round(float(np.max(mag)), 5),
    }
    axes = {}
    for i, ax in enumerate("xyz"):
        col = x[:, i]
        bias = float(np.mean(col))
        std = float(np.std(col))
        axes[ax] = {
            "bias": round(bias, 6),
            "std": round(std, 6),
            "noise_density": round(std / math.sqrt(fs), 6) if fs > 0 else None,
            "n_spikes_over_6sigma": int(np.sum(np.abs(col - bias) > 6 * std)) if std > 0 else 0,
            "min": round(float(np.min(col)), 5), "max": round(float(np.max(col)), 5),
        }
    rep["axes"] = axes
    rep["allan_dev_tau_s"] = allan_deviation(mag, fs)
    # motion segmentation: what fraction of samples exceed gyro-rate thresholds
    if unit == "rad/s":
        rep["motion_fraction"] = {
            f"above_{thr}_rad_s": round(float(np.mean(mag > thr)), 4)
            for thr in (0.05, 0.1, 0.25, 0.5)
        }
        rep["magnitude"]["p50"] = round(float(np.median(mag)), 5)
        rep["magnitude"]["p95"] = round(float(np.percentile(mag, 95)), 5)
    return rep


# --------------------------------------------------------------------------
# Main exploration
# --------------------------------------------------------------------------
def explore(bag_path, depth_samples=20):
    db = sqlite3.connect(f"file:{bag_path}?mode=ro", uri=True)
    db.execute("PRAGMA query_only=ON")
    topics = {r[1]: (r[0], r[2]) for r in db.execute("select id,name,type from topics")}
    rep = {
        "bag": os.path.basename(bag_path),
        "size_mb": round(os.path.getsize(bag_path) / 1e6, 1),
        "n_topics_total": len(topics),
    }

    # ---- recorder metadata (camera info, options) ----
    meta = {}
    for name, (tid, typ) in topics.items():
        if typ != "std_msgs/msg/String":
            continue
        n = db.execute("select count(*) from messages where topic_id=?", (tid,)).fetchone()[0]
        if n != 1:
            continue
        blob = db.execute("select data from messages where topic_id=?", (tid,)).fetchone()[0]
        if any(k in name for k in ("camera_info", "/info", "/value", "imu_intrinsic")):
            meta[name] = parse_ros_string(blob)[:600]
    rep["metadata"] = meta
    # metadata/data count consistency
    consistency = {}
    for base, data_t in (("Gyro_0", GYRO), ("Accel_0", ACCEL), ("Depth_0", DEPTH), ("Color_0", COLOR)):
        mt = f"/device_0/sensor_2/{base}/imu/metadata" if "0/imu" in data_t or "Gyro" in base or "Accel" in base else None
        if base in ("Gyro_0", "Accel_0"):
            mt = f"/device_0/sensor_2/{base}/imu/metadata"
        else:
            mt = f"/device_0/sensor_0/{base}/image/metadata" if base == "Depth_0" else f"/device_0/sensor_1/{base}/image/metadata"
        if data_t in topics and mt in topics:
            nd = db.execute("select count(*) from messages where topic_id=?", (topics[data_t][0],)).fetchone()[0]
            nm = db.execute("select count(*) from messages where topic_id=?", (topics[mt][0],)).fetchone()[0]
            consistency[base] = {"data_n": nd, "metadata_n": nm, "match": nd == nm}
    rep["data_metadata_count_consistency"] = consistency

    # ---- per-stream decode ----
    streams = {}
    for name in DATA_TOPICS:
        if name not in topics:
            streams[name] = {"present": False}
            continue
        tid = topics[name][0]
        rows = db.execute(
            "select timestamp, data from messages where topic_id=? order by timestamp", (tid,)).fetchall()
        bag_ts = np.array([r[0] for r in rows], dtype=np.float64)
        if "imu/data" in name:
            parsed = [parse_imu(r[1]) for r in rows]
            hdr_ts = np.array([p["stamp_ns"] for p in parsed])
            info = {
                "present": True, "type": "sensor_msgs/msg/Imu",
                "frame_id": parsed[0]["frame_id"],
                "bag_timestamps": ts_stats(bag_ts),
                "header_timestamps": ts_stats(hdr_ts),
                "header_matches_bag_ts": bool(np.array_equal(bag_ts, hdr_ts)),
                "orientation": "identity (0,0,0,1)" if all(
                    p["orientation"][3] == 1.0 for p in parsed) else "VARIABLE - CHECK",
            }
            if "Gyro" in name:
                vecs = [p["angular_velocity"] for p in parsed]
                fs = 1.0 / (np.median(np.diff(hdr_ts)) / 1e9)
                info["gyro"] = imu_report(vecs, fs, "rad/s", "gyro")
                info["linear_acc_carried"] = bool(any(
                    abs(v) > 1e-9 for p in parsed for v in p["linear_acceleration"]))
            else:
                vecs = [p["linear_acceleration"] for p in parsed]
                fs = 1.0 / (np.median(np.diff(hdr_ts)) / 1e9)
                info["accel"] = imu_report(vecs, fs, "m/s^2", "accel")
                info["angular_vel_carried"] = bool(any(
                    abs(v) > 1e-9 for p in parsed for v in p["angular_velocity"]))
        else:
            parsed_hdr = []
            for _, blob in rows:
                stamp_ns, frame_id = parse_header(blob)
                parsed_hdr.append((stamp_ns, frame_id))
            hdr_ts = np.array([p[0] for p in parsed_hdr])
            first = parse_image(rows[0][1])
            info = {
                "present": True, "type": "sensor_msgs/msg/Image",
                "frame_id": first["frame_id"],
                "resolution": f'{first["width"]}x{first["height"]}',
                "encoding": first["encoding"],
                "step": first["step"],
                "is_bigendian": first["is_bigendian"],
                "bag_timestamps": ts_stats(bag_ts),
                "header_timestamps": ts_stats(hdr_ts),
                "header_matches_bag_ts": bool(np.array_equal(bag_ts, hdr_ts)),
            }
            if "Depth" in name and first["encoding"] in ("16UC1", "mono16"):
                idx = np.linspace(0, len(rows) - 1, min(depth_samples, len(rows))).astype(int)
                fracs, meds = [], []
                for i in idx:
                    img = parse_image(rows[i][1])
                    d = np.frombuffer(img["data"], dtype="<u2").reshape(img["height"], img["width"])
                    fracs.append(float(np.mean(d > 0)))
                    nz = d[d > 0]
                    if len(nz):
                        meds.append(float(np.median(nz)))
                fracs = np.array(fracs)
                info["depth_validity"] = {
                    "sampled_frames": int(len(fracs)),
                    "valid_fraction_mean": round(float(np.mean(fracs)), 4),
                    "valid_fraction_min": round(float(np.min(fracs)), 4),
                    "valid_fraction_max": round(float(np.max(fracs)), 4),
                }
                if meds:
                    du = meta.get("/device_0/sensor_0/option/Depth_Units/value", "0.001")
                    try:
                        du = float(du)
                    except ValueError:
                        du = 0.001
                    info["depth_validity"]["median_depth_m_over_samples"] = round(float(np.median(meds)) * du, 3)
        streams[name] = info
    rep["streams"] = streams

    # ---- cross-topic alignment ----
    align = {}
    if all(streams[t].get("present") for t in (COLOR, DEPTH)):
        r = streams[COLOR]["header_timestamps"]
        d = streams[DEPTH]["header_timestamps"]
        tid_c, tid_d = topics[COLOR][0], topics[DEPTH][0]
        rs = np.array([parse_header(b)[0] for _, b in db.execute(
            "select timestamp,data from messages where topic_id=? order by timestamp", (tid_c,))])
        ds = np.array([parse_header(b)[0] for _, b in db.execute(
            "select timestamp,data from messages where topic_id=? order by timestamp", (tid_d,))])
        j = np.clip(np.searchsorted(ds, rs), 0, len(ds) - 1)
        offs = np.minimum(np.abs(rs - ds[j]), np.abs(rs - ds[np.clip(j - 1, 0, len(ds) - 1)])) / 1e9
        align["rgb_vs_depth"] = {
            "rgb_n": r["count"], "depth_n": d["count"],
            "rgb_median_rate_hz": r["median_rate_hz"], "depth_median_rate_hz": d["median_rate_hz"],
            "nn_pair_offset_s": {"mean": round(float(np.mean(offs)), 5),
                                 "median": round(float(np.median(offs)), 5),
                                 "max": round(float(np.max(offs)), 5)},
        }
    if all(streams[t].get("present") for t in (GYRO, ACCEL)):
        g = streams[GYRO]["header_timestamps"]
        a = streams[ACCEL]["header_timestamps"]
        align["gyro_vs_accel"] = {
            "gyro_median_rate_hz": g["median_rate_hz"], "accel_median_rate_hz": a["median_rate_hz"],
            "gyro_span_s": g["span_s"], "accel_span_s": a["span_s"],
        }
    if all(streams[t].get("present") for t in (COLOR, GYRO)):
        # recompute from raw first/last stamps
        def first_last(tid, parser):
            f = db.execute("select data from messages where topic_id=? order by timestamp limit 1",
                           (tid,)).fetchone()[0]
            l = db.execute("select data from messages where topic_id=? order by timestamp desc limit 1",
                           (tid,)).fetchone()[0]
            return parser(f)[0], parser(l)[0]
        rf, rl = first_last(topics[COLOR][0], parse_header)
        gf, gl = first_last(topics[GYRO][0], parse_header)
        df, dl = first_last(topics[DEPTH][0], parse_header)
        af, al = first_last(topics[ACCEL][0], parse_header)
        align["imu_vs_images"] = {
            "first_stamp_ms": {"rgb": rf / 1e6, "depth": df / 1e6, "gyro": gf / 1e6, "accel": af / 1e6},
            "last_stamp_ms": {"rgb": rl / 1e6, "depth": dl / 1e6, "gyro": gl / 1e6, "accel": al / 1e6},
            "stream_start_spread_ms": round((max(rf, df, gf, af) - min(rf, df, gf, af)) / 1e6, 2),
            "stream_end_spread_ms": round((max(rl, dl, gl, al) - min(rl, dl, gl, al)) / 1e6, 2),
        }
    rep["cross_topic_alignment"] = align
    db.close()
    return rep


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--out", default=None)
    ap.add_argument("--depth-samples", type=int, default=20)
    args = ap.parse_args()
    rep = explore(args.bag, depth_samples=args.depth_samples)
    js = json.dumps(rep, indent=1)
    if args.out:
        with open(args.out, "w") as f:
            f.write(js)
        print(f"wrote {args.out}")
    else:
        print(js)


if __name__ == "__main__":
    main()
