#!/usr/bin/env python3
"""Extract what the IMU-vs-RGB-D rest-start check needs, into small files.

For each recording this writes one .npz with
  * raw gyro and accelerometer samples with header timestamps,
  * colour and depth frame timestamps, calibration and IMU extrinsics,
  * dense RGB-D odometry: SIFT matches between colour frames i and i+step,
    3D points from the registered depth of frame i, and a PnP pose of
    frame i+step, plus the median image motion of the matches.

The recordings are opened read-only. Nothing is uploaded.
"""
import argparse
import ctypes
import json
import shutil
import sqlite3
import sys
import time
import traceback
from pathlib import Path

_connect = sqlite3.connect


def _immutable_connect(database, *args, **kwargs):
    # Recordings never change while we read them; immutable=1 skips file
    # locking, which avoids spurious disk I/O errors on synced folders.
    if isinstance(database, str) and database.startswith("file:") and "immutable=" not in database:
        database += "&immutable=1" if "?" in database else "?immutable=1"
    return _connect(database, *args, **kwargs)


sqlite3.connect = _immutable_connect


def keep_awake():
    try:  # ES_CONTINUOUS | ES_SYSTEM_REQUIRED: no sleep while extracting
        ctypes.windll.kernel32.SetThreadExecutionState(0x80000000 | 0x00000001)
        return True
    except Exception:
        return False

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[2] / "stitch-map"))
from recording_camera import ACCEL, COLOR, GYRO, Recording, rays  # noqa: E402


def features(sift, image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    keypoints, descriptors = sift.detectAndCompute(gray, None)
    xy = np.array([k.pt for k in keypoints], float).reshape(-1, 2)
    return xy, descriptors


def match(da, db, ratio=0.75):
    if da is None or db is None or len(da) < 2 or len(db) < 2:
        return np.empty((0, 2), int)
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    forward = matcher.knnMatch(da, db, k=2)
    backward = matcher.knnMatch(db, da, k=2)
    reverse = {p[0].queryIdx: p[0].trainIdx for p in backward
               if len(p) == 2 and p[0].distance < ratio * p[1].distance}
    pairs = [(p[0].queryIdx, p[0].trainIdx) for p in forward
             if len(p) == 2 and p[0].distance < ratio * p[1].distance
             and reverse.get(p[0].trainIdx) == p[0].queryIdx]
    return np.array(pairs, int).reshape(-1, 2)


def pair_pose(recording, cache, sift, i, j, focal):
    """Pose of frame j relative to frame i (x_j = R x_i + t), metres."""
    for k in (i, j):
        if k not in cache:
            cache[k] = features(sift, recording.image(COLOR, k))
    (xa, da), (xb, db) = cache[i], cache[j]
    m = match(da, db)
    out = dict(matches=len(m), inliers=0, ok=False,
               flow_px=float(np.median(np.linalg.norm(xb[m[:, 1]] - xa[m[:, 0]], axis=1)))
               if len(m) else np.nan)
    if len(m) < 12:
        return out, None
    try:
        cloud, offset = recording.cloud_for_color(i)
    except ValueError:
        return out, None
    px = np.rint(xa[m[:, 0]]).astype(int)
    h, w = cloud.shape[:2]
    px[:, 0] = np.clip(px[:, 0], 0, w - 1)
    px[:, 1] = np.clip(px[:, 1], 0, h - 1)
    xyz = cloud[px[:, 1], px[:, 0]]
    good = np.isfinite(xyz).all(axis=1)
    out["depth_points"] = int(good.sum())
    out["depth_offset_ms"] = float(offset)
    if good.sum() < 12:
        return out, None
    target = rays(xb[m[good, 1]], recording.color)[:, :2]
    ok, rv, tv, inl = cv2.solvePnPRansac(
        xyz[good].astype(np.float64), target.astype(np.float64), np.eye(3), None,
        iterationsCount=400, reprojectionError=2.0 / focal, confidence=0.999,
        flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or inl is None or len(inl) < 12:
        return out, None
    idx = inl.ravel()
    ok, rv, tv = cv2.solvePnP(xyz[good][idx], target[idx], np.eye(3), None,
                              rv, tv, useExtrinsicGuess=True,
                              flags=cv2.SOLVEPNP_ITERATIVE)
    out.update(inliers=int(len(idx)), ok=bool(ok))
    return out, np.concatenate([rv.ravel(), tv.ravel()])


def extract(bag, out_dir, step, max_seconds):
    started = time.time()
    recording = Recording(bag)
    try:
        gyro_t, gyro = recording.imu(GYRO)
        accel_t, accel = recording.imu(ACCEL)
        ct = recording.color_times
        focal = float((recording.color["fx"] + recording.color["fy"]) / 2)
        count = len(ct)
        if max_seconds > 0:
            count = int(np.searchsorted(ct, ct[0] + max_seconds * 1e9))
        sift = cv2.SIFT_create(nfeatures=2000)
        cache, rows, poses = {}, [], []
        for i in range(0, count - step, step):
            j = i + step
            info, pose = pair_pose(recording, cache, sift, i, j, focal)
            cache.pop(i - step, None)
            rows.append([i, j, info["matches"], info.get("depth_points", 0),
                         info["inliers"], info["flow_px"],
                         info.get("depth_offset_ms", np.nan)])
            poses.append(pose if pose is not None else np.full(6, np.nan))
        r_cg, t_cg = recording.color_from_gyro
        r_ca, t_ca = recording.color_from_accel
        name = Path(bag).stem
        np.savez_compressed(
            Path(out_dir) / f"{name}.npz",
            gyro_t=gyro_t, gyro=gyro, accel_t=accel_t, accel=accel,
            color_t=recording.color_times, depth_t=recording.depth_times,
            pair_rows=np.array(rows, float), pair_pose_rvec_tvec=np.array(poses, float),
            color_from_gyro_R=r_cg, color_from_gyro_t=t_cg,
            color_from_accel_R=r_ca, color_from_accel_t=t_ca,
            color_K=recording.color["K"], color_coeffs=np.array(recording.color["coeffs"]),
            step=step)
        meta = dict(recording=Path(bag).name, size_bytes=Path(bag).stat().st_size,
                    color_frames=int(len(ct)), pairs=len(rows),
                    pairs_with_pose=int(sum(np.isfinite(p).all() for p in poses)),
                    gyro_samples=int(len(gyro_t)), accel_samples=int(len(accel_t)),
                    color_model=recording.color["model"], step_frames=step,
                    seconds=round(time.time() - started, 1))
        (Path(out_dir) / f"{name}.json").write_text(json.dumps(meta, indent=2))
        return meta
    finally:
        recording.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--names", nargs="+", required=True)
    parser.add_argument("--step", type=int, default=3)
    parser.add_argument("--max-seconds", type=float, default=0.)
    parser.add_argument("--opencv-threads", type=int, default=1)
    args = parser.parse_args()
    cv2.setNumThreads(args.opencv_threads)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    awake = keep_awake()
    usage = shutil.disk_usage(args.output_dir)
    print(json.dumps(dict(sleep_blocked=awake, free_gb=round(usage.free / 1e9, 2),
                          sqlite=sqlite3.sqlite_version)), flush=True)
    failed = []
    for name in args.names:
        bag = args.input_dir / name
        if (args.output_dir / f"{Path(name).stem}.npz").exists():
            print(f"already extracted {name}", flush=True)
            continue
        print(f"extracting {name} ...", flush=True)
        try:
            meta = extract(bag, args.output_dir, args.step, args.max_seconds)
            print(json.dumps(meta), flush=True)
        except Exception as exc:  # report and continue with the others
            failed.append(dict(recording=name, error=repr(exc),
                               traceback=traceback.format_exc()))
            print(f"FAILED {name}: {exc!r}", flush=True)
            print(traceback.format_exc(), flush=True)
    (args.output_dir / "extract_status.json").write_text(
        json.dumps(dict(failed=failed), indent=2))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
