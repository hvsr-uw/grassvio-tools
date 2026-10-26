#!/usr/bin/env python3
"""Generate small synthetic GrassVIO sequences for validator testing.

Creates a test dataset root with four cell-A sequences in the Section 2
layout, each exercising a different validator outcome:

  2026-09-27_A_still_still_01  pass      clean static take
  2026-09-27_A_still_still_02  drop      rgb frames 10-14 deleted
  2026-09-27_A_still_still_03  badmeta   invalid enum + missing required field
  2026-09-27_A_still_still_04  baddvi    30% depth validity + gyro bias on cell-A

Usage:
    python make_test_sequence.py --root test_data/grassvio-test [--frames 30] [--seed 7]
"""

import argparse
import csv
import os
import shutil

import cv2
import numpy as np
import yaml

FPS = 30.0
IMU_HZ = 200.0
W, H = 320, 240
DEPTH_MM = 1200

META_TEMPLATE = """sequence_id: {seq_id}
date: 2026-09-27
location: "synthetic test fixture"
operator: "validator self-test"

camera_motion: {camera_motion}
vegetation_motion: still
matrix_cell: A

wind_source: none
wind_instrument: "synthetic"
wind_mean_ms: 0.5
wind_max_ms: 0.8
wind_beaufort: 1
wind_direction_deg: 0
wind_direction_rel_camera: crosswind
wind_height_m: 0.9
wind_fan: null

illumination: stable

mount: tripod
camera_height_m: 0.9
view: oblique
approx_scene: "synthetic static texture"

duration_s: {duration_s}
num_rgb_frames: {n_frames}
num_imu_samples: {n_imu}
calibration_session: 2026-09-27

reference_type: tripod-zero-motion
reference_notes: "synthetic; camera static"
"""


def write_meta(seq_dir, seq_id, n_frames, n_imu, camera_motion="still",
               drop_operator=False):
    text = META_TEMPLATE.format(
        seq_id=seq_id, camera_motion=camera_motion,
        duration_s=n_frames / FPS, n_frames=n_frames, n_imu=n_imu)
    if drop_operator:
        text = "\n".join(l for l in text.splitlines()
                         if not l.startswith("operator:")) + "\n"
    with open(os.path.join(seq_dir, "meta.yaml"), "w") as f:
        f.write(text)


def make_texture(rng):
    base = rng.integers(0, 256, (H, W), dtype=np.uint8)
    base = cv2.GaussianBlur(base, (0, 0), 2.0)
    # tint it green-ish so it reads as vegetation-ish
    rgb = np.stack([ (base * 0.55).astype(np.uint8),
                     (base * 0.85).astype(np.uint8),
                     (base * 0.45).astype(np.uint8) ], axis=-1)
    return rgb


def write_sequence(root, seq_id, mode, n_frames=30, seed=7):
    rng = np.random.default_rng(seed)
    seq_dir = os.path.join(root, "sequences", seq_id)
    if os.path.isdir(seq_dir):
        shutil.rmtree(seq_dir)
    rgb_dir = os.path.join(seq_dir, "rgb")
    depth_dir = os.path.join(seq_dir, "depth")
    os.makedirs(rgb_dir)
    os.makedirs(depth_dir)

    n_imu = int(round(n_frames / FPS * IMU_HZ))
    texture = make_texture(rng)

    # --- rgb: static texture + tiny sensor noise -------------------------
    for i in range(n_frames):
        if mode == "drop" and 10 <= i <= 14:
            continue  # simulate dropped frames
        noise = rng.normal(0, 1.0, (H, W, 3))
        frame = np.clip(texture.astype(float) + noise, 0, 255).astype(np.uint8)
        cv2.imwrite(os.path.join(rgb_dir, "frame_%06d.png" % i), frame)

    # --- depth: constant plane with invalid-pixel mask --------------------
    invalid_frac = 0.70 if mode == "baddvi" else 0.05
    for i in range(n_frames):
        d = np.full((H, W), DEPTH_MM, dtype=np.uint16)
        mask = rng.random((H, W)) < invalid_frac
        d[mask] = 0
        cv2.imwrite(os.path.join(depth_dir, "frame_%06d.png" % i), d)

    # --- timestamps.csv (keeps original indices even for dropped frames) --
    with open(os.path.join(seq_dir, "timestamps.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame_index", "timestamp_s"])
        for i in range(n_frames):
            w.writerow([i, "%.6f" % (i / FPS)])

    # --- imu.csv -----------------------------------------------------------
    t = np.arange(n_imu) / IMU_HZ
    if mode == "baddvi":
        gyro = rng.normal(0, 0.005, (n_imu, 3)) + np.array([0.30, 0.15, -0.20])
    else:
        gyro = rng.normal(0, 0.005, (n_imu, 3))
    accel = rng.normal(0, 0.02, (n_imu, 3)) + np.array([0.0, 0.0, 9.81])
    with open(os.path.join(seq_dir, "imu.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["timestamp", "gyro_x", "gyro_y", "gyro_z",
                    "accel_x", "accel_y", "accel_z"])
        for j in range(n_imu):
            w.writerow(["%.6f" % t[j],
                        "%.6f" % gyro[j, 0], "%.6f" % gyro[j, 1],
                        "%.6f" % gyro[j, 2],
                        "%.6f" % accel[j, 0], "%.6f" % accel[j, 1],
                        "%.6f" % accel[j, 2]])

    # --- meta.yaml ---------------------------------------------------------
    if mode == "badmeta":
        write_meta(seq_dir, seq_id, n_frames, n_imu,
                   camera_motion="hovering", drop_operator=True)
    else:
        write_meta(seq_dir, seq_id, n_frames, n_imu)

    with open(os.path.join(seq_dir, "notes.txt"), "w") as f:
        f.write("synthetic fixture, mode=%s\n" % mode)

    # sanity: the good modes must be yaml-parseable
    with open(os.path.join(seq_dir, "meta.yaml")) as f:
        yaml.safe_load(f)
    return seq_dir


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="test_data/grassvio-test")
    ap.add_argument("--frames", type=int, default=30)
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()

    root = os.path.abspath(args.root)
    os.makedirs(os.path.join(root, "sequences"), exist_ok=True)
    cases = [
        ("2026-09-27_A_still_still_01", "pass"),
        ("2026-09-27_A_still_still_02", "drop"),
        ("2026-09-27_A_still_still_03", "badmeta"),
        ("2026-09-27_A_still_still_04", "baddvi"),
    ]
    for seq_id, mode in cases:
        d = write_sequence(root, seq_id, mode, n_frames=args.frames,
                           seed=args.seed)
        print("wrote %-32s mode=%s" % (d, mode))
    print("dataset root: %s" % root)


if __name__ == "__main__":
    main()
