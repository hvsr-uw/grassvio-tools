#!/usr/bin/env python3
"""Build a four-panel video of the mapping running live.

Panels, all advancing one chosen frame at a time:
  1. camera      - the current frame, as the camera sees it
  2. IMU odometry - path from double-integrated accelerometer
                    (demeaned by the take mean, same as imu_fusion.py
                    step 3). This drifts: integrating noisy accel twice
                    accumulates error, so the path wanders off on
                    longer takes. That drift is real and worth seeing.
  3. visual odometry - frame centres chained through the neighbour
                    homographies (same math as trajectory_overlay.py)
  4. panorama    - the mosaic canvas filling in frame by frame

Usage:
    python3 four_panel_video.py --bag <take>.db3 --out demo.mp4

Options mirror stitch_map.py: --stride N (default 5), --fps (default 10).
Writes demo.mp4 plus a .json with the run stats.
"""
import argparse
import json
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import stitch_map  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "bag-exploration"))
from explore_bag import COLOR, ACCEL, parse_image, parse_imu  # noqa: E402
import sqlite3  # noqa: E402


def load_images(bag_path, topic):
    """(images, stamps_ns): RGB as HxWx3 uint8. Read-only on the bag."""
    db = sqlite3.connect(f"file:{bag_path}?mode=ro", uri=True)
    db.execute("PRAGMA query_only=ON")
    tid = db.execute("select id from topics where name=?",
                     (topic,)).fetchone()[0]
    rows = db.execute(
        "select data from messages where topic_id=? order by timestamp",
        (tid,)).fetchall()
    db.close()
    imgs, stamps = [], []
    for (blob,) in rows:
        p = parse_image(blob)
        rgb = np.frombuffer(p["data"], dtype=np.uint8).reshape(
            p["height"], p["width"], 3)
        imgs.append(rgb)
        stamps.append(p["stamp_ns"])
    return imgs, np.array(stamps, dtype=np.float64)


def load_imu(bag_path, topic):
    """(stamps_ns, vecs): gyro or accel vectors. Read-only on the bag."""
    db = sqlite3.connect(f"file:{bag_path}?mode=ro", uri=True)
    db.execute("PRAGMA query_only=ON")
    tid = db.execute("select id from topics where name=?",
                     (topic,)).fetchone()[0]
    rows = db.execute(
        "select data from messages where topic_id=? order by timestamp",
        (tid,)).fetchall()
    db.close()
    stamps, vecs = [], []
    for (blob,) in rows:
        p = parse_imu(blob)
        stamps.append(p["stamp_ns"])
        vecs.append(p["angular_velocity"] if "Gyro" in topic
                    else p["linear_acceleration"])
    return np.array(stamps, dtype=np.float64), np.array(vecs, dtype=np.float64)

PANEL_W, PANEL_H = 640, 480
BG = (18, 18, 18)


def label_panel(img, text):
    cv2.rectangle(img, (0, 0), (PANEL_W, 34), BG, -1)
    cv2.putText(img, text, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (255, 255, 255), 2, cv2.LINE_AA)
    return img


def chain_transforms(frames, stride):
    """Neighbour homography chain, identical to stitch_map's chain mode."""
    detector = stitch_map.make_detector("sift")
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    small = [stitch_map.downscale(f) for f in frames]
    grays = [cv2.cvtColor(f, cv2.COLOR_BGR2GRAY) for f in small]
    chosen = list(range(0, len(frames), stride))
    feats = {i: detector.detectAndCompute(grays[i], None) for i in chosen}
    transforms = [np.eye(3)]
    skipped = 0
    inliers = []
    for pos in range(1, len(chosen)):
        a, b = chosen[pos - 1], chosen[pos]
        H, n = stitch_map.pair_homography(feats[a], feats[b], matcher)
        if H is None:
            skipped += 1
            transforms.append(transforms[-1].copy())
        else:
            inliers.append(n)
            transforms.append(transforms[-1] @ H)
    med = float(np.median(inliers)) if inliers else 0.0
    return chosen, small, transforms, skipped, med


def imu_path(accel_stamps, accel_vecs, frame_stamps):
    """2D path from double-integrated demeaned accel, sampled at frames.

    Continuous integration from zero velocity at the take start (odometry),
    not the per-interval zero-velocity proxy. Demeaning by the take mean
    removes gravity only when the camera does not rotate; under rotation
    the leftover leaks in and the path drifts. That is the point of this
    panel: it shows what raw IMU odometry looks like.
    """
    mean_vec = accel_vecs.mean(axis=0)
    v = np.zeros(3)
    p = np.zeros(3)
    pos = [p.copy()]
    for k in range(len(accel_stamps) - 1):
        dt = (accel_stamps[k + 1] - accel_stamps[k]) / 1e9
        if dt <= 0 or dt > 0.5:
            continue
        v = v + (accel_vecs[k] - mean_vec) * dt
        p = p + v * dt
        pos.append(p.copy())
    pos = np.array(pos)
    ts = accel_stamps
    # Nearest IMU sample at or before each frame stamp.
    idx = np.searchsorted(ts, frame_stamps, side="right") - 1
    idx = np.clip(idx, 0, len(pos) - 1)
    return pos[idx][:, :2]  # x-y plane


def fit_view(pts, w, h, margin=0.1):
    """Map 2D points into a w x h panel, y flipped, with a margin."""
    lo = pts.min(axis=0)
    hi = pts.max(axis=0)
    span = np.maximum(hi - lo, 1e-6)
    span = span * (1 + 2 * margin)
    lo = lo - (span - (hi - lo)) / 2
    s = min(w / span[0], h / span[1])
    mapped = (pts - lo) * s
    mapped[:, 1] = h - mapped[:, 1]
    return mapped, s


def draw_path_panel(full_pts, upto, title, colour):
    img = np.full((PANEL_H, PANEL_W, 3), 255, dtype=np.uint8)
    mapped, _ = fit_view(full_pts, PANEL_W - 40, PANEL_H - 80)
    mapped = mapped + np.array([20, 40])
    ipts = np.round(mapped).astype(int)
    if upto >= 1:
        cv2.polylines(img, [ipts[:upto + 1].reshape(-1, 1, 2)], False,
                      (60, 60, 60), 4, cv2.LINE_AA)
        cv2.polylines(img, [ipts[:upto + 1].reshape(-1, 1, 2)], False,
                      colour, 2, cv2.LINE_AA)
    sx, sy = ipts[0]
    cv2.circle(img, (sx, sy), 7, (0, 180, 0), -1, cv2.LINE_AA)
    if upto >= 1:
        cx, cy = ipts[upto]
        cv2.circle(img, (cx, cy), 7, (0, 0, 220), -1, cv2.LINE_AA)
    return label_panel(img, title)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--stride", type=int, default=5)
    ap.add_argument("--fps", type=int, default=10)
    args = ap.parse_args()

    print("loading frames...", flush=True)
    frames_rgb, frame_stamps = load_images(args.bag, COLOR)
    frames = [cv2.cvtColor(f, cv2.COLOR_RGB2BGR) for f in frames_rgb]
    print("loading IMU...", flush=True)
    accel_stamps, accel_vecs = load_imu(args.bag, ACCEL)
    if len(frames) < 2:
        print("not enough frames", file=sys.stderr)
        return 1

    print("chaining homographies...", flush=True)
    chosen, small, transforms, skipped, med_inliers = chain_transforms(
        frames, args.stride)

    # Final panorama geometry, fixed for the whole video.
    h, w = small[0].shape[:2]
    corners = np.float32([[0, 0], [w, 0], [w, h], [0, h]]).reshape(-1, 1, 2)
    all_pts = np.concatenate(
        [cv2.perspectiveTransform(corners, T) for T in transforms], axis=0)
    x_min, y_min = np.floor(all_pts.min(axis=0)[0]).astype(int)
    x_max, y_max = np.ceil(all_pts.max(axis=0)[0]).astype(int)
    shift = np.array([[1, 0, -x_min], [0, 1, -y_min], [0, 0, 1]],
                     dtype=float)
    canvas_w, canvas_h = x_max - x_min, y_max - y_min
    if max(canvas_w, canvas_h) > 8000:
        print(f"chained canvas {canvas_w}x{canvas_h}: diverged, no video",
              file=sys.stderr)
        return 1

    print("IMU odometry...", flush=True)
    imu_pts = imu_path(accel_stamps, accel_vecs,
                       frame_stamps[np.array(chosen)])
    vo_pts = []
    for idx, T in zip(chosen, transforms):
        fh, fw = small[idx].shape[:2]
        c = np.float32([[[fw / 2.0, fh / 2.0]]])
        vo_pts.append(cv2.perspectiveTransform(c, shift @ T)[0][0])
    vo_pts = np.array(vo_pts)

    # Progressive panorama: accumulate one frame per video frame.
    acc = np.zeros((canvas_h, canvas_w, 3), dtype=np.float64)
    cover = np.zeros((canvas_h, canvas_w), dtype=np.float64)
    ones = np.ones((h, w), dtype=np.float64)

    vw, vh = PANEL_W * 2, PANEL_H * 2
    writer = cv2.VideoWriter(args.out,
                             cv2.VideoWriter_fourcc(*"mp4v"),
                             args.fps, (vw, vh))
    n = len(chosen)
    for k in range(n):
        idx = chosen[k]
        M = shift @ transforms[k]
        warped = cv2.warpPerspective(small[idx], M, (canvas_w, canvas_h))
        mask = cv2.warpPerspective(ones, M, (canvas_w, canvas_h))
        acc += warped.astype(np.float64) * mask[..., None]
        cover += mask

        # Panel 1: camera.
        p1 = label_panel(cv2.resize(frames[idx], (PANEL_W, PANEL_H)), "camera")
        # Panels 2-3: odometry paths.
        p2 = draw_path_panel(imu_pts, k, "IMU odometry", (220, 120, 0))
        p3 = draw_path_panel(vo_pts, k, "visual odometry", (255, 0, 255))
        # Panel 4: panorama so far.
        pano = (acc / np.maximum(cover, 1)[..., None]).astype(np.uint8)
        s = min(PANEL_W / canvas_w, PANEL_H / canvas_h)
        rw, rh = max(1, int(canvas_w * s)), max(1, int(canvas_h * s))
        p4 = np.full((PANEL_H, PANEL_W, 3), BG, dtype=np.uint8)
        small_pano = cv2.resize(pano, (rw, rh))
        y0, x0 = (PANEL_H - rh) // 2, (PANEL_W - rw) // 2
        p4[y0:y0 + rh, x0:x0 + rw] = small_pano
        p4 = label_panel(p4, "panorama")
        grid = np.zeros((vh, vw, 3), dtype=np.uint8)
        grid[0:PANEL_H, 0:PANEL_W] = p1
        grid[0:PANEL_H, PANEL_W:vw] = p2
        grid[PANEL_H:vh, 0:PANEL_W] = p3
        grid[PANEL_H:vh, PANEL_W:vw] = p4
        writer.write(grid)
        if (k + 1) % 20 == 0:
            print(f"  frame {k + 1}/{n}", flush=True)
    writer.release()

    stats = {
        "frames_total": len(frames),
        "frames_used": n,
        "pairs_skipped": skipped,
        "median_inliers": med_inliers,
        "canvas_width_px": int(canvas_w),
        "canvas_height_px": int(canvas_h),
        "stride": args.stride,
        "fps": args.fps,
        "video": os.path.basename(args.out),
    }
    with open(os.path.splitext(args.out)[0] + ".json", "w") as f:
        json.dump(stats, f, indent=2)
    print(f"wrote {args.out}")
    print(json.dumps(stats, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
