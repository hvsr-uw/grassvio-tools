#!/usr/bin/env python3
"""Chain-stitch the 8 new Nov 6 takes, same method as batch_stitch.py.

Same matching and chaining as stitch_map.py (chain mode), same canvas
guard (8000 px): takes whose chained canvas exceeds the guard are
recorded as blow-ups WITHOUT warping. Writes per take: <stamp>.png +
<stamp>.json when stitched, and one results.jsonl with a record for
every take. Bag reads are read-only. A per-take failure is recorded,
never fatal.

Usage: python3 stitch_new_takes.py --outdir <dir>
"""
import argparse
import gc
import json
import os
import sys
import time
import traceback

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "..", "dataset-tools", "stitch-map"))
import stitch_map  # noqa: E402
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "..", "analysis-drafts", "stitch-all"))
from batch_stitch import chain_transforms, canvas_bounds, warp_mosaic  # noqa: E402

NEW_DIR = os.path.expanduser("~/workspace/browser_downloads/sess-2653029502")
CANVAS_GUARD_PX = 8000
STRIDE = 5
DETECTOR = "sift"

BAGS = {
    "20261106_084542": "browser-download-20261106T164822.122638062Z-0-20261106_084542.db3",
    "20261106_084741": "browser-download-20261106T165215.795397528Z-1-20261106_084741.db3",
    "20261106_084852": "browser-download-20261106T165443.220684395Z-2-20261106_084852.db3",
    "20261106_085224": "browser-download-20261106T165757.605027021Z-3-20261106_085224.db3",
    "20261106_085301": "browser-download-20261106T170119.774966625Z-4-20261106_085301.db3",
    "20261106_085422": "browser-download-20261106T170419.658440459Z-5-20261106_085422.db3",
    "20261106_085453": "browser-download-20261106T170707.176918096Z-6-20261106_085453.db3",
    "20261106_085611": "browser-download-20261106T171330.872403110Z-8-20261106_085611.db3",
}


def run_take(stamp, bag_path, outdir):
    t0 = time.time()
    frames = stitch_map.load_frames_from_bag(bag_path)
    if len(frames) < 2:
        return {"take": stamp, "status": "failed",
                "reason": "fewer than 2 frames"}
    chosen, frames, transforms, skipped, median_inliers = chain_transforms(
        frames, STRIDE, DETECTOR)
    canvas_w, canvas_h = canvas_bounds(frames, transforms)
    rec = {
        "take": stamp,
        "bag": os.path.basename(bag_path),
        "frames_total": len(frames),
        "frames_used": len(chosen),
        "pairs_skipped": skipped,
        "median_inliers": median_inliers,
        "canvas_width_px": int(canvas_w),
        "canvas_height_px": int(canvas_h),
        "stride": STRIDE,
        "detector": DETECTOR,
        "elapsed_s": round(time.time() - t0, 1),
    }
    if max(canvas_w, canvas_h) > CANVAS_GUARD_PX:
        rec["status"] = "blow_up"
        rec["reason"] = (f"chained canvas {canvas_w}x{canvas_h}px exceeds "
                         f"guard {CANVAS_GUARD_PX}px; warp skipped")
        return rec
    h, w = frames[0].shape[:2]
    corners = np.float32([[0, 0], [w, 0], [w, h], [0, h]]).reshape(-1, 1, 2)
    all_pts = np.concatenate(
        [cv2.perspectiveTransform(corners, T) for T in transforms], axis=0)
    x_min, y_min = np.floor(all_pts.min(axis=0)[0]).astype(int)
    mosaic = warp_mosaic(frames, chosen, transforms, canvas_w, canvas_h,
                         x_min, y_min)
    out_png = os.path.join(outdir, f"{stamp}.png")
    cv2.imwrite(out_png, mosaic)
    rec["status"] = "stitched"
    rec["png"] = os.path.basename(out_png)
    with open(os.path.join(outdir, f"{stamp}.json"), "w") as f:
        json.dump(rec, f, indent=2)
    return rec


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--outdir", required=True)
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)
    log_path = os.path.join(args.outdir, "run.log")
    res_path = os.path.join(args.outdir, "results.jsonl")

    with open(log_path, "a") as log, open(res_path, "a") as res:
        for i, stamp in enumerate(sorted(BAGS)):
            bag = os.path.join(NEW_DIR, BAGS[stamp])
            t0 = time.time()
            try:
                rec = run_take(stamp, bag, args.outdir)
            except Exception as e:  # noqa: BLE001 - one bad take must not stop the batch
                rec = {"take": stamp, "status": "failed",
                       "reason": f"{type(e).__name__}: {e}",
                       "traceback": traceback.format_exc()[-2000:]}
            rec["batch_index"] = i
            res.write(json.dumps(rec) + "\n")
            res.flush()
            line = (f"[{i+1}/{len(BAGS)}] {stamp}: {rec['status']} "
                    f"({time.time()-t0:.0f}s)")
            print(line, flush=True)
            log.write(line + "\n")
            log.flush()
            gc.collect()
    print("done", flush=True)


if __name__ == "__main__":
    main()
