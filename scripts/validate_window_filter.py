#!/usr/bin/env python3
"""Reproducible synthetic camera/grass 2x2; NOT measured field performance."""
import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'stitch-map'))
from geometry import apply_transform, fit_homography
from stitch_map import stitch
from window_motion import track_window, temporal_filter


def make_sequence(camera_moves, grass_moves):
    rng = np.random.default_rng(566)
    base = cv2.GaussianBlur(rng.integers(0, 256, (240, 320, 3), dtype=np.uint8), (3, 3), .5)
    # A large independent textured patch; about 55% of the image moves.
    grass = cv2.GaussianBlur(rng.integers(0, 256, base.shape, dtype=np.uint8), (3, 3), .5)
    frames, models = [], []
    for k in range(26):
        scene = base.copy()
        dx = 10*np.sin(np.pi*k/50) if grass_moves else 0.
        moving = cv2.warpAffine(grass, np.float32([[1, 0, dx], [0, 1, 0]]), (320, 240))
        scene[30:210, 40:280] = moving[30:210, 40:280]
        H = np.array([[1., 0, .18*k if camera_moves else 0.],
                      [0, 1, -.08*k if camera_moves else 0.], [0, 0, 1.]])
        frames.append(cv2.warpPerspective(scene, H, (320, 240)))
        models.append(H)
    return frames, np.array(models)


def run_case(camera_moves, grass_moves, outdir):
    frames, models = make_sequence(camera_moves, grass_moves)
    gray = [cv2.cvtColor(f, cv2.COLOR_BGR2GRAY) for f in frames]
    history, detected = track_window(gray)
    keep, scores = temporal_filter(history, models)
    baseline, _ = fit_homography(history[-1], history[0], min_inliers=20)
    filtered, _ = fit_homography(history[-1, keep], history[0, keep], min_inliers=20)
    x, y = history[0].T
    # Ignore the patch seam and nearby points (mixed tracking windows).
    dynamic = (x > 55) & (x < 265) & (y > 45) & (y < 195)
    static = (x < 25) | (x > 295) | (y < 15) | (y > 225)
    grid = np.array([[x, y] for x in np.linspace(40, 280, 7) for y in np.linspace(30, 210, 5)])
    truth = np.linalg.inv(models[-1])
    def error(H):
        return float(np.mean(np.linalg.norm(apply_transform(H, grid)-apply_transform(truth, grid), axis=1))) if H is not None else None
    label = f"camera_{'moving' if camera_moves else 'still'}_grass_{'moving' if grass_moves else 'static'}"
    prior = {'translation_compensated': True, 'image_transforms': models.tolist()} if camera_moves else None
    mosaic, stats = stitch(frames, 5, 'sift', motion_filter=True,
                           camera_still=not camera_moves, camera_motion=prior,
                           blend='best', debug_out=str(outdir/f'{label}_debug.png'))
    cv2.imwrite(str(outdir/f'{label}_map.png'), mosaic)
    if camera_moves and grass_moves:
        debug = cv2.imread(str(outdir/f'{label}_debug.png'))
        cv2.imwrite(str(outdir/'synthetic_debug_preview.jpg'), debug,
                    [cv2.IMWRITE_JPEG_QUALITY, 85])
    return {"case": label, "synthetic_only": True, "detected": detected,
            "complete_tracks": len(keep), "kept": int(keep.sum()),
            "static_control_tracks": int(static.sum()),
            "static_kept_fraction": float(keep[static].mean()) if static.any() else None,
            "grass_interior_tracks": int(dynamic.sum()),
            "grass_rejected_fraction": float((~keep[dynamic]).mean()) if dynamic.any() else None,
            "unfiltered_homography_error_px": error(baseline),
            "filtered_homography_error_px": error(filtered),
            "windows": stats['windows'], "frames_used": stats['frames_used']}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--outdir', type=Path, required=True)
    args = ap.parse_args()
    args.outdir.mkdir(parents=True, exist_ok=True)
    cv2.setNumThreads(1)
    cv2.setRNGSeed(0)
    results = [run_case(camera, grass, args.outdir) for camera in (False, True) for grass in (False, True)]
    report = {'opencv_version': cv2.__version__, 'numpy_version': np.__version__,
              'seed': 566, 'window_frames': 25, 'residual_threshold_px': 1.5,
              'camera_prior': 'known synthetic full camera motion, including translation',
              'real_recordings_tested': False, 'cases': results}
    (args.outdir/'synthetic_results.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
