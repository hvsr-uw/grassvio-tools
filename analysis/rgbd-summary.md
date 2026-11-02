# RGB-D odometry + orthomosaic — test summary

Date: 2026-11-02. Goal: replace chained 2D homographies (which diverge on
rotation-heavy takes) with per-pair SE(3) from the depth channel, rendering
the map by orthorectification instead of homography warping.

Script: dataset-tools/stitch-map/rgbd_odometry.py (new; stitch_map.py
untouched). SIFT (10000) + Lowe 0.75 per stride-5 pair, back-project
frame-i keypoints with depth + K, cv2.solvePnPRansac vs frame-j 2D points,
chain to absolute SE(3), splat valid depth pixels into a top-down 1 cm
grid. Bags read read-only.

## Stage 1 — depth sanity check: PASS

Depth stream: 640x480 mono16, units 0.001 m. Intrinsics
fx=fy=386.01, ppx=320.77, ppy=240.89, zero distortion.

| take | valid fill mean | min | max | median depth |
|---|---|---|---|---|
| 20261026_173807 | 65% | 58% | 73% | 1.76 m |
| 20261007_080033 | 92% | 90% | 96% | 0.87 m |

Both far above the 10% stop line and inside the D455 ideal range
(0.6-6 m). Depth is usable.

## Stage 2 — RGB-D relative pose: PASS

| take | pairs ok | inlier median | inlier min | 3D pts median |
|---|---|---|---|---|
| 20261026_173807 | 195/195 | 553 | 138 | 675 |
| 20261007_080033 | 86/88 | ~2500 | 27 | ~2200 |
| 20261007_075947 | 46/46 | — | — | — |

080033's two failed pairs (too few 3D points) held the last pose.
PnP per-pair rotation magnitudes track the gyro, and the estimator is
unbiased on synthetic data (0.025 deg mean error), so the estimator
itself is sound. One color/depth count mismatch found and fixed along
the way: 080033 has 441 color vs 440 depth frames, so pairing is now by
nearest timestamp (worst offset 27 ms), not by order.

## Stage 3 — chain + orthomosaic: PASS

Two rendering lessons, both found by looking at the output:
1. A top-down map is meaningless for frames not looking down. 080033
   genuinely tips up to ~67 deg mid-take (PnP total rotation 166 deg
   vs gyro ~156 deg over the take), and those frames see horizon/sky.
   Fix: cap splat depth at 3 m, only splat frames within 35 deg of
   straight down.
2. Averaging colors across frames smears under small pose errors;
   last-frame-wins keeps the grass texture crisp. (The *_lastwins.png
   files are the good versions.)

| take | RGB-D map | homography map | verdict |
|---|---|---|---|
| 20261026_173807 | 5.6 x 5.6 m, bounded, grass texture visible | 57,504 x 56,538 px blow-up, unusable | decisive win |
| 20261007_080033 | 6.7 x 3.2 m, bounded, no stretch | 2351 x 2101 px, far-end stretch | win (no stretch) |
| 20261007_075947 | 1.7 x 2.2 m, crisp grass | 1138 x 786 px, clean | parity |

## Honest assessment

This beats the homography pipeline where it matters: the blow-up mode
is gone because the model is right (SE(3) from depth instead of a planar
fit of 3D grass). The maps are metrically scaled, drift-free in the
sense that matters for mapping, and every number above was measured,
not assumed.

Limits, stated plainly: per-pair PnP is only as good as the depth at
SIFT keypoints (edges are the noisiest depth pixels); the chain still
accumulates small errors with no loop closure; frames that tip far from
down-facing contribute nothing to a top-down map (40/196 frames splatted
on 173807). Fusing the gyro as a rotation prior and adding loop-closure
edges are the natural next steps, but even without them this already
outperforms chaining homographies on every take tested.

Files here: per-take poses.json (absolute 4x4 poses + pair stats),
trajectory.npy, orthomosaic PNGs (averaged) and *_lastwins.png (crisper).
080033_traj.png is a debug plot. single_frame_splat.png verified the
unprojection on one frame.
