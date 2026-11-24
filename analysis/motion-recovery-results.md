# Motion recovery and channel checks

The later [multiview follow-up](multiview-results.md) adds a longer image-only
reconstruction and refines timing. The tables below retain the earlier run.

I ran the six November 19 recordings on November 20, 2026. They contain 4,080 colour frames. I recovered the complete sampled homography path in every video and added calibrated RGB-D tracking, temporal-filtered loop recovery and registered-depth maps. The IMU translation initialization still fails its checks, so I have not established agreement between all three motion channels.

## Tracking and maps

The earlier first-image projection discarded local motion links when a composed full-image warp crossed a projective horizon. I now keep finite, invertible local links separate from rendering and split failed temporal windows into shorter filtered windows. Rendering still rejects invalid footprints and excessive canvas sizes.

| Video | Colour frames | Connected homography nodes | Connected RGB-D nodes | RGB-D end connected |
|---|---:|---:|---:|---|
| 20261119_073137.db3 | 901 | 37/37 | 37/37 | yes |
| 20261119_073608.db3 | 818 | 34/34 | 34/34 | yes |
| 20261119_073711.db3 | 687 | 29/29 | 29/29 | yes |
| 20261119_074041.db3 | 818 | 34/34 | 34/34 | yes |
| 20261119_074125.db3 | 470 | 20/20 | 20/20 | yes |
| 20261119_074200.db3 | 386 | 19/19 | 14/19 | yes |

The new homography graph connects 173/173 sampled nodes through 167 motion links. The earlier filtered first-image projection placed 53/171 nodes. The totals differ because the new runner splits two failed windows in the fastest video. Graph connectivity alone does not remove projective drift. The bounded local homography maps are overlapping views, not one metric orthomosaic.

For RGB-D motion, I use the recorded intrinsics, depth units, distortion model and reference-to-stream extrinsics to register depth in the colour camera. I estimate camera motion from temporally filtered image tracks with measured depth. I also project the registered 3D samples into a ground-plane map at 200 pixels per metre. I retain holes and sparse areas instead of filling them with invented depth. These metric estimates depend on calibration, measured depth and the camera-motion model; they are not ground truth.

The fastest video still has five unconnected RGB-D nodes: frames 175, 187, 200, 225 and 237. The map omits them. A supported loop at frame 275 reconnects the later component, including frame 250, without inventing the missing intermediate motion.

## Start/end image consistency

I periodically match the initial anchor against the current view. I track the exact SIFT sites across their temporal windows and filter their motion before admitting their descriptors to reciprocal matching. I fit an anchor-to-current RGB-D pose on even 40-pixel checkerboard tiles in the first image and check it on the other tiles. I use the measured pose rather than forcing the camera back to its initial position. A normal loop updates the current pose without rebasing the fixed initial component.

The table compares the retained homography chain and the loop-corrected RGB-D pose in the same end-to-first direction, using common direct-loop image inliers with measured final-frame depth. The scored points lie in odd first-image checkerboard tiles, which were not used to fit the loop. Those spatial areas also support loop acceptance checks, and the methods have different loop-correction capabilities. This measures internal image consistency rather than an unbiased pose benchmark or an isolated temporal-filter ablation.

| Video | Common scored matches | Homography median error | RGB-D median error |
|---|---:|---:|---:|
| 20261119_073137.db3 | 364 | 95.87 px | 0.565 px |
| 20261119_073608.db3 | 323 | 104.46 px | 0.621 px |
| 20261119_073711.db3 | 442 | 119.63 px | 0.641 px |
| 20261119_074041.db3 | 345 | 102.84 px | 0.604 px |
| 20261119_074125.db3 | 324 | 71.44 px | 0.556 px |
| 20261119_074200.db3 | 381 | 46.61 px | 0.613 px |

The low endpoint reprojection errors do not establish centimetre-accurate motion throughout each video. Intermediate drift, loop-pose jumps and local map distortion still need an independent reference.

## Gyro rotation comparison

I fit a constant three-axis gyro bias and choose a clock-shift hypothesis using only the first 20% of each recording, then compare later windows with RGB-D rotations. I test shifts of -50, -25, 0, 25 and 50 milliseconds, where gyro query time equals camera time plus the shift. Every recording selects -50 milliseconds. Because this is the edge of the tested grid, it is a useful hypothesis rather than a precise latency measurement. I do not extrapolate outside recorded sensor coverage.

| Video | Later windows | Raw gyro median disagreement | Calibrated median disagreement |
|---|---:|---:|---:|
| 20261119_073137.db3 | 27 | 1.335 degrees | 0.369 degrees |
| 20261119_073608.db3 | 25 | 1.031 degrees | 0.336 degrees |
| 20261119_073711.db3 | 21 | 1.006 degrees | 0.265 degrees |
| 20261119_074041.db3 | 25 | 0.995 degrees | 0.267 degrees |
| 20261119_074125.db3 | 14 | 1.214 degrees | 0.443 degrees |
| 20261119_074200.db3 | 8 | 1.375 degrees | 0.356 degrees |

These are disagreements with the RGB-D model, not absolute orientation errors. Blindly subtracting the first-second gyro mean can make a moving startup worse, so I keep that conditional experiment separate from the fitted calibration.

## Inertial translation and SfM

I tested a nine-unknown linear initialization for initial velocity, accelerometer bias and gravity using the first 20% of RGB-D positions. I check rank, scaled condition number, plausible gravity and bias, and later-window position RMSE against a constant-velocity prefix baseline. I do not apply a rejected fit to the trajectory.

| Video | Fit rank / unknowns | Scaled condition number | Initialized later RMSE | Constant-velocity later RMSE | Usable |
|---|---:|---:|---:|---:|---|
| 20261119_073137.db3 | 9/9 | 210269 | 34.04 m | 2.46 m | no |
| 20261119_073608.db3 | 9/9 | 66458 | 18.81 m | 4.21 m | no |
| 20261119_073711.db3 | 9/9 | 70732 | 9.31 m | 3.60 m | no |
| 20261119_074041.db3 | 9/9 | 81746 | 30.95 m | 4.05 m | no |
| 20261119_074125.db3 | 6/9 | rank deficient | 2.98 m | 4.22 m | no |
| 20261119_074200.db3 | 6/9 | rank deficient | 2.73 m | 2.30 m | no |

All six initializations fail at least one required check. The four full-rank fits have much worse later position error than the simple baseline. The last two have rank six for nine unknowns, so a smaller error on one video does not identify a reliable bias/gravity solution. The comparison reference is the RGB-D trajectory, including loop corrections, rather than independent metric ground truth. A complete optimized inertial initializer and a recording with independently verified startup conditions are still needed.

I also compare calibrated two-view SfM geometry at the common filtered windows. Its translation has unknown scale, so I compare translation direction only when the pair passes the parallax and geometry checks. Planar ambiguity and large direction disagreements remain. This is a two-view assessment rather than a full SfM reconstruction, and it does not establish metric three-channel agreement.

## Reproduce the run

```bash
python scripts/run_motion_recovery.py --input-dir recordings --output-dir local-results/recovery
python scripts/evaluate_recovery.py --input-dir recordings --results-dir local-results/recovery
python -m pytest -q tests
```

The runner saves five stage checkpoints per recording, logs, component-aware poses, filtered correspondences, local homography maps, registered-depth maps and channel diagnostics. Repeating the completed run reused all 30 stages. The checkpoint keys include input identity, relevant source files, upstream measurements, arguments and numerical-library versions. Command success and scientific usability are separate fields.

I ran 67 regression tests successfully and repeated the four synthetic camera/grass combinations. The synthetic filtered homography errors remained approximately 0, 0.184, 0.007 and 0.234 pixels in the documented case order. These synthetic tests use a known camera prediction; the real-video visual plane proxy can still follow coherent grass motion.

The raw recordings, detailed local outputs and maps stay outside the repository. I have not rerun the unavailable October 7 080033 comparison.

## What still needs evidence

I still need independently labelled camera motion and wind, a full metric SfM reference, reliable inertial translation, and ablations that separate the temporal filter from depth registration and loop correction. The current six-video run establishes recovery and image-consistency improvements, not a new vegetation-motion method or publication result.

I checked closer prior work in [motion-filter-related-work.md](../research/motion-filter-related-work.md). Schofield's 2023 thesis already describes optical-flow rejection before RANSAC in windblown vegetation, so the broad idea is not a novelty claim.

Calibration conventions follow the [librealsense projection/deprojection implementation](https://github.com/realsenseai/librealsense/blob/master/src/rs.cpp) and [recording-format documentation](https://github.com/realsenseai/librealsense/blob/master/doc/record-and-playback-legacy-ros1.md).
