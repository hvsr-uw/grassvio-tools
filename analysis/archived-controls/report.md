# Archived recording controls

I measured the September 26 video and three October controls using the recorded calibration. I retained every requested interval and marked unavailable synchronized depth explicitly. I did not assign zero camera motion or a physical wind label to any video.

## Pairing and calibration

| Recording | Duration | RGB-D windows / requested | First colour/depth offset | Quiet-window mean gyro | Low-motion candidate |
|---|---:|---:|---:|---:|---|
| 20260926_155419.db3 | 1.802 s | 2 / 3 | 26.678 ms | 0.211 rad/s | no |
| 20261007_080033.db3 | 14.684 s | 17 / 18 | 26.897 ms | 0.099 rad/s | no |
| 20261026_173701.db3 | 8.444 s | 10 / 11 | 6.493 ms | 0.026 rad/s | yes |
| 20261030_174745.db3 | 6.641 s | 8 / 8 | 5.797 ms | 0.037 rad/s | no |

The depth pairing limit is 20 ms. The first colour frames of 155419 and 080033 exceed it. I keep those RGB-D intervals unavailable and continue the later comparisons. I do not increase the pairing limit or replace missing measurements with an identity transform. The quiet-window test checks a 0.5 s interval, using mean gyro magnitude below 0.03 rad/s and accelerometer-magnitude MAD below 0.05 m/s^2. Passing these thresholds is a heuristic and does not prove stationary camera velocity.

## Filtering on separate image points

I use 25-frame windows, including the shorter final window, and a 1.5 px temporal residual threshold. Each filter fits only checkerboard training tiles and scores the same held-out tiles. The colour cue uses its existing 0.35 likelihood threshold and a stricter 1.125 px residual threshold. The scores below are medians of available per-window median pixel errors. They measure image consistency across all held-out tracks, including moving objects; they are not wind-detection accuracy.

| Recording | Unfiltered error | Temporal error | Temporal + colour error | Colour candidates | Additional colour rejections |
|---|---:|---:|---:|---:|---:|
| 20260926_155419.db3 | 2.021 px | 1.941 px | 1.941 px | 0 | 0 |
| 20261007_080033.db3 | 0.315 px | 0.313 px | 0.313 px | 0 | 0 |
| 20261026_173701.db3 | 6.095 px | 6.120 px | 6.120 px | 0 | 0 |
| 20261030_174745.db3 | 0.085 px | 0.085 px | 0.085 px | 0 | 0 |

The colour cue makes 0 additional rejections across these 40 windows. These measurements do not establish a colour-cue improvement. A visually fitted plane can follow a coherently moving majority, so these scores also do not prove that the filter separated camera motion from grass motion.

## Same-source September frame comparisons

I also measured every adjacent frame pair in 155419. The recording contains 55 colour frames over 1.802 s, giving 54 adjacent intervals. 53 RGB-D intervals are available; the first interval fails depth pairing. The valid source indices are 1 through 53 and the targets are 2 through 54. The matching count of 53 does not establish that these were the exact endpoints used in the original Fig. 6.

The median RGB-D translation magnitude is 3.758 mm per adjacent interval, and its median rotation magnitude is 0.424 degrees. With raw gyro and zero clock shift, the median relative rotation disagreement against RGB-D is 0.142 degrees. These are model comparisons, not ground-truth errors.

The quietest 0.5 s window still has a mean gyro magnitude of 0.211 rad/s, so it fails the low-motion gate. I do not subtract the video-wide mean gyro as if it were sensor bias. The raw-IMU translation diagnostic carries velocity across intervals but starts at assumed zero velocity and uses an offline gravity heuristic. It is not a validated camera trajectory.

The raw September recording is now available for analysis. The original report and exact plotting implementation are still needed to identify the quantity behind its Fig. 6 curves. These new measurements do not settle that provenance.

## Short-recording initialization

The October 30 video does not supply enough disjoint prefix and later rotation windows for the existing gyro fit. The linear inertial initializer now returns `unavailable_gyro_calibration` and `initialization_usable=false` instead of raising a missing-key error. This reports an unavailable estimate; it does not make the initialization pass.

## Reproduce

```bash
python scripts/inspect_archived_controls.py --input-dir recordings --output-dir local-results/archived-controls --names 20260926_155419.db3 20261007_080033.db3 20261026_173701.db3 20261030_174745.db3
python scripts/inspect_archived_controls.py --input-dir recordings --output-dir local-results/fig6-frame-pairs --window 1 --names 20260926_155419.db3
python scripts/summarize_archived_controls.py --controls-dir local-results/archived-controls --pairs-dir local-results/fig6-frame-pairs --outdir analysis/archived-controls
python -m pytest -q tests
```

Each local `controls.json` contains the input SHA-256, full interval list, depth offsets, relative transforms, filter counts and scoring definitions. The repository summary contains aggregates and input hashes. Recordings and detailed motion traces remain outside the repository.

Independent motion accuracy and physical wind validation remain unresolved. I still need independently measured camera motion and labelled stationary/moving vegetation controls to make those claims.
