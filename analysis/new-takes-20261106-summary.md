# New takes Nov 6 — wind blowing and illumination change

Date: 2026-11-06. Jinming's message this morning: "I uploaded more
recordings for wind blowing and illumination change. Please check out!"
Eight new takes (20261106_084542, 084741, 084852, 085224, 085301,
085422, 085453, 085611), all recorded Nov 6, 08:45-08:56, all read
from the Box download session sess-2653029502. The 085611 file appears
twice on disk (first download attempt errored, retry succeeded); the
two files are byte-identical (md5 0b7de10d7030202ada40f076ea4eac26) and
the second (-8) file was used. Every bag passed sqlite integrity_check
and opened cleanly. Bags read read-only.

Methods are the same as the 41-take feature table (see
analysis-drafts/recording-feature-table.md): loop = SIFT last-5 vs
first-5 stride-5 frames, 20+ inliers = YES; wind = rigid-fit residuals
on consecutive pairs vs the wind-free control 102631c (pooled median
0.185 px / p90 0.458 px, recomputed here as 0.185 / 0.458, confirming
the pipeline); motion classes from the gyro thresholds in the table;
stitch = chain SIFT, stride 5, same settings as batch_stitch.py, with
the 8000 px canvas guard. New this time: an illumination check
(per-frame mean gray over time; brightness range, largest
frame-to-frame jump, and whether SIFT match counts drop when
brightness jumps), calibrated on the same control (brightness range
3.06, max jump 0.73, std 0.72 over its 4.1 s).

Script: analyze_new_takes.py (new). Stitch driver: stitch_new_takes.py
(same chain as batch_stitch.py). RGB-D: rgbd_odometry.py + render_v2.py
(existing, untouched). Frame sheets and brightness plots were opened
and judged by eye; every panorama below was opened and judged.

## Headline result

No take in this batch shows a wind signal, by any test tried. All 8
takes sit at or below the wind-free control on rigid-fit residuals
(pooled medians 0.093-0.181 px vs 0.185 control), and no single pair
in any take has a median residual above 0.5 px. The batch's measurable
content is illumination variation, not wind. Don's still-camera clip
(camera still, grass moving) is not in this batch by these
measurements — the four still takes are the cleanest takes in the
whole dataset by the residual test, and a closer look at their
frame-to-frame changes finds camera shake and moving shadows, not
moving grass (see the deep dive below).

The stitching is the best-behaved yet: all 8 takes stitched, 0 skipped
pairs, median inliers 1900-3777, no blow-ups. The fallen leaves all
over the lawn are distinctive features everywhere, which is likely why
matching is so strong here.

## Per-take table

| take | duration | color frames | motion class | loop | wind (med / p90) | illumination (range / max jump) | stitch |
|---|---|---|---|---|---|---|---|
| 20261106_084542 | 11.8 s | 356 | still | YES trivial (max 890) | no wind signal (0.108 / 0.312) | range 8.54, max jump 1.01 | worked, 644x492, med 3160 |
| 20261106_084741 | 13.8 s | 415 | still | YES trivial (max 565) | no wind signal (0.108 / 0.308) | range 8.59, max jump 1.72 | worked, 643x483, med 2629 |
| 20261106_084852 | 17.1 s | 513 | still | YES trivial (max 371) | no wind signal (0.097 / 0.279) | range 8.05, max jump 0.97 | worked, 642x482, med 3119 |
| 20261106_085224 | 17.5 s | 524 | rotation (high); translation uncertain | NO (max 0) | no wind signal (0.181 / 0.461) | range 33.89, max jump 1.69 | worked, 1052x914, med 2095 |
| 20261106_085301 | 14.4 s | 432 | rotation (high); translation uncertain | NO (max 0) | no wind signal (0.180 / 0.484) | range 66.21, max jump 2.99 | worked, 1394x957, med 2777 |
| 20261106_085422 | 13.7 s | 411 | rotation (moderate); translation uncertain | NO (max 0) | no wind signal (0.171 / 0.423) | range 20.45, max jump 2.24 | worked, 848x694, med 2518 |
| 20261106_085453 | 13.1 s | 393 | rotation (high); translation uncertain | NO (max 0) | no wind signal (0.176 / 0.431) | range 39.30, max jump 10.14 | sort-of-ok, 1054x860, med 1900 |
| 20261106_085611 | 12.8 s | 386 | still | YES trivial (max 3731) | no wind signal (0.093 / 0.266) | range 23.31, max jump 4.39 | worked, 642x482, med 3777 |

Stitch tally for the batch: 7 worked, 1 sort-of-ok, 0 blow-ups.

## Which takes are wind, which are illumination

Wind-blowing: none identified. Every take is at or below control on
the residual test, including the four still takes, which are the
cleanest in the dataset so far (medians 0.093-0.108 px, below the
0.185 px control). See the deep dive below for what the frame-level
checks found instead.

Illumination-change: all 8 takes vary more than the control (range
8.05-66.21 vs 3.06), but the character differs and on the rotating
takes the brightness curve mixes view change with light change:

- 20261106_085611 — the cleanest genuine illumination change. Still
  camera, fixed view of grass and leaves, brightness oscillates
  repeatedly between about 90 and 114 (range 23.31) for the whole
  12.8 s. The two frames at the extremes show the same leaves in the
  same places, one darker and one brighter: the light itself is
  changing, not the scene. No pixel-motion event anywhere in the take
  (frame-difference max 4.08, at baseline).
- 20261106_085301 — the largest swing (range 66.21). Brightness flat
  near 118 for the first 6 s, then a smooth wave up to 133, down to
  101, then a steady climb to 167 by the end. The frames show the
  camera rotating from shaded leaf litter into a sunlit patch, so
  most of this swing is the view changing, not the light changing.
  Same caution applies to 085224 (smooth rise 115 to 140) and 085422
  (wobble 104-125 with a moving shadow band visible in its panorama).
- 20261106_085453 — one sharp 10.14 jump, but it is not a light
  change: the frames show a white sign entering the view as the
  camera rotates past it. The brightness jump is a white object
  entering the frame. Otherwise a wobble like 085422.
- 20261106_084542, 084741, 084852 — near-steady. 084741 and 084852
  sit flat after the first 1-2 s of auto-exposure settling (their
  ranges 8.59 and 8.05 are mostly that start-up step, 99 to 106 and
  93 to 101). 084542 instead wobbles slowly through the take
  (about 107 up to 116 near 6 s, back to 111). Each has one mid-take
  event: see the deep dive.

## What illumination changes do to SIFT matching

Measured, not assumed. Matching never breaks in this batch: every
consecutive pair fits (355/355 to 523/523 per take), median inliers
per pair stay in the thousands (2490-4048 even on high-jump pairs),
and no pair in any take fails. Brightness jumps do cost some matches:
the correlation between jump size and per-pair inlier count is
weakly negative on all 8 takes (-0.07 to -0.52), strongest on 085611
(-0.52; median inliers 3435 on high-jump pairs vs 4010 on low-jump
pairs, about a 14% drop). On 085301 the drop is about 9% (3691 vs
4048) and on 085453 about 13% (2490 vs 2865). So illumination change
thins the matches but does not break tracking at these magnitudes.

## Deep dive: what is actually moving in the still takes

The four still takes all read "no wind signal", but their
frame-to-frame pixel differences are not all flat, so each suspicious
event was checked directly (feature displacement field + phase
correlation + diff heatmap), not just by the residual test:

- 084542 — intermittent camera micro-shake. Frame differences run
  3-9 through the take (baseline for a still take is about 2.5),
  with two episodes up to 17.4 (frames 35-45) and 15.9 (frames
  175-215). The displacement field during the first episode is
  uniform across the whole frame (about 1 px, same direction
  everywhere) — that is the camera shifting, not grass moving.
  Grass swaying would be patchy and direction-varying. The gyro
  stays low (mean 0.0147) because the shake is a small translation,
  not a rotation.
- 084741 — a moving shadow. Frame differences peak mid-take (up to
  8.3) in a localized blob in the lower-center of the frame, but
  feature displacements stay at baseline (median 0.12 px, no global
  shift). The pixels change while nothing moves: a shadow sweeping
  across the lawn. The diff heatmap shows a broad soft-edged region,
  not an object.
- 084852 — an illumination event at about pair 310 (frame difference
  10.28). No feature motion (median displacement 0.13 px, no global
  shift) and the SIFT feature count halves during it (1834 vs about
  3800 at baseline), consistent with part of the frame going dark
  under a shadow.
- 085611 — nothing moves at all. Frame differences stay at baseline
  the whole take; the brightness oscillation is a slow global light
  change on a fixed view.

None of the four events is grass moving. That is why the batch is
reported as having no wind take: the residual test, the displacement
test, and the frame views all agree.

## RGB-D orthomosaics (still takes)

All four still takes ran through rgbd_odometry.py + render_v2.py
(all pairs ok: 71/71, 82/82, 102/102, 77/77, pair inliers in the
thousands). The maps are NOT showable, and the reason is measured,
not a rendering problem: the chained VO poses drift badly on a camera
that never moves.

| take | VO pairs ok | VO path length | VO start-end translation | VO start-end rotation | map extent | v2 coverage | map verdict |
|---|---|---|---|---|---|---|---|
| 20261106_084542 | 71/71 | 0.30 m | 0.27 m | 80.3 deg | 4.97 x 4.27 m | 24.5% | smeared wedge, not showable |
| 20261106_084741 | 82/82 | 0.32 m | 0.27 m | 96.3 deg | 4.66 x 3.07 m | 58.9% | streaked wedge, not showable |
| 20261106_084852 | 102/102 | 0.40 m | 0.33 m | 117.7 deg | 5.28 x 2.41 m | 73.6% | striped smear, not showable |
| 20261106_085611 | 77/77 | 0.28 m | 0.24 m | 86.3 deg | 3.95 x 2.96 m | 57.7% | smeared wedge, not showable |

The camera is still in all four takes (gyro means 0.006-0.015 rad/s,
translation proxy 0.007-0.092 mm), yet the VO chain says it traveled
about 0.3 m and rotated 80-118 degrees. Every per-pair PnP fit looks
confident (inliers in the thousands, no pair fails), so the drift is
a small per-pair rotation bias — about 1 degree per pair — compounding
over 70-100 chained pairs. The v1 and v2 renders agree (both smeared),
so it is the poses, not the render. The homography panoramas are the
maps to show for these takes: they are clean single-view maps, because
a still camera needs no chaining at all. Lesson for the project: RGB-D
VO needs real camera motion to anchor on; on a still camera the chain
drifts and the orthomosaic is worse than the panorama, not better.

## Stitch notes

- 085453 is the only sort-of-ok: its panorama is a coherent fan, but
  the far end stretches into a thin diagonal streak and there is a
  brightness seam across the middle. The white sign the camera sweeps
  past does not appear in the map.
- 085224 and 085301 are worked: coherent fans, leaves line up, no
  stretch. 085301 has mild blur in the densest leaf litter from
  averaging overlapping frames.
- 085422 is worked, and its panorama visibly carries the moving
  shadow as a diagonal darker band across the map.
- The four still takes stitch to clean single-view maps (642x482),
  as expected: the camera never moved.

## Feature table extension

The 41-take table (analysis-drafts/recording-feature-table.md) gains
8 rows: 4 still takes and 4 rotating takes, all Nov 6. New totals:
49 takes, 48 usable (172642 remains the only failed recording). Loop:
the 4 still takes close trivially; the 4 rotating takes are NO (all
25 pairs zero). Wind: all 8 "no wind signal". The table's evidence
rules are unchanged: loop labels come from the end-vs-start check
only, wind labels are signal-vs-control comparisons, never wind
proof, and motion classes come from the IMU alone.

## Files

- analyze_new_takes.py — per-take checks (motion, wind, loop,
  illumination) + brightness plots.
- stitch_new_takes.py — chain stitch driver for the 8 takes.
- run_rgbd.sh — RGB-D odometry + render v2 for the 4 still takes.
- checks/ — per-take JSONs (motion, wind, loop matrix,
  illumination, brightness curve) + _summary.json +
  *_brightness.png plots.
- stitch/ — per-take panoramas + JSONs + results.jsonl.
- plots/frames/ — frame sheets per take + the 085611 peak-vs-trough,
  085453 spike, 084741 intrusion and diff checks.
- rgbd/ — poses, orthomosaics, trajectories, render-v2 variants for
  the 4 still takes.
