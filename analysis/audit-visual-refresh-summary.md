# Visual refresh — trajectory overlay, four-panel video, depth translation check

Date: 2026-11-09. Take 20261007_080033 for the two visual deliverables,
take 20261026_173701 against control 20260926_102631c for the depth
check. Bags read read-only. Nothing uploaded.

## Verdicts

- **Overlay: keep the new figure.** The ORB-pose map with the ORB-SLAM3
  path replaces the old homography overlay. The old path's big loop is
  mostly chain drift: the real camera stayed in a 9 by 5 cm patch.
- **Video: the rebuild works.** The four-panel video on the ORB map
  renders, all four panels animate, and its final panorama panel is the
  ORB map itself (mean difference 3.9 gray levels, 99.0% footprint
  overlap). The IMU arc still drifts off while the ORB loop closes.
- **Depth check: elevated, but inconclusive.** The median 3D
  displacement on 173701 is 22.6 mm against 4.1 mm on the control, so
  translation parallax is formally back on the table. But the fitted
  translation cannot be literal camera motion: it implies about 1 m of
  travel that the long-baseline image check rules out, and the PnP fit
  itself is biased on this data (its rotation is 2.3 times the gyro on
  both takes). The most likely cause of the elevation is the unaligned
  depth on discontinuous foliage, not camera motion. The check neither
  proves translation explains the signal nor excludes sub-millimetre
  translations. Its floor on foliage is centimetre-level and
  artifact-dominated.

## Task 1 — Trajectory overlay on the ORB map

Method: the base is the best map, the last-wins ORB-pose orthomosaic
(668 by 445 px, 6.67 by 4.44 m). The path is the 441 ORB-SLAM3 poses.
Map geometry was recomputed with the renderer's own bounds code, so the
path lands on the same pixels the renderer's trajectory figure uses.

Numbers, all measured from the files:

| | new overlay | old overlay |
|---|---|---|
| base | ORB-pose orthomosaic, 668x445 | homography chain panorama, 2204x1891 |
| path | ORB-SLAM3 poses (441) | frame centres through chained homographies (147 frames, stride 3, 1 skipped, median 2251 inliers) |
| path extent on screen | 9 by 5 px (1.3% of map width) | 417 by 289 px (19% of canvas width) |
| start to end | 6.0 px = 0.060 m (3D end error 0.061 m) | markers close together on a sheared map |
| camera travel | 0.55 m in the plane, 0.69 m in 3D | not measurable in metres from the chain |

Cross-check: the computed start pixel (318.3, 329.8) and end pixel
(324.2, 331.0) reproduce the dots in the renderer's trajectory figure
within 1 to 2 px.

Judged by eye: the new overlay shows the coherent grass patch with
"start" and "end" labels and two distinct ring markers at its centre.
The magenta path is hidden behind the markers at map scale, which is
honest: the whole path fits in a 9 by 5 px box. The loop shape only
reads in the separate detail figure (centimetre scale), where the path
is a genuine small loop that starts at the origin, wanders left and
down, crosses to the right, and ends 6 cm from its start. The old
overlay shows a large magenta loop on a sheared diamond; the loop shape
is real but its size is chain drift, not travel.

Two fixes were needed before the figure passed its eye check. Filled
markers swallowed the path, so the markers are hollow rings that leave
the path visible inside them. The first crop cut the labels off, so the
crop margin went to 45 px at 6 times magnification.

Verdict: keep the new figure. The old figure's path is tens of times
larger on screen than the real camera motion, and its base is a sheared
map. The new figure's only weakness is that the path is a dot at map
scale, which the crop and the detail figure cover.

Files: `overlay_orb_080033.png`, `overlay_orb_080033_crop.png`,
`overlay_orb_080033_box.png`, `overlay_comparison.png`,
`trajectory_detail_orb_080033.png`, `overlay_orb_080033.json`,
`overlay_orb_080033.py`.

## Task 2 — Four-panel video rebuilt on the ORB map

The existing demo video was checked first, by extracting frames 0, 22,
44, 66, and 88 and looking at them. It shows what the November 1
summary says: the camera panel, the IMU blue arc drifting off, the
magenta homography path roughly closing with a small overshoot kink at
the end, and the panorama panel building up the sheared homography
mosaic. The old video is 89 frames at 10 fps, 1280 by 960, 8.9 s, with
a 2351 by 2101 px chained canvas.

Rebuild: same layout, same bag, stride 5, 89 frames at 10 fps. The
camera and IMU panels reuse the original script's code unchanged. The
third panel is the ORB-SLAM3 trajectory, labelled as such instead of
"visual odometry". The panorama panel splats frames in order with the
renderer's rules (last-wins, depth capped at 3 m, only frames within
35 deg of straight down), so the final frame is the best map itself.

Judged by eye on the new video's frames 0, 44, and 88:

- Frame 0: the panorama panel shows the first frame's splat as a small
  quadrilateral patch. The IMU and ORB panels show only their green
  start dots.
- Frame 44: the IMU arc matches the old video exactly (same code). The
  ORB panel shows the magenta path mid-loop. The panorama panel shows
  the ORB map building up as a coherent patch with no shear.
- Frame 88: the IMU arc ends far left of its start. The ORB loop closes
  back near its green start with the same small overshoot kink the old
  video's path had. The panorama panel shows the full ORB map.

Quantitative check on the final frame: the panorama panel matches the
base map at a mean absolute difference of 3.9 gray levels over the map
footprint, 0.03% of footprint pixels differ by more than 30 levels, and
the footprint overlap is 99.0% (the gap is the label bar covering the
top rows plus resize edge effects). The difference is video compression
level, not a different map.

Caveat, carried over from the original script: the IMU and ORB path
panels are each auto-scaled to fit their panel independently, so the two
paths are not in the same units. The "IMU drifts, ORB closes" reading
is about shape and closure, not distance.

Verdict: the rebuild works and replaces the old video. No technical
blocker was hit.

Files: `four_panel_orb_080033.mp4` (89 frames, 10 fps, 1280x960,
10,119,954 bytes), `four_panel_orb_080033.json`,
`four_panel_orb_080033.py`, `new-video-frames/` (frames 0, 22, 44, 66,
88), `old-video-frames/` (same frames from the old demo for comparison).

## Task 3 — Depth-based translation check on 173701

Method: the same stride-5 pairs as the wind-parallax work (50 pairs on
173701, 24 on the control), the same SIFT settings (10000 features,
Lowe 0.75). Per pair: the median 3D displacement of matches with valid
depth in both frames, back-projected with depth (the IMU fusion
script's visual-distance statistic, depth intrinsics, valid depth 0.05
to 3.0 m); the same statistic with the color intrinsics as a
sensitivity check; the relative pose from solvePnPRansac (the RGB-D
odometry method) with its translation magnitude |t| and rotation; and
the post-fit 3D residual after the best rigid motion is removed. The
2D residual and the accel proxy were recomputed in the same run, so
every comparison is on the same pairs. The run was done twice; the
headline numbers reproduce within 0.3 mm. All 254 frames on 173701 and
all 124 on the control paired with depth (worst pairing offset 7.1 ms
on 173701, 7.0 ms on the control).

Headline numbers (median of per-pair medians unless noted):

| | 173701 | control 102631c | ratio |
|---|---|---|---|
| 2D residual (px) | 1.12 | 0.19 | 5.9x |
| median 3D displacement, depth intrinsics (mm) | 22.6 | 4.1 | 5.5x |
| median 3D displacement, color intrinsics (mm) | 22.5 | 4.1 | 5.5x |
| pooled median 3D displacement (mm) | 19.9 | 4.2 | 4.7x |
| fitted translation PnP \|t\| (mm) | 25.9 | 4.9 | 5.3x |
| post-fit 3D residual (mm) | 35.1 | 15.7 | 2.2x |
| accel proxy (mm) | 3.69 | 1.66 | 2.2x |
| median matched depth (m) | 1.33 | 0.65 | |
| rotation-only displacement predicted (mm) | 12.6 | 5.2 | |
| correlation, 2D residual vs PnP \|t\| | 0.80 | -0.23 | |

The intrinsics choice does not matter (22.6 vs 22.5 mm). Rotation
accounts for part of the 3D displacement but not all of it: on 173701
the rotation-only prediction is 12.6 mm against 22.6 mm observed, while
on the control it is 5.2 mm against 4.1 mm observed. The 2D residuals
reproduce the wind work exactly (1.12 and 0.19 px), so the pairs match.

Splits on 173701: the elevation is present in both halves (3D
displacement 21.1 mm first half, 25.5 mm second; |t| 21.7 mm first,
29.6 mm second) and in the proxy-still segment (19 pairs: 3D
displacement 15.7 mm, |t| 16.6 mm, 2D residual 0.67 px, against
control still-segment values of 4.1 mm, 4.9 mm, 0.19 px). At onset
(pairs 5-8) the 3D statistics are already about twice the control
(11.7 mm displacement, 11.2 mm |t|, 2D residual 0.48 px), while the
first four pairs sit near control level (7.2 mm, 8.5 mm, 0.20 px). The
3D elevation switches on with the 2D signal, not after it.

Follow-up diagnostics, which decide what the elevation means:

1. **The PnP rotation does not match the gyro.** PnP rotation is 1.19
   deg per pair against 0.54 deg implied by the gyro on 173701, and
   1.18 deg against 0.46 deg on the control: a factor of 2.3 and 2.5.
   The fit is biased on this data, the same on both takes, so its
   translation is not a clean measurement either.
2. **The fitted translation cannot be real camera motion.** Its mean
   vector on 173701 is (-19.2, -8.2, -9.2) mm per pair, consistent in
   direction (mean cosine between consecutive pairs 0.97, all 49 pairs
   above 0.7, none negative). Sustained, that is about 1 m of travel
   over the take. The long-baseline image check rules that out: SIFT
   matches between frame 0 and frame 100 (125 matches) move a median
   of 50 px, which the gyro rotation alone explains (about 12 deg over
   those frames). A 1 m translation would add at least 100 px on top
   of that. The no-flip direction pattern also rules out oscillatory
   shake with a period shorter than the take.
3. **The control shows the same biases smaller.** Its fitted
   translations are also directionally consistent (mean cosine 0.91),
   and its post-fit 3D residual is 15.7 mm on a rigid take: that is the
   floor of the 3D statistics on this data, from depth noise and the
   unaligned depth. The 173701 foliage has a wider matched-depth
   spread (10th to 90th percentile 1.01 to 2.37 m, against 0.48 to
   1.35 m on the control) and far fewer valid depth matches per pair
   (median 600 against 1469), which is where the 5.9 cm depth-color
   baseline bites hardest: a shifted depth sample lands on a different
   leaf.

Verdict: per the pre-stated rule the elevation puts translation
parallax back on the table, but the follow-ups show the elevation is
most plausibly the unaligned-depth artifact on discontinuous foliage,
not camera motion. The depth check as designed cannot test
sub-millimetre translations on this scene: its floor is centimetre
level and artifact-dominated. It neither proves translation explains
the signal nor excludes small translations. It is inconclusive, and the
wind-parallax verdict should carry that: the "persists while still"
finding still rests on the gyro and proxy evidence, and the depth check
was supposed to confirm it independently and did not.

Two corrections to the wind-parallax record follow from this work.
First, "frame 0 and frame 200 show the same viewpoint; the camera did
not move between them" is too strong: the camera rotates about 30 deg
over the take (gyro mean 0.063 rad/s over 8.4 s) and drifts, with frame
0 to 100 matches already displaced a median of 50 px. "Same viewpoint"
holds only loosely by eye. Second, the open follow-up is now sharper:
warp the depth into the color frame (the 5.9 cm baseline) and re-run
this check. Until that is done, no depth-based translation claim on
173701 should be trusted at the centimetre level or below.

Files: `depth_translation_173701.py`, `plot_depth_translation.py`,
`depth-check/depth_summary.json`,
`depth-check/20261026_173701_depth_pairs.json`,
`depth-check/20260926_102631c_depth_pairs.json`,
`depth-check/fig_depth_time.png`,
`depth-check/fig_depth_scatter.png`,
`depth-check-run.log`, `depth-check-run2.log`.

## Files in this directory

- `SUMMARY.md` — this file.
- `PROGRESS.md` — work log.
- `overlay_orb_080033.py`, `overlay_orb_080033.png`,
  `overlay_orb_080033_crop.png`, `overlay_orb_080033_box.png`,
  `overlay_comparison.png`, `trajectory_detail_orb_080033.png`,
  `overlay_orb_080033.json` — Task 1.
- `four_panel_orb_080033.py`, `four_panel_orb_080033.mp4`,
  `four_panel_orb_080033.json`, `new-video-frames/`,
  `old-video-frames/` — Task 2.
- `depth_translation_173701.py`, `plot_depth_translation.py`,
  `depth-check/` — Task 3.
