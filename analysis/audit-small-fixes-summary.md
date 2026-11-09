# Small fixes — K_COLOR and the feature-table translation proxy

Date: 2026-11-09. Work area:
`analysis-drafts/full-audit-20261109/small-fixes/`. CS 566 only.
Nothing was uploaded to GitHub. No message was sent to Jinming,
Julius, or Don. All bags were read read-only. Every image below was
opened and judged by eye.

## Gap 1 — K_COLOR in rgbd_odometry.py does not match the bag

### Verification of the mismatch

Calibration was read from the 080033 bag itself with the depth-warp
`calibration.py` (`calib_080033_verify.json`, `calib_080033_verify.log`):

- Bag color camera_info: fx = 386.040802, fy = 385.300934,
  ppx = 323.054626, ppy = 250.530975, Inverse Brown Conrady,
  k1 = -0.056599766, k2 = 0.0620031841, p1 = -0.000234535619,
  p2 = 0.000596299826, k3 = -0.0195424054.
- Bag depth camera_info: fx = fy = 386.009827, ppx = 320.77182,
  ppy = 240.889587, zero distortion.
- `K_COLOR` in `dataset-tools/stitch-map/rgbd_odometry.py` was
  fx = 386.251221, fy = 385.510895, ppx = 323.054626,
  ppy = 250.530975. So fx was 0.210 px too high and fy 0.210 px too
  high; ppx and ppy already matched the bag exactly.
- `K_DEPTH` in the same script already matches the bag exactly.

Distortion: the script passes `None` as the distortion argument to
`cv2.solvePnPRansac`, so the color distortion coefficients are not
used anywhere in the script. No distortion change was made.

### The fix (minimal)

In `dataset-tools/stitch-map/rgbd_odometry.py`, two numbers changed:

```
K_COLOR = np.array([[386.040802, 0, 323.054626],
                    [0, 385.300934, 250.530975],
                    [0, 0, 1.0]])
```

plus a three-line comment stating the values are the color
camera_info from the 080033 bag and giving the previous values.
Nothing else in the file changed (`rgbd_odometry_before.py` is the
copy of the file before the fix; the diff is the two numbers plus
the comment). `render_v2.py` imports only `DEPTH_UNITS`, `K_DEPTH`,
`load_synced`, and `unproject` from the script, not `K_COLOR`, so
the fix reaches the render only through the poses.

### Re-run on 20261106_085611 (still camera, cleanest test)

Baseline is the Nov 6 run in
`analysis-drafts/new-takes-20261106/rgbd/20261106_085611_poses.json`
(end error 0.238 m / 86.3 deg, recomputed here as 0.2379 m /
86.33 deg). The take was re-run with the fixed intrinsics
(`vo_085611_fixed_poses.json`, `vo_085611_fixed.log`) and, as an
extra check, with 085611's own bag intrinsics
(`vo_085611_ownK_poses.json`, `run_vo_ownK.py`, which monkeypatches
`K_COLOR` without touching the file). 085611's own bag calibration
is fx = 385.409698, fy = 384.67099 (`calib_085611_verify.json`) —
different again from the 080033 bag. Each recording session carries
its own calibration (the September takes in `imu_fusion.py` list
386.672/385.931 and 386.567/385.826), so no single hard-coded
`K_COLOR` can match every take; the fix matches the 080033 bag as
specified, and a per-bag calibration read would be the fuller fix.

| run | pairs ok | end translation | end rotation | path length | map extent | inliers median |
|---|---|---|---|---|---|---|
| baseline K (old) | 77/77 | 0.2379 m | 86.33 deg | 0.2783 m | 3.948 x 2.651 m | 2385 |
| fixed K (080033 bag) | 77/77 | 0.2445 m | 86.80 deg | 0.2848 m | 3.944 x 2.660 m | 2371 |
| own-bag K (085611 bag) | 77/77 | 0.2778 m | 88.45 deg | 0.3179 m | 3.907 x 2.419 m | 2415 |

V2 render from the fixed-K poses (`vo_085611_fixed_v2.json`):
extent 3.949 x 2.974 m against baseline 3.953 x 2.962 m, coverage
57.65% against 57.73%, 156 frames splatted against 157. Pixel
comparison: fixed-K orthomosaic against baseline, mean absolute
difference 12.37 with 52.1% of pixels identical; v2 last-wins on
the shared overlap, mean absolute difference 9.38 with 54.9%
identical.

### Verdict — the fix changes nothing measurable

The end error moves from 0.238 m / 86.3 deg to 0.245 m / 86.8 deg
with the fixed intrinsics, and to 0.278 m / 88.5 deg with the
take's own exact intrinsics. The 86-degree drift on a camera that
never moves survives the fix and survives exact-match intrinsics.
The K mismatch was real and is now fixed, but it is immaterial to
this visual odometry: the drift is a per-pair rotation bias of
about 1 degree per pair compounding over 77 chained pairs, not a
calibration error. That is a result, stated plainly.

### Judged by eye

`comparison_vo_085611.png` (all five panels opened individually
first): baseline, fixed-K, and own-bag-K orthomosaics, plus
baseline and fixed-K v2 last-wins. All five are the same smeared
wedge: same shape, same vertical pink stripes, same small speck
below the map. The own-bag-K map is slightly narrower
(392x243 against 396x267) but the same smear. The map stays not
showable, exactly as the November 6 summary called it.

## Gap 2 — feature-table translation-proxy column does not reproduce

### Reproduction of the discrepancy

The table's own motion script
(`analysis-drafts/table-completion/motion_stats.py`) was re-run on
the two bags (`gap2_motion_repro/`, `gap2_motion_repro.log`):

| take | table mean / median / max (mm) | re-run mean / median / max (mm) |
|---|---|---|
| 20261026_173701 | 0.132 / 0.109 / 0.579 | 0.236 / 0.217 / 0.729 |
| 20260926_102631c (control) | 0.084 / 0.074 / 0.277 | 0.142 / 0.128 / 0.326 |

The gyro numbers reproduce exactly (173701 gyro mean 0.06317,
control 0.05026, as published), and the accel column reproduces
(the table's accel column is the mean of per-sample magnitudes,
9.7094 for 173701, against the magnitude of the mean vector,
9.7023). Only the proxy column disagrees. The wind-parallax
cross-check independently gives the same re-run numbers
(0.236/0.217/0.729 and 0.142/0.128/0.326 in its pairs JSONs).

### Why the published values differ

The October 30 table was built with an earlier integration
variant; the script was then changed to the current method for the
November 1 extension and the old rows were never recomputed.
Evidence, in `gap2_variants.py`, `gap2_variants2.py`,
`gap2_variants3.py`, and the full 49-take recompute
`gap2_full.py` / `gap2_full_recompute.json`:

- Ten integration variants were tested on 173701 and the control.
  Semi-implicit Euler with interval-clipped dt (the current
  script, `imu_fusion.py` step 3) gives 0.236 / 0.142 mm.
  Explicit Euler over only the accel samples inside each frame
  interval, each with its full gap-to-next-sample dt, gives
  0.138 / 0.086 mm mean, 0.112 / 0.076 mm median,
  0.592 / 0.284 mm max — against published 0.132 / 0.084,
  0.109 / 0.074, 0.579 / 0.277. The other variants are far off
  (trapezoidal 0.187 / 0.113, single-step 0.050 / 0.030,
  per-interval demeaning 0.040 / 0.032, filtered window-5
  0.221 / 0.122, median demeaning 0.236 / 0.142,
  inside-only clipped dt 0.175 / 0.110, xy-only 0.201 / 0.099).
- Recomputing all 49 takes both ways: the 36 rows from
  2026-09-26, 2026-10-07, and 2026-10-26 match the explicit
  variant with a median published/recomputed ratio of 0.947
  (10th percentile 0.872, 90th percentile 0.983) across mean,
  median, and max, while they sit at a median ratio of 0.549
  against the current method. The 13 newer rows (2026-10-30 and
  2026-11-06) match the current method exactly, 39 of 39 stats
  to three decimals. The script file `motion_stats.py` carries a
  November 1 modification time, and the November 6 script
  `analyze_new_takes.py` uses the current method verbatim.
- The exact old code is not on disk, so the variant match is
  strong evidence, not a proof: no tested variant reproduces the
  published values exactly (0 of 108 old-row stats exact under
  the explicit variant). The remaining few percent could be a
  small edge detail of the old script.
- One row matches neither variant: 102608b, published
  0.020 / 0.007 / 0.284 mm against explicit 0.040 / 0.028 / 0.372
  and current 0.121 / 0.110 / 0.336. The two downloads of that
  take are byte-identical (md5 1e09789e872efcdfe677ca9988c82fe7),
  so a different bag file is ruled out; that row's original
  error is unexplained.

The vectorized recompute was checked against the scalar
`motion_stats.py` on 080033 (2.010 / 1.999 / 3.861 mm both ways)
and 173701 (0.236 / 0.217 / 0.729 mm both ways).

### Correction applied to the table

`correct_table.py` rewrote the proxy mean, median, and max in the
IMU appendix of `analysis-drafts/recording-feature-table.md` for
the 36 affected rows, from `gap2_full_recompute.json`. Verification
after the edit: all 49 appendix rows equal the recomputed values,
all non-proxy fields are byte-identical to the original
(`recording-feature-table_before.md` is the backup), and the 13
newer rows were untouched. Corrected means for the 36 rows:

| take | published mean | corrected mean |
|---|---|---|
| 20260926_102608b | 0.020 | 0.121 |
| 20260926_102631c | 0.084 | 0.142 |
| 20260926_102706d | 0.096 | 0.169 |
| 20260926_102737a | 0.219 | 0.397 |
| 20260926_153136 | 0.067 | 0.109 |
| 20260926_153610 | 0.053 | 0.095 |
| 20260926_153825 | 0.126 | 0.228 |
| 20260926_155411 | 0.076 | 0.136 |
| 20260926_155419 | 0.228 | 0.419 |
| 20260926_155430 | 0.068 | 0.119 |
| 20260926_155507 | 0.069 | 0.122 |
| 20260926_155555 | 0.068 | 0.118 |
| 20260926_155614 | 0.076 | 0.134 |
| 20261007_075947 | 0.418 | 0.764 |
| 20261007_080033 | 1.079 | 2.010 |
| 20261026_172532 | 0.461 | 0.840 |
| 20261026_172642 | 0.134 | 0.219 |
| 20261026_172644 | 0.329 | 0.588 |
| 20261026_172739 | 0.339 | 0.606 |
| 20261026_172828 | 0.446 | 0.813 |
| 20261026_172859 | 0.197 | 0.353 |
| 20261026_172912 | 0.232 | 0.429 |
| 20261026_172955 | 0.233 | 0.430 |
| 20261026_173017 | 0.200 | 0.366 |
| 20261026_173045 | 0.389 | 0.712 |
| 20261026_173124 | 0.545 | 1.002 |
| 20261026_173154 | 0.565 | 1.039 |
| 20261026_173220 | 0.674 | 1.251 |
| 20261026_173250 | 0.253 | 0.460 |
| 20261026_173403 | 0.206 | 0.377 |
| 20261026_173507 | 0.299 | 0.543 |
| 20261026_173540 | 0.522 | 0.960 |
| 20261026_173603 | 0.430 | 0.785 |
| 20261026_173628 | 0.460 | 0.848 |
| 20261026_173701 | 0.132 | 0.236 |
| 20261026_173807 | 0.570 | 1.055 |

Median and max for every row are in the appendix and in
`gap2_full_recompute.json`. Prose that quoted proxy numbers was
fixed in the same edit: the still-take noise floor from about
0.1-0.15 mm to about 0.1-0.25 mm per interval; the 0.2 mm line
example now reads 172859 (0.353 mm) and 173017 (0.366 mm) together
in the uncertain band; the 102737a note now says proxy 0.397 mm;
the 080033 note now says 2.01 mm per interval mean. A correction
note was added to the table before the Methods section, stating
what was wrong, the corrected headline values, and the scope. The
corrected values agree exactly with `imu_fusion.py`'s own
displacement numbers on the four September takes it supports
(0.121, 0.142, 0.169, 0.397 mm), so the table's cross-check
sentence now holds exactly.

### Wind and loop verdicts are unaffected — verified

- Wind verdicts rest on the rigid-fit residual appendix (pooled
  median/p90 against the control), which does not use the proxy
  column and was not touched. No wind cell changes.
- Loop verdicts rest on SIFT end-vs-start inliers, which use no
  IMU at all. No loop cell changes.
- The wind-parallax test on 173701 already used the
  current-method proxy on both takes for its still/moving split,
  so its verdict already rested on corrected values.

### What does change: translation bands and motion-class labels

Twelve takes cross a translation band (negligible below 0.2 mm,
uncertain 0.2-0.4 mm, present at or above 0.4 mm) under the
corrected means: 153825, 155419, 172642, 172644, 172739, 172859,
172912, 172955, 173045, 173250, 173507, 173701. Motion-class
labels were left as published and are flagged for re-check in the
table's correction note, because the "present" call also requires
judgement about gravity leakage at that rotation level:

- 172859 (0.353 mm, now uncertain, labeled rotation-only),
  172912, 172955, 173250, 173507 (0.429-0.543 mm, now above the
  0.4 mm present bar, labeled uncertain), 155419 (0.419 mm,
  labeled uncertain).
- The three high-rotation takes that cross 0.4 mm (172644,
  172739, 173045) stay uncertain under the table's
  gravity-leakage rule.
- 153825 and 173701 cross from negligible to uncertain but carry
  no translation verdict in their motion class; 172642 is the
  failed recording.

## Files changed

- `dataset-tools/stitch-map/rgbd_odometry.py` — `K_COLOR` fx
  386.251221 to 386.040802, fy 385.510895 to 385.300934, plus a
  comment. The only change in that file.
- `analysis-drafts/recording-feature-table.md` — 36 IMU appendix
  rows corrected, 4 prose fixes, correction note added. The 13
  newer rows are unchanged.

## Files in this folder

- `SUMMARY.md` — this file.
- `calib_080033_verify.json/.log`, `calib_085611_verify.json/.log`
  — bag calibration read back from each bag.
- `rgbd_odometry_before.py` — the script before the K fix.
- `vo_085611_fixed_poses.json`, `vo_085611_fixed_orthomosaic.png`,
  `vo_085611_fixed_trajectory.npy`, `vo_085611_fixed.log` — VO
  re-run with the fixed intrinsics.
- `vo_085611_fixed_v2.json`, `vo_085611_fixed_v2_lastwins.png`,
  `vo_085611_fixed_v2_avg.png`, `vo_085611_fixed_v2_avg_fill.png`,
  `vo_085611_fixed_v2_traj.png`, `vo_085611_fixed_render.log` —
  v2 render from the fixed-K poses.
- `vo_085611_ownK_poses.json`, `vo_085611_ownK_orthomosaic.png`,
  `vo_085611_ownK_trajectory.npy`, `vo_085611_ownK.log`,
  `run_vo_ownK.py` — extra VO run with 085611's own bag
  intrinsics (script file untouched for this run).
- `comparison_vo_085611.png`, `make_vo_comparison.py` —
  side-by-side of the three orthomosaics and the two v2 maps.
- `gap2_motion_repro/`, `gap2_motion_repro.log` — reproduction
  of the discrepancy with the table's own script.
- `gap2_motion_check/`, `gap2_motion_check.log` — scalar-script
  spot check of the vectorized recompute on 080033 and 173701.
- `gap2_variants.py/.log`, `gap2_variants2.py/.log`,
  `gap2_variants3.py/.log` — integration-variant tests.
- `gap2_full.py`, `gap2_full.log`, `gap2_full_recompute.json` —
  all-49-take recompute under both methods.
- `correct_table.py` — the script that applied the table
  correction. `recording-feature-table_before.md` — the table
  before the correction.
