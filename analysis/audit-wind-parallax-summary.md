# Wind vs parallax on 20261026_173701

Date: 2026-11-09. Take 20261026_173701 (254 frames, 8.4 s, still take)
against the wind-free control 20260926_102631c (124 frames, 4.1 s).
Question: does the non-rigid signal on 173701 track camera translation
(parallax over 3D foliage) or persist while the camera is still (wind or
another non-translation cause)?

## Verdict

The signal persists while the camera is still. It is not explained by
translation parallax. Wind is the leading explanation for the signal's
onset and floor, not a proof: the peak amplitude in the second half is
partly driven by increased camera rotation, and sub-millimetre
translations below the proxy's noise floor cannot be excluded. The
take's label stays "non-rigid signal, persists while still" — the
"not separated from parallax" qualifier is removed for the onset and
the floor, and kept for the second-half peak amplitude.

Evidence in one line: 20 of the 50 pairs have gyro at or below the
control's median gyro and still show a residual median of 0.60 px
against a control median of 0.19 px; the signal switches on at about
0.8 s while gyro and proxy are both at control level.

## Method

Pairs: consecutive stride-5 chosen frames (0, 5, 10, ...), 50 pairs on
173701 and 24 on the control. Same pair construction as the
reliability-model prototype.

Residuals per pair: SIFT (10000), Lowe 0.75, homography by RANSAC at
3.0 px, median residual of all good matches after the fit. These are
the feature-table wind check's settings, applied to stride-5 pairs
instead of consecutive pairs. The control reproduces the table under
this change: pooled median 0.190 px and p90 0.470 px here, against
0.185 px and 0.458 px in the table for consecutive pairs.

Translation proxy per pair window: accelerometer demeaned by the take
mean, double-integrated from zero velocity over the window between the
two chosen frames (the feature-table method, applied to the 5-frame
window instead of one frame interval). Because the window is 5x longer
than the table's per-frame interval, absolute millimetre values are not
comparable to the table's per-interval numbers. The still/moving split
is therefore calibrated on the control take computed the same way, not
on the table's 0.2/0.4 mm bands.

Still/moving split (fixed before looking at 173701's residuals): a pair
counts as moving when its gyro mean exceeds 0.25 rad/s (the feature
table's rotation threshold) or its window proxy exceeds the control
take's p95 window proxy (3.23 mm). No gyro sample in either take
exceeds 0.25 rad/s, so the split is driven by the proxy alone; a
gyro-only split is reported alongside it because the proxy is noisy.

Gyro per pair window: mean and max gyro magnitude over the window.
Brightness per chosen frame: mean gray value, to check for an
illumination change. Correlations are Pearson and Spearman across the
50 pairs (24 on the control).

## Numbers

### Residuals

| | 173701 | control 102631c |
|---|---|---|
| pairs fit | 50 of 50 | 24 of 24 |
| pooled median / p90 (px) | 0.647 / 3.152 | 0.190 / 0.470 |
| median of per-pair medians (px) | 1.117 | 0.191 |
| first-half per-pair median (px) | 0.612 | 0.193 |
| second-half per-pair median (px) | 1.603 | 0.190 |
| per-pair median range (px) | 0.19–2.42 | 0.16–0.20 |

The control is flat at 0.19 px from the first pair to the last.
173701 starts at control level (pairs 1–4: 0.19, 0.21, 0.20, 0.24 px),
rises to 0.43–0.57 px by pairs 5–8, holds near 0.6–0.8 px through
pair 20, jumps to 1.3–2.0 px for pairs 21–33, dips to 0.80 px at
pairs 36–37, spikes to 2.23–2.42 px at pairs 40–42, and decays to
1.0–1.2 px at the end. The rise is episodic (two high episodes with a
dip between), not a single step.

Residuals grow with the pair gap on 173701 but not on the control:
pooled median 0.348 px at stride 1 (feature table) against 0.647 px
at stride 5 here, while the control is 0.185 px at stride 1 against
0.190 px at stride 5. Non-rigid motion accumulates over the 167 ms
gap on 173701; on the rigid control it does not.

### Motion channels

| | 173701 | control 102631c |
|---|---|---|
| gyro mean / median / p95 / max (rad/s) | 0.063 / 0.055 / 0.125 / 0.171 | 0.050 / 0.049 / 0.089 / 0.118 |
| fraction of gyro samples above 0.25 rad/s | 0.0 | 0.0 |
| pair gyro mean, first / second half (rad/s) | 0.048 / 0.078 | 0.056 / 0.045 |
| pair proxy mean / median / p95 / max (mm) | 4.02 / 3.69 / 7.41 / 13.38 | 1.80 / 1.66 / 3.23 / 3.28 |
| pair proxy mean, first / second half (mm) | 3.18 / 4.86 | 2.01 / 1.60 |

The take is "still" by the table's threshold (gyro mean below 0.10
rad/s, no sample above 0.25 rad/s), but it is not perfectly still at
fine resolution: pair gyro mean nearly doubles from the first half
(0.048 rad/s) to the second (0.078 rad/s), and the three highest
per-pair gyro values in the take (0.142, 0.125, 0.124 rad/s) are all
in the second half. The control's gyro falls slightly over its take.

### Moving vs still segments

Pre-registered split (gyro above 0.25 rad/s or proxy above the
control p95 of 3.23 mm): 31 moving pairs against 19 still pairs on
173701. Moving-segment per-pair median 1.59 px (p90 2.00 px);
still-segment per-pair median 0.67 px (p90 1.64 px). The still
segment sits 3.5x above the control median of 0.19 px. On the
control the same split gives 2 moving pairs against 22 still pairs,
both at 0.19 px — the split itself creates no difference there.

Gyro-only splits, which do not depend on the noisy proxy:

- 20 of 50 pairs have gyro at or below the control's median gyro
  (0.048 rad/s): residual median 0.60 px, range 0.19–1.82 px.
- 37 of 50 pairs have gyro inside the control's whole gyro range
  (at or below 0.077 rad/s): residual median 0.80 px.
- Second half only, 5 lowest-gyro pairs (0.028–0.051 rad/s):
  residual median 1.59 px; 5 highest-gyro pairs (0.108–0.142
  rad/s): residual median 2.23 px. Rotation adds amplitude on top
  of an already elevated floor; it does not create the floor.
- Onset pairs 5–8: residuals 0.43, 0.47, 0.49, 0.57 px with gyro
  0.046, 0.026, 0.014, 0.039 rad/s and proxy 2.9, 1.9, 1.5, 2.3 mm
  — gyro and proxy both at control level while residuals are
  2–3x control. Pairs 5–20 as a group: residual median 0.61 px,
  gyro median 0.044 rad/s, proxy median 2.96 mm (control proxy
  median 1.66 mm).

### Correlations (per-pair, 173701, n = 50)

| | Pearson | Spearman |
|---|---|---|
| residual vs translation proxy | 0.28 | 0.42 |
| residual vs gyro mean | 0.68 | 0.68 |
| residual vs time | 0.70 | 0.71 |
| gyro vs time | 0.67 | 0.71 |
| proxy vs time | 0.53 | 0.53 |
| brightness vs time | 0.87 | 0.89 |
| residual vs brightness | 0.57 | 0.68 |

After removing a linear time trend from both variables, the
residual–proxy correlation vanishes (-0.14) and the
residual–brightness correlation vanishes (-0.10), while the
residual–gyro correlation survives at 0.39. Within the first half
(n = 25) the residual–gyro correlation is 0.61 / 0.75 and the
residual–proxy correlation is 0.50 / 0.59; within the second half
the residual–gyro correlation is 0.47 / 0.32 and the
residual–proxy correlation is -0.20 / -0.17. The proxy link is a
shared time trend only. The gyro link is partly a shared trend
and partly real: in the second half it survives detrending at
0.63, and the three highest-gyro pairs are exactly the three
highest-residual pairs (pairs 40–42: gyro 0.142, 0.124,
0.125 rad/s; residuals 2.23, 2.42, 2.37 px).

On the control (n = 24) every correlation is small: residual vs
proxy -0.18 / -0.20, residual vs gyro 0.09 / -0.01, residual vs
time -0.30 / -0.30. Gyro and proxy variation of the same size as
on 173701 produces no residual change at all on the control.
Residuals are flat across the control's gyro tertiles (0.190,
0.196, 0.192 px) and proxy tertiles (0.191, 0.195, 0.186 px).
On 173701 the gyro tertiles read 0.58, 1.58, 1.52 px and the
proxy tertiles 0.61, 1.73, 1.52 px — but both tertile splits are
confounded with time, so the detrended numbers above are the ones
to trust.

### Supporting checks

- Match counts: 173701 falls from a mean of 1987 good matches per
  pair in the first half to 993 in the second; the control is flat
  near 1710. RANSAC inlier fraction falls from 0.90 to 0.75 on
  173701 and stays near 1.00 on the control. The inlier fraction
  dips episodically (to 0.58–0.65 near the residual peaks) and
  recovers to 0.92 between episodes.
- Brightness rises smoothly from 108 to 118 and back to 114 over
  the take. It is flat during the residual onset (pairs 1–10:
  108–109), and its correlation with residuals vanishes after
  detrending, so the brightening does not drive the signal.
- Sharpness (Laplacian variance) is 3602 on frame 0 and 4007–4160
  on frames 175–250: no motion blur in the second half. The match
  collapse is not blur.
- Raw pixel change does not track the residuals: mean absolute
  frame difference is 21.7 (calm pair, frames 0–5), 25.3 (early
  rise, frames 25–30), 38.7 (peak, frames 200–205) and 38.9 (dip,
  frames 170–175) while the corresponding residuals are 0.19,
  0.47, 2.42 and 0.80 px. The dip frames change as much as the
  peak frames in raw pixels but fit the rigid model far better:
  the peak is non-rigid deformation, not more motion overall.
- The scene is close-range 3D foliage (leafy branches, leaves at
  many depths), not flat grass. Frame 0 and frame 200 show the
  same viewpoint with the same branches in the same places while
  individual leaves have shifted — the camera did not move
  between them, the foliage did.

## Plots (all opened and judged by eye)

- fig_residual_time.png — per-pair median residual vs time for both
  takes. The control is a flat line at 0.19 px; 173701 leaves
  control level at about 0.8 s and never returns, with two high
  episodes and a dip near 6 s.
- fig_motion_time.png — residual, gyro and proxy vs time for
  173701 with the control overlaid. Gyro drifts up in the second
  half but stays far below the 0.25 rad/s threshold; the proxy is
  noisy (0.6–13.4 mm) with no clean shape and a single 13.4 mm
  spike on the last pair.
- fig_scatter.png — residual vs proxy and residual vs gyro. At a
  proxy of 3–4 mm, residuals span 0.2–2.4 px: the proxy does not
  separate low- from high-residual pairs. Against gyro there is a
  loose upward cloud with wide vertical spread at fixed gyro; the
  three highest residuals sit at the three highest gyros.
- fig_matches_time.png — match counts and inlier fraction vs
  time. The match collapse and the episodic inlier-fraction dips
  on 173701, flat on the control.
- diff_calm.png, diff_peak.png — absolute frame differences over
  5-frame gaps. Both are fine speckle from the foliage texture;
  the peak pair shows brighter localised patches (a branch of
  leaves, lower centre) on top of the same speckle. Frame 0 and
  frame 200 (frame_0.png, frame_200.png) show the same viewpoint
  with shifted leaves.

## Caveats and what would change the answer

1. The second-half peak is confounded by rotation. Gyro mean
   nearly doubles in the second half, the detrended gyro
   correlation is 0.63 there, and the top-3 gyro pairs are the
   top-3 residual pairs. One plausible mechanism is wind gusts
   shaking the camera as well as the foliage — wind driving both
   channels — but independent small camera motion cannot be
   ruled out from this data. Only the onset and the floor are
   cleanly separated from motion, not the peak amplitude.
2. The translation proxy cannot exclude sub-millimetre
   translations. The control's stride-5 proxy spans 0.7–3.3 mm
   with no residual effect, so a real translation of that size
   or smaller hides in the noise. The proxy elevation on 173701
   (mean 4.02 mm vs 1.80 mm on the control) could be higher
   accelerometer noise rather than translation: the per-frame
   proxy sits inside the documented still-take noise range.
   A depth-based check (median 3D displacement of matches, as
   in the IMU fusion script) would test small translations
   directly and is the clear next step.
3. The feature table's published per-frame proxy values do not
   reproduce. Re-running the table's own motion script on the
   same bag gives a per-frame proxy mean of 0.236 mm for 173701
   (table: 0.132 mm) and 0.142 mm for the control (table:
   0.084 mm); the median and max disagree the same way, and
   three other integration variants (inside-only samples,
   median demeaning, trapezoidal) all give about 0.23–0.25 mm.
   The gyro numbers reproduce exactly, and the table's accel
   column is the mean of per-sample magnitudes (9.7094) rather
   than the magnitude of the mean vector (9.7023), so the bag
   and the parsing agree — only the proxy column disagrees.
   This affects the table's absolute noise-floor claims, not
   this test: every comparison here is computed with one method
   on both takes.
4. No anemometer data exists for the recording, so "wind" is
   the leading explanation for a signal that persists while
   still, not a measurement of wind. The alternatives that stay
   open: sub-millimetre translation parallax below the proxy
   floor, camera micro-vibration below gyro resolution, and
   wind shaking the camera. What would settle it: a repeat
   still take with no second-half gyro rise, or the depth-based
   translation check in (2).

## Files

- wind_parallax_173701.py — analysis script (stride-5 residuals,
  per-pair proxy and gyro, correlations, still/moving split).
- plot_wind_parallax.py — plotting script for the four figures.
- summary.json — all summary numbers.
- 20261026_173701_pairs.json, 20260926_102631c_pairs.json —
  per-pair records (residual, proxy, gyro, brightness, match
  counts) plus take-level cross-checks.
- run1.log — full run log.
- fig_residual_time.png, fig_motion_time.png, fig_scatter.png,
  fig_matches_time.png, diff_calm.png, diff_peak.png,
  diff_early_rise.png, diff_dip.png, frame_0.png, frame_25.png,
  frame_170.png, frame_200.png — plots and frame checks.
