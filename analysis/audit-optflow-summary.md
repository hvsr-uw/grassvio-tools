# Optical-flow consistency check with held-out frames

Date: 2026-11-09. This answers the Nov 5 question: for feature detection,
at what frequency is it performed (every frame pair, or less), and should
some frames be held out for a reliability check based on optical flow
consistency? Script: `optflow_check.py` in this directory. Bags read
read-only, resolved by exact filename stamp. Plots: `plot_optflow.py`;
every plot below was opened and judged by eye before this summary was
written.

## Verdict: works, but only at the window scale, not per frame step

Holding frames out for an optical-flow check works, and it works
strongly — provided the consistency metric is measured over the held-out
window (25 frames), not per frame step. Three findings, in order of
strength:

1. The 25-frame flow-vs-rigid residual separates the non-rigid-signal
   take from both controls by about 100x: median 6.0–12.4 px per window
   on 173701 from frame 100 on, against 0.05–0.13 px on 102631c and
   0.02–0.07 px on 085611. The same take and the same second half the
   SIFT model flags, found from flow alone.
2. SIFT matching across the 25-frame hold-out collapses exactly where
   the signal appears: on 173701 the good SIFT matches between anchors
   fall from 2,803 (frames 0–25) to 67–166 per window after frame 100,
   while flow keeps about 1,900 of 2,000 points alive over the same
   windows. On the controls the match count stays at 1,499–1,641
   (102631c) and 2,347–4,199 (085611) all the way through.
3. Forward-backward error rises about 5x in the second half of 173701
   (window medians 0.12–0.30 px, against 0.03–0.06 px early and
   0.004–0.049 px on the controls). A real signal, but a weaker one.

Two things do not work as reliability checks, and both are measured,
not assumed:

- The per-step flow HMM, with the SIFT model's parameters unchanged,
  separates the takes far less than the SIFT HMM does: fraction above
  0.8 is 0.834 on 173701 against 0.999–1.000 on the controls, while the
  SIFT HMM gives 0.472 against 0.975–0.989. Per-frame-step flow
  residuals are small in absolute terms (pooled median 0.228 px on
  173701), so the filter mostly scores tracks reliable. The signal
  lives in the accumulation over the window, and a per-step filter
  dilutes it. The existing sigmas (0.5 / 3.0 px) were calibrated on
  stride-5 SIFT residuals and are the wrong scale for per-frame flow
  residuals.
- Flow-vs-SIFT agreement error is floored by SIFT's own repeatability
  on grass: median 1.37–1.49 px on both clean controls, so it cannot
  register anything below about 1.4 px. On 173701 it rises to 2.2–3.0 px
  in the second half, but on only 9–31 agreeing points per window, so
  it is noisy. It confirms the direction; it is not a clean
  discriminator.

The premise the reliability model needed is confirmed: optical flow
tracks grass features for the whole take. Median flow track length is
123 frames on 102631c (124 frames total), 253 on 173701 (254 total),
and 385 on 085611 (386 total), against a SIFT re-detection median of
1 pair on every take. The Markov part of the reliability model now has
long tracks to accumulate over — the tracking problem is solved; what
remains is feeding the filter observations at the right scale (one
residual per 25-frame window per track, not one per frame step). That
variant is designed here, not run.

## Method

Detection happens only at anchor frames, every 25 frames. The 24
frames between anchors are held out from detection and used only for
tracking. Features are seeded at frame 0 and tracked frame to frame
with Lucas-Kanade flow through the whole take; when fewer than 500
points are alive, fresh points are seeded at the current frame (new
track ids, lengths counted from there). At each anchor, three checks
run on the points alive at the anchor:

- Forward-backward error: re-track the anchor's points forward frame
  by frame to the next anchor, then back frame by frame to the start;
  the error is the distance between the returned point and the start.
- Flow-vs-SIFT agreement: SIFT is detected independently at both
  anchors and matched (Lowe 0.75). For a flow point whose start sits
  within 2 px of a matched SIFT keypoint, the flow endpoint is compared
  with the matched SIFT point at the next anchor. Flow predicts where
  the feature should be; SIFT measures where it is.
- Flow-vs-rigid residual, at two scales: per frame step, a homography
  is fitted with RANSAC (3.0 px) to that step's flow correspondences
  and each point's residual is recorded (the same residual definition
  the reliability model uses); per 25-frame window, one homography is
  fitted to the window's flow start-to-end correspondences and the
  residual is the endpoint minus the rigid prediction.

Parameters, all fixed before running, nothing tuned per take:

- Seed: `goodFeaturesToTrack`, maxCorners 2000, qualityLevel 0.01,
  minDistance 7 px, blockSize 7.
- Tracker: `calcOpticalFlowPyrLK`, winSize (21, 21), maxLevel 3,
  termination 30 iterations or 0.01 epsilon.
- Anchor spacing: 25 frames. Re-seed threshold: 500 alive points.
- SIFT: nfeatures 10000, Lowe 0.75, RANSAC 3.0 px.
- Reliability filter parameters copied unchanged from the SIFT model:
  prior 0.5, P(R|R) 0.95, P(U|R) 0.05, P(R|U) 0.30, P(U|U) 0.70,
  half-normal likelihoods sigma_R 0.5 px, sigma_U 3.0 px.

Takes, from the feature-table labels (the held-out labels, not used
anywhere in the method): 20261026_173701 (still, non-rigid signal in
the second half), 20260926_102631c (still, wind-free control), and
20261106_085611 (still, clean illumination oscillation on a fixed
view). The SIFT baselines below are the residual-only reliability
model on the same takes: 173701 and 085611 from the earlier
reliability-model summaries, 102631c re-run here with the same script
and settings for this comparison.

## Per-take numbers

Track length is frames survived; the SIFT baseline length is in
stride-5 pairs. Per-step loss is the fraction of alive points lost in
one frame step.

| take | frames | flow tracks | median flow length | frac >= 25 frames | frac >= 50 frames | per-step loss (median / mean) |
|---|---|---|---|---|---|---|
| 20260926_102631c | 124 | 2,000 | 123 frames | 0.994 | 0.985 | 0.000 / 0.00017 |
| 20261026_173701 | 254 | 2,000 | 253 frames | 1.000 | 0.965 | 0.00067 / 0.00134 |
| 20261106_085611 | 386 | 2,000 | 385 frames | 1.000 | 1.000 | 0.000 / 0.00008 |

Flow tracking does not fail on grass. On all three takes the median
track spans the whole take, and the median per-step loss is 0.07% at
worst. The only re-seeding happens on 173701 in the second half
(3.5% of tracks shorter than 50 frames), where the grass motion is
strongest.

| take | flow rigid residual, per step (median / p90) | frac > 1 px | per-step displacement (median / p90) | flow HMM median rel. | flow HMM frac > 0.8 | flow HMM frac < 0.2 | SIFT HMM frac > 0.8 (baseline) |
|---|---|---|---|---|---|---|---|
| 20260926_102631c | 0.048 / 0.089 px | 0.00002 | 0.566 / 1.095 px | 0.990 | 1.000 | 0.001 | 0.975 |
| 20261026_173701 | 0.228 / 0.778 px | 0.052 | 0.893 / 2.089 px | 0.980 | 0.834 | 0.089 | 0.472 |
| 20261106_085611 | 0.014 / 0.033 px | 0.0008 | 0.015 / 0.037 px | 0.990 | 0.999 | 0.002 | 0.989 |

The per-step flow residual is elevated on 173701 (0.228 px vs 0.048
and 0.014 px), and the first-half / second-half split of the per-step
median residual is 0.159 px vs 0.386 px, so the time pattern is there
— but with sigma_R 0.5 px the filter still calls 83% of 173701 tracks
reliable, against 47% for the SIFT HMM. Separation, measured as the
clean-take mean fraction above 0.8 minus the 173701 fraction: flow
HMM 0.165, SIFT HMM 0.510. The flow HMM is weaker, not stronger, at
the per-step scale. Note also that the flow residual is systematically
smaller than the SIFT residual on the same take (0.048 vs 0.201 px
median on 102631c): flow tracks move smoothly, so their rigid-fit
residuals sit below SIFT's detection-noise floor. The two residual
scales are not interchangeable.

### Window-scale numbers (the check that works)

| take | 25-frame window residual, median per window (px) | SIFT anchor matches per window | forward-backward error, window medians (px) | flow-vs-SIFT agreement, median (px) |
|---|---|---|---|---|
| 20260926_102631c | 0.13, 0.07, 0.07, 0.05 | 1,499–1,641 | 0.042–0.049 | 1.37–1.49 |
| 20261026_173701 | 0.32, 2.19, 3.37, 2.41, 10.56, 9.71, 10.45, 8.02, 12.36, 5.96 | 2,803, 570, 540, 626, 79, 69, 92, 113, 67, 166 | 0.031–0.064, then 0.12–0.30 | 1.37–1.69, then 1.46–3.01 |
| 20261106_085611 | 0.02–0.07 | 2,347–4,199 | 0.004–0.042 | 1.37–1.45 |

On 173701 the window residual is already 2–3 px by frames 25–100 and
6–12 px from frame 100 on (p90 up to 49 px in window 200–225),
against a 0.13 px maximum on 102631c and 0.07 px on 085611. The SIFT
anchor-match collapse and the forward-backward rise follow the same
time course: window 0 looks like a control, the signal is present but
smaller in windows 25–100, and it is large after frame 100. That
matches the feature table's note independently (first pairs at control
level, then residuals rise and stay up).

Two cautions on reading these numbers:

- Flow displacement over 25 frames does NOT separate rigid camera
  motion from grass motion on its own: on 173701 it is 13–39 px from
  window 25 on, but the take has a slow gyro drift (mean 0.063 rad/s)
  and 102631c shows 16.4 px of displacement in its first window from
  camera settling. The window residual separates them because the
  homography absorbs the rigid part; the displacement alone does not.
- The SIFT match count varies on clean takes too (2,347–4,199 on
  085611 as the illumination oscillates), so only a collapse to the
  ~100 level is a signal, not small variations.

## Does the flow check flag the same features the HMM flags?

At take level and at window level, yes: the same take (173701) and the
same second half, with the window residual giving the sharpest
separation of any metric tried so far on this take. At per-feature
level, not tested in the same terms: flow tracks and SIFT tracks are
different point sets (different seeding), so there is no per-feature
identity to compare, and the flow HMM's per-feature scores are weaker
than the SIFT HMM's (8.9% of tracks below 0.2, against 32.2% for the
SIFT HMM). What the flow check adds per feature is the window-scale
residual distribution: in 173701's second-half windows the median
point has a 6–12 px window residual, so most points in those windows
are individually inconsistent with rigid motion — the population
splits the same way the SIFT HMM's bimodal histogram does, but at a
scale the per-step filter cannot see.

## What this answers, and what is still unproven

- Detection frequency: SIFT currently runs on every stride-5 pair.
  This test shows detection can drop to every 25th frame with flow
  filling the gaps: 2,000 seeded points survive the whole take on all
  three takes, and the held-out frames give a reliability check at
  every anchor.
- The reliability check itself: the window-scale flow-vs-rigid
  residual, the SIFT anchor-match count, and the forward-backward
  error each flag 173701's second half. The per-step HMM and the
  flow-vs-SIFT agreement do not add separation beyond what the SIFT
  HMM already gives.
- Wind vs parallax: unchanged from the earlier work. The check flags
  features that do not follow the rigid motion. On 173701 that signal
  is labelled "non-rigid signal, not separated from parallax" in the
  feature table, and this check does not separate the two either. It
  detects non-rigidity, not wind.
- Only three takes were tested, all still. On a moving take the
  homography still absorbs rigid camera motion (that is what the SIFT
  version relies on), but flow tracking under fast rotation was not
  tested here.
- The window-scale HMM variant — one residual observation per
  25-frame window per track, with sigmas recalibrated to the
  window-residual scale — is the direct next step. It is designed,
  not run: the window residuals here (control median 0.02–0.13 px,
  signal median 6–12 px) suggest sigmas near 0.5 px and 5 px would
  separate them, but those numbers are a guess until fitted.

## Files

- `optflow_check.py` — the analysis script (method above, in full).
- `20260926_102631c_summary.json`, `20261026_173701_summary.json`,
  `20261106_085611_summary.json` — per-take summaries, including the
  per-window tables and the per-step residual time series.
- `20260926_102631c_tracks.json`, `20261026_173701_tracks.json`,
  `20261106_085611_tracks.json` — per-track lengths, median residuals,
  and flow-HMM reliability.
- `20260926_102631c_windows.json`, `20261026_173701_windows.json`,
  `20261106_085611_windows.json` — per-anchor-window consistency
  numbers.
- `sift_baseline_102631c_tracks.json` (+ `_hist.png`, `_curve.png`) —
  the SIFT reliability model re-run on 102631c for the baseline column
  above (median track length 1 pair, median reliability 0.909, fraction
  above 0.8 0.975, median residual 0.201 px).
- `fig_track_lengths.png` — flow track-length histograms; nearly every
  track spans the whole take on all three takes.
- `fig_residual_time.png` — per-step median rigid residual and the
  25-frame window residual along each take. Note the y-axes differ
  (0.13 px max on 102631c, 12 px on 173701, 0.07 px on 085611): the
  signal is 100x the control level. The small periodic wiggle in
  085611's per-step residual (0.011–0.026 px) follows the illumination
  oscillation and is negligible at this scale.
- `fig_windows.png` — per-window flow displacement, forward-backward
  error, flow-vs-SIFT agreement, and SIFT anchor-match counts.
- `fig_hmm.png` — flow-HMM final-reliability histograms; 173701 keeps
  a small peak near 0 (8.9% below 0.2) but most tracks stay near 1.
- `fig_residual_hist.png` — per-track median rigid-residual
  histograms (flow scale: medians 0.048 / 0.228 / 0.014 px).
- `plot_optflow.py` — the plotting script.
- `*.log` — run logs for each take.
