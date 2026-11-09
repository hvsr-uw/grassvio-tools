# Window-scale flow HMM for the reliability model

Date: 2026-11-09. This runs the variant the optical-flow summary
designed but did not run: one flow-vs-rigid residual observation per
25-frame window per track, with the filter sigmas fitted from the
window residuals themselves. Script: `window_hmm.py` in this
directory. Bags read read-only, resolved by exact filename stamp.
Plots: `plot_window_hmm.py`; every plot below was opened and judged
by eye before this summary was written.

## Verdict: does not beat the SIFT HMM as a general reliability model

The window HMM separates the signal take far more strongly than the
SIFT HMM does (fraction above 0.8 is 0.008 on 173701, against 0.472
for the SIFT HMM, and 97.8% of tracks score below 0.2, against 32.2%
for SIFT). But it buys that gap by mis-scoring a clean control take:
on the rotating take 080033 the fraction above 0.8 falls to 0.637,
against 0.975 for the SIFT HMM, and 28.0% of its tracks score below
0.2, against 0.2% for SIFT. Measured as clean-take mean minus
signal, the gap is wider for the window HMM (0.871 against 0.507
for SIFT), but the clean-take minimum falls from 0.975 to 0.637.
By the same standard the IMU variant was judged against, a wider
gap bought by scoring a clean take as unreliable is not better
separation.

Two qualifications keep this honest. First, on still takes alone
the window HMM beats the SIFT HMM decisively: the still-control
mean above 0.8 is 0.960 against 0.008 on the signal take (a gap of
0.952), while the SIFT HMM gives 0.981 against 0.472 (a gap of
0.509). The failure is specific to rotation, not to the window
scale. Second, the failure is a sigma problem, not a window
problem: with 0.5 / 5.0 px sigmas the window HMM's gap is 0.917
with every control above 0.93, which beats the SIFT HMM on the gap
while keeping the controls intact. The fitted sigma_R of 0.128 px
is tighter than the rigid-misfit scale on the rotating take, and
over 17 windows the filter compounds mildly elevated residuals
into confident unreliability. The next step is a sigma fit that
accommodates the hardest control take, specified before running,
not tuned after seeing these results.

## Method

Detection happens only at anchor frames, every 25 frames. Features
are seeded at frame 0 and tracked frame to frame with Lucas-Kanade
flow through the whole take; the 24 frames between anchors are held
out from detection and used only for tracking. For each full 25-frame
window, one homography is fitted to the window's flow start-to-end
correspondences, and each point's residual under that fit is its one
observation for that window. A forward filter turns each track's
window-residual history into P(reliable). The final partial window
(fewer than 25 frames) is dropped. A track contributes an observation
only for windows it spans in full; tracks with no full window get no
score and are counted separately below.

Parameters, all fixed before running except the fitted sigmas:
- Seed: `goodFeaturesToTrack`, maxCorners 2000, qualityLevel 0.01,
  minDistance 7 px, blockSize 7.
- Tracker: `calcOpticalFlowPyrLK`, winSize (21, 21), maxLevel 3,
  termination 30 iterations or 0.01 epsilon.
- Anchor spacing: 25 frames. Re-seed threshold: 500 alive points.
- Every homography fit (per step and per window) uses RANSAC at
  3.0 px. The window homography is fitted over the points alive at
  both ends of the window, and the same points are then scored
  against it, as in the prototype and the optical-flow check.
- Prior P(reliable) = 0.5; transition P(R|R) = 0.95, P(U|R) = 0.05,
  P(R|U) = 0.30, P(U|U) = 0.70, the same as the prototype.
- Likelihoods are half-normal on the residual.

The per-step flow HMM is run in the same pass from the same tracks
with the prototype sigmas (0.5 / 3.0 px), so its column in the table
below is a same-tracks comparison. It reproduces the optical-flow
summary's numbers exactly on the three overlapping takes (fraction
above 0.8 of 0.9995 / 0.834 / 0.9985 on 102631c / 173701 / 085611),
and the per-window residual medians match that summary's window
table within about 0.1 to 3.2 px (the earlier run re-seeded fresh
points per window; this run uses continuously tracked points).

## Sigma fitting, and the fitted values

The sigmas are fitted from the window residuals, not taken from the
0.5 / 5.0 px starting guess in the optical-flow summary. The fitting
procedure, fixed before the filter was run:

- Pool all window residuals (one per track per window) from the four
  control takes (174745, 080033, 085611, 102631c): 67,800 residuals,
  median 0.086 px, p90 0.422 px. Pool all window residuals from the
  signal take (173701): 16,445 residuals, median 4.166 px, p90
  30.13 px. The late windows of the signal take alone (window 4 on,
  frames 100-250) have median 8.72 px over 9,032 residuals.
- For a half-normal distribution the median is sigma times 0.6745,
  so sigma is the pooled median divided by 0.6745. That gives
  sigma_R = 0.128 px and sigma_U = 6.177 px. These are the primary
  sigmas.
- The maximum-likelihood half-normal scale, sqrt(mean r squared),
  is reported as a sensitivity check: sigma_R = 3.645 px and
  sigma_U = 16.142 px. The median estimator is primary because the
  control pool has a heavy tail (p99.9 is 61.2 px, maximum 187.1 px,
  from a few flow blow-ups), and the mean of squared residuals is
  dominated by those outliers. A single wild residual moves the MLE
  scale; it barely moves the median.

The filter is run three times per take: with the median-fitted
sigmas (primary), with the MLE sigmas, and with the 0.5 / 5.0 px
guess, so the verdict does not hinge on one fitting choice. The
per-take pooled window-residual medians show why one sigma_R is
stretched across takes: 0.043 px on 085611, 0.081 px on 102631c,
0.106 px on 174745, 0.235 px on 080033, and 4.166 px on 173701.
The rotating take's rigid misfit is about five times the
illumination take's, and the pooled-median fit sits below it.

## Per-take comparison table

Fractions are over tracks with at least one full-window observation
(the scored tracks); the unscored count is the tracks that died
before completing one 25-frame window. Median reliability is the
median over scored tracks of final P(reliable). SIFT HMM numbers
are the residual-only model from the earlier summaries, read from
their track files. Per-step flow HMM numbers are from this run.

| take | motion / label | frames | flow tracks | scored | windows | window HMM median | window HMM > 0.8 | window HMM < 0.2 | SIFT HMM median | SIFT HMM > 0.8 | SIFT HMM < 0.2 | per-step flow > 0.8 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 20261026_173701 | still, non-rigid signal | 254 | 2,000 | 2,000 | 10 | 0.000 | 0.008 | 0.978 | 0.759 | 0.472 | 0.322 | 0.834 |
| 20261030_174745 | still, control | 200 | 2,000 | 1,720 | 7 | 0.999 | 0.902 | 0.078 | 0.909 | 0.978 | 0.002 | 0.973 |
| 20261007_080033 | rotation, control | 441 | 3,477 | 3,190 | 17 | 0.931 | 0.637 | 0.280 | 0.909 | 0.975 | 0.002 | 0.876 |
| 20261106_085611 | still, illumination, control | 386 | 2,000 | 2,000 | 15 | 0.999 | 0.997 | 0.003 | 0.979 | 0.989 | 0.002 | 0.999 |
| 20260926_102631c | still, wind-free control | 124 | 2,000 | 1,988 | 4 | 0.999 | 0.981 | 0.016 | 0.909 | 0.975 | 0.003 | 1.000 |

Median flow track length is 253 frames on 173701, 199 on 174745,
156 on 080033, 385 on 085611, and 123 on 102631c, against a SIFT
median of 1 pair on every take except 085611 (2 pairs). The Markov
part finally has long tracks to accumulate over: the median scored
track contributes 10 window observations on 173701, 7 on 174745,
5 on 080033, 15 on 085611, and 4 on 102631c. The 3,477 tracks on
080033 are re-seeding under rotation; the other takes keep their
original 2,000 points alive for the whole take.

## Separation by the gap measure

Gap is the clean-take mean fraction above 0.8 minus the 173701
fraction. Clean minimum is the lowest control take's fraction.

| method | signal 173701 | clean mean | clean minimum | gap | still-only gap |
|---|---|---|---|---|---|
| window HMM, fitted sigmas 0.128 / 6.18 px | 0.008 | 0.879 | 0.637 (080033) | 0.871 | 0.952 |
| window HMM, 0.5 / 5.0 px sigmas | 0.057 | 0.974 | 0.934 (080033) | 0.917 | 0.930 |
| window HMM, MLE sigmas 3.64 / 16.14 px | 0.424 | 0.997 | 0.990 (085611) | 0.573 | 0.573 |
| SIFT HMM | 0.472 | 0.980 | 0.975 (080033) | 0.507 | 0.509 |
| per-step flow HMM | 0.834 | 0.962 | 0.876 (080033) | 0.128 | 0.156 |

The fitted window HMM has the second-widest gap and the worst
clean minimum. The guess-sigma window HMM has the widest gap and
keeps every control above 0.93. The MLE-sigma window HMM separates
less than the fitted one: with sigma_R at 3.64 px, even 4 px
residuals look reliable, so 42% of the signal take's tracks stay
above 0.8. The outcome depends on the fitting method, which is
itself a finding: the window scale separates, but the sigma fit
decides whether the controls survive it.

A plain threshold on the same window residuals (fraction of tracks
with median window residual under 0.5 px) gives 0.018 on 173701
against 0.938 / 0.856 / 0.990 / 0.985 on the controls: a gap of
0.924 with a clean minimum of 0.856. The threshold beats the
fitted HMM on both numbers. The HMM's compounding is the specific
weakness: 17 windows of mildly elevated residuals drive belief to
zero where a single median does not.

## What the plots show (all opened and judged by eye)

- Histograms (`fig_hist.png`): 173701 is a single pile at 0.
  085611 and 102631c are single piles at 1.0. 174745 is a pile at
  1.0 with a small pile at 0 (134 tracks). 080033 is bimodal with
  a large pile at 0 (about 800 tracks) and a large pile at 0.95-1.0
  and a smear between them: a clean take split into two populations.
- Belief curves (`fig_curves.png`): on 173701 the window HMM mean
  belief falls from 0.57 after the first window to 0.07 after the
  second and stays near 0.01 for the rest of the take, while the
  SIFT HMM falls only to about 0.3 mid-take and oscillates between
  0.25 and 0.7, and the per-step flow HMM stays at 0.8-0.99. On
  174745 the window HMM starts at 0.60 after the settling first
  window and recovers to 0.998. On 080033 it falls to 0.64 by
  frame 125, recovers to 0.99 by frame 250, then collapses to
  0.15-0.18 at frames 375-400 and recovers to 0.93 in the last
  window. On 085611 and 102631c all three curves sit flat at
  0.94-1.0.
- Window residuals (`fig_winresid.png`, log scale): the 173701
  medians sit at 2-12 px from window 1 on, above the fitted
  sigma_U line for the late windows. The 080033 medians sit at
  0.1-0.94 px, mostly above the fitted sigma_R line at 0.128 px:
  the rotation take's ordinary residuals live above the scale the
  filter calls reliable. The still controls sit at or below the
  sigma_R line except for their first windows (0.305 px on 174745,
  0.132 px on 102631c, camera settling).
- Comparison bars (`fig_comparison.png`): the fitted window HMM
  is near zero on 173701 and 0.64 on 080033 while every other
  method sits at 0.88-1.0 there; the guess-sigma window HMM is
  near zero on 173701 and above 0.93 on every control.

## Why the rotating take fails

Three measurements pin the mechanism down. First, the scale
mismatch: the pooled window-residual median on 080033 is 0.235 px
against the fitted sigma_R of 0.128 px, and 21% of its window
observations are above 0.5 px. A 0.3 px window residual is ordinary
rigid misfit on this take, but the filter reads it as evidence of
unreliability. Second, the compounding: the low-belief tracks on
080033 have a median of 5 window observations each, and their last
observations concentrate in windows 3-4 and 11-14, the stretches
where the per-window residual medians run 0.3-0.94 px. Seventeen
windows give the filter seventeen chances to compound. Third, the
same stretch is genuinely harder for every method, just less so:
the SIFT HMM belief curve on 080033 dips to 0.661 at frame 360,
against about 0.95 elsewhere, so the fast-rotation frames around
350-400 stress the rigid fit itself. The window HMM turns that
mild stress into confident unreliability because its sigma_R is
tighter than the misfit scale there.

The two still-control shortfalls have a different, smaller cause.
On 174745, 60% of the 134 tracks below 0.2 have exactly one window
observation, and 80 of them end in window 0, the settling window
with median residual 0.305 px: one bad first window kills a short
track before it can recover. The same pattern, smaller, on 102631c
(32 tracks below 0.2, 56% with one window, 18 ending in window 0).
These are short-track edge effects, not a take-level failure.

## Honest limits

- Only one non-rigid-signal take exists in the test set, so every
  separation claim rests on one take against four clean ones.
- The sigmas are fitted on the same five takes they are evaluated
  on: sigma_U from 173701 itself, sigma_R from the four controls.
  There is no held-out take. The SIFT comparison is still
  meaningful because the SIFT sigmas were likewise set from these
  takes (sigma_R 0.5 px from 102631c), but a fresh take would need
  the sigmas fixed in advance. The guess-sigma run is the closest
  to a pre-fixed scale, and it is the run that beats SIFT.
- Wind vs parallax is still not separated: the check flags features
  that do not follow the rigid motion, and on 173701 that signal is
  labelled non-rigid signal, not separated from parallax, in the
  feature table.
- The window homography is fitted in-sample over the points it
  scores, with a RANSAC threshold of 3.0 px even in windows whose
  median residual is 10 px, so the fit is a compromise over the
  surviving consensus set. That is the same construction the
  optical-flow check used, and the two runs agree on the window
  medians, but it is not an independent rigid reference.

## Files

All under `analysis-drafts/full-audit-20261109/window-hmm/`:

- SUMMARY.md — this file.
- `window_hmm.py` — the staged script (`collect` tracks one take
  and writes its cache; `fit` pools the residuals, fits the sigmas,
  and runs the filter). The prototype, the IMU variant, and the
  optical-flow script are untouched.
- `run_all.sh`, `run_all.log` — the collect run over the five
  takes, each finishing in under a minute.
- `*_cache.json` — per-take window residuals, per-step residuals,
  and the per-step flow HMM from the collect step.
- `*_tracks.json`, `*_summary.json` — per-take window-HMM results
  under the fitted, MLE, and guess sigmas, plus belief curves.
- `fit.json` — the fitted sigmas, pool sizes, and pool medians.
- `comparison.json` — the aggregated comparison table.
- `aggregate.py` — builds the comparison table from the JSONs.
- `plot_window_hmm.py` — the plotting script.
- `fig_hist.png`, `fig_curves.png`, `fig_winresid.png`,
  `fig_comparison.png` — the figures, all judged by eye.
- `*.log` — per-take run logs, plus `fit.log`.
