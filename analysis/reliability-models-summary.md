# Reliability models: a per-feature Bayesian + Markov model, prototyped

Date: 2026-11-06. This answers the Oct 30 "All" TODO: "Look into
reliability models: Bayesian model, Markov chain etc". Don is separately
building an empirical reliability score; this file is the models side,
not a duplicate of his work. Script:
dataset-tools/reliability/reliability_hmm.py (new). Bags read read-only.

## The model, in plain words

Every tracked feature is either reliable (it sits on something rigid,
so it moves exactly the way the camera moved) or unreliable (wind-blown
grass, a bad match, something moving on its own). We never see the
state directly. Each frame pair gives one piece of evidence: the
feature's reprojection residual under the pairwise rigid fit, in
pixels. Reliable features cluster near zero residual; unreliable ones
spread widely. A first-order Markov chain says the state usually stays
what it was, but can change. A forward filter (exact Bayesian
inference) turns the residual history into P(reliable) for each
feature. That number is the reliability score.

Parameters, all written down so they can be argued with:
- prior P(reliable) = 0.5 (no prior information, as in BANSAC).
- transition: P(R|R) = 0.95, P(U|R) = 0.05, P(R|U) = 0.30,
  P(U|U) = 0.70. Reliable usually stays reliable; unreliable can
  recover, for example a leaf that settles.
- likelihoods are half-normal on the residual r: reliable with
  sigma = 0.5 px, unreliable with sigma = 3.0 px. Sigma 0.5 comes
  from the wind-free control 102631c: pooled median residual
  0.185 px, p90 0.458 px. Sigma 3.0 is a broad spread for features
  that do not follow the rigid motion.
- track length is not a separate input: a track's evidence count is
  its length, so long consistent tracks end near P = 1 and long
  inconsistent tracks end near 0. Short tracks stay near the prior.
- the take-level wind label from the feature table is deliberately
  NOT used as a prior: it is the held-out label the model is tested
  against, so using it would be circular.

## What was read

- BANSAC (Piedade and Miraldo, ICCV 2023): a dynamic Bayesian network
  that updates each data point's inlier probability over RANSAC
  iterations, then samples by those probabilities. Method section read
  in full. Two ideas taken directly: per-point inlier probabilities
  updated by exact forward inference, and the 1st-order Markov
  assumption (their ablation found 1st order most accurate). Our model
  moves the same update from RANSAC iterations to video frames: the
  "iterations" are consecutive frame pairs, the evidence is the
  reprojection residual instead of an inlier/outlier classification.
- DSS-SLAM (Dynamic Semantic SLAM, hierarchical multi-sensor fusion):
  uses a hidden Markov model for the temporal dynamics of the
  environment, with Viterbi estimation of the state sequence. Read via
  detailed search text, not the full paper. It supports the same
  pattern: a latent state evolving over time, observations per frame.
- AIS integrity monitoring (Sensors 2021): a two-state Markov-chain
  integrity model (reliable / unreliable) for navigation data, with
  EKF process models as the measurement comparison. Read via detailed
  search text. It supports the two-state framing used here.

## Results on three real takes

Method: SIFT (10000) + Lowe 0.75 on consecutive stride-5 frame pairs,
homography by RANSAC at 3.0 px, residual per match, tracks chained by
keypoint identity, forward filter per track. Takes picked from the
feature table labels: 174745 (still, no wind signal — control), 173701
(still, non-rigid signal in the second half), 080033 (rotation, genuine
loop, no wind signal).

| take | label (feature table) | tracks | median track length | median final reliability | frac > 0.8 | frac < 0.2 | median residual |
|---|---|---|---|---|---|---|---|
| 20261030_174745 | still, no wind signal | 32,030 | 1 | 0.909 | 0.978 | 0.002 | 0.204 px |
| 20261026_173701 | still, non-rigid signal | 35,456 | 1 | 0.759 | 0.472 | 0.322 | 0.791 px |
| 20261007_080033 | rotation, no wind signal | 85,903 | 1 | 0.909 | 0.975 | 0.002 | 0.209 px |

## Does it separate reliable from unreliable? Yes, in two ways

1. Across takes. The wind-signal take (173701) stands apart: only 47%
   of its tracks score above 0.8, against 98% on both the still
   control and the rotating take, and 32% score below 0.2, against
   0.2% on the controls. The rotating take looks identical to the
   still control (median reliability 0.909 on both), which is the
   right behaviour: the rigid fit absorbs camera motion, so rotation
   alone does not make features look unreliable. The model responds
   to non-rigid signal, not to camera motion.
2. Over time within a take. On 173701 the mean belief starts at
   control level (0.89-0.95 for the first pairs), falls to about 0.3
   mid-take, and stays low, with a partial recovery to about 0.5 at
   the end (see 173701_curve.png). That matches the feature table's
   note independently: first pairs at control level, then residuals
   rise and stay up. On both control takes the curve is flat at
   0.94-0.95 from start to finish. The model finds WHEN a take goes
   unreliable, not just that it did.

The 173701 reliability histogram is bimodal: a large peak near 0 and
a large peak near 1 (see 173701_hist.png). The feature population
splits into two groups, which is what a reliability score is for.

One caveat on the headline number. The 47 percent depends on the
likelihood choice. Re-running the same tracks with a Rayleigh
likelihood instead of the half-normal moves 173701 to 64 percent or
26 percent depending on how the scale is set, while the ordering
stays the same, 173701 far below both controls, and the curve shape
holds. The 25 frame window is 0.834 seconds, and the verdict comes
about 0.83 seconds late but does not look ahead past the window.
The 0.8 threshold has no recorded justification, it is a chosen
headline value, and the gap stays about the same at 0.7 and 0.9.

## Does the HMM add anything over a plain threshold?

Partly. A plain rule (median residual under 0.5 px) gives 0.926 /
0.344 / 0.921 reliable on the three takes — it separates them too,
and is stricter than the HMM on 173701 (0.344 vs 0.472 above 0.8).
The honest limit: median track length is 1 on all three takes, so
almost no track accumulates more than one residual and the Markov
temporal part contributes little; the separation comes almost
entirely from the residual likelihood. What the HMM adds over the
threshold is a probabilistic score instead of a hard cut, the
reliability-over-time curve, and accumulation on the long tracks
that do exist. Chaining is fragile because SIFT re-detects slightly
different keypoints each frame on grass, so tracks break constantly.
Longer tracks (optical-flow tracking instead of per-frame
re-detection) would let the Markov part do real work; that is the
clear next step, and it connects directly to Jinming's Nov 5
question about leaving frames for reliability checking.

## What is still unproven

- Wind vs parallax. The model flags features that do not follow the
  rigid motion. On 173701 that signal is labelled "non-rigid signal,
  not separated from parallax" in the feature table, and this model
  does not separate the two either. It detects non-rigidity, not wind.
- IMU agreement is designed but not implemented. The design: per
  pair, compare the gyro-integrated rotation with the visual
  rotation as a pair-level prior weight; on still takes, the
  rotation-only predicted pixel motion gives a per-feature residual
  that is wind plus noise (no parallax when the camera does not
  translate). That channel is specified here, not run.
- The numbers sigma_U = 3.0 px and the transition matrix are set by
  hand, not fitted to data. Only three takes were tested.
- One bug was found and fixed during this work: the first belief
  curve plotted mean belief against track age, not time along the
  take. It is now mean belief against the global pair index, and
  the numbers above come from the fixed version.

## Files

- dataset-tools/reliability/reliability_hmm.py — the prototype.
- 174745_tracks.json, 173701_tracks.json, 080033_tracks.json — per-track
  reliability, length, median residual, plus the per-take summary.
- 174745_hist.png, 173701_hist.png, 080033_hist.png — reliability
  histograms. 174745_curve.png, 173701_curve.png, 080033_curve.png —
  reliability over time. All plots opened and judged before this
  summary was written.
