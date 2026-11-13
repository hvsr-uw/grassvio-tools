# Reliability models: a per-feature reliability check, control-anchored

Date: 2026-11-06. Updated 2026-11-13. This answers the Oct 30 "All"
TODO: "Look into reliability models: Bayesian model, Markov chain etc".
Don is separately building an empirical reliability score; this file
is the models side, not a duplicate of his work. Script:
dataset-tools/reliability/reliability_hmm.py (new). Bags read read-only.

## The model, in plain words

Every tracked feature sits on something rigid, so it moves exactly
the way the camera moved, or it does not: wind-blown grass, a bad
match, something moving on its own. Each frame pair gives one piece
of evidence: the feature's reprojection residual under the pairwise
rigid fit, in pixels. Rigid features cluster near zero residual;
unreliable ones spread widely.

The headline is the fraction of tracks whose median residual sits
below 0.5 px. The 0.5 px cutoff is measured from the wind-free
calibration control 102631c: its pooled 90th percentile is 0.4654 px,
rounded up to the next 0.1 px. It is frozen across evaluation takes.
This is a homography-fit compatibility fraction, not an estimated
fraction of rigid features or wind-free grass.

A Rayleigh reference score is kept as a diagnostic only. It is the
survival score of a Rayleigh with scale fitted by median matching
to the control. It is not a probability that a track is reliable,
and it saturates to 0 on the wind take, so it is never quoted as a
result.

There is no markov chain in this version. The first version had one,
and it is superseded, see below. The take-level wind label from the
feature table is deliberately NOT used as a prior: it is the held-out
label the model is tested against, so using it would be circular.

Parameters, all written down so they can be argued with:
- cutoff 0.5 px, from the control pooled p90 0.4654 px, rounded up.
  Control pool: 40,858 residuals, median 0.1893 px, p90 0.4654 px.
- Rayleigh diagnostic scale 0.160777 px, from median matching:
  0.1893 / sqrt(2 ln 2). The scale from p90 matching is 0.216872 px.
  The two disagree, so the control rejects a single-Rayleigh fit:
  the median fit predicts p90 0.3450 px, below the measured 0.4654 px.
- the equivalent diagnostic-score threshold at the cutoff is
  0.007940695, not 0.8. Changing the Rayleigh scale and converting
  the same pixel cutoff leaves the headline unchanged.

## What was read

- BANSAC (Piedade and Miraldo, ICCV 2023): a dynamic Bayesian network
  that updates each data point's inlier probability over RANSAC
  iterations, then samples by those probabilities. Method section read
  in full. Two ideas taken directly: per-point inlier probabilities
  updated by exact forward inference, and the 1st-order Markov
  assumption (their ablation found 1st order most accurate). Our first
  version moved the same update from RANSAC iterations to video
  frames: the "iterations" are consecutive frame pairs, the evidence
  is the reprojection residual instead of an inlier/outlier
  classification.
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
keypoint identity. Takes picked from the feature table labels: 174745
(still, no wind signal — control), 173701 (still, non-rigid signal in
the second half), 080033 (rotation, genuine loop, no wind signal).

| take | label (feature table) | tracks | median track length | frac median resid < 0.5 px | median residual |
|---|---|---|---|---|---|
| 20261030_174745 | still, no wind signal | 32,030 | 1 | 0.926 | 0.204 px |
| 20261026_173701 | still, non-rigid signal | 35,456 | 1 | 0.344 | 0.791 px |
| 20261007_080033 | rotation, no wind signal | 85,903 | 1 | 0.921 | 0.209 px |

## Does it separate reliable from unreliable? Yes, in two ways

1. Across takes. The wind-signal take (173701) stands apart: only
   34.4% of its tracks sit below the cutoff, against 92.6% on the
   still control and 92.1% on the rotating take. The gap is 58.0
   points. The rotating take looks like the still control, which is
   the right behaviour: the rigid fit absorbs camera motion, so
   rotation alone does not make features look unreliable. The check
   responds to non-rigid signal, not to camera motion.
2. Over time within a take. On 173701 the per-pair fraction below
   0.5 px starts at control level (about 0.91 for the first pairs),
   falls to about 0.15 in the second half, and ends near 0.2 (see
   173701_curve.png). That matches the feature table's note
   independently: first pairs at control level, then residuals rise
   and stay up. On 174745 the curve is flat near 0.92 from start to
   finish. On 080033 it mostly stays high, but pairs 68 to 76 fall
   to 0.22 to 0.33, so that control is not flat the whole way
   through. The check finds WHEN a take goes unreliable, not just
   that it did.

The 173701 histogram shows track median residuals with the 0.5 px
cutoff line, and the Rayleigh reference score per pair (see
173701_hist.png). The 25 frame window is 0.834 seconds, and the
verdict comes about 0.83 seconds late but does not look ahead past
the window.

## Why the markov model was removed

The first version was a Bayesian + Markov model: half-normal
likelihoods, sigma 0.5 px reliable and 3.0 px unreliable, transitions
P(R|R) = 0.95, P(U|R) = 0.05, P(R|U) = 0.30, P(U|U) = 0.70, prior
0.5, headline 0.8. It gave 47.2% of tracks above 0.8 on 173701,
against 97.8% on the still control and 97.5% on the rotating take.

That number did not survive. Re-running the same tracks with a
Rayleigh likelihood instead of the half-normal moves 173701 to 64
percent or 26 percent depending on how the scale is set, while the
ordering stays the same and the curve shape holds. The headline was
a property of the likelihood choice, not the data alone. The 0.8
threshold had no recorded justification either.

Median track length is 1 on all three takes, so almost no track
accumulates more than one residual and the Markov temporal part
contributes little; the separation comes almost entirely from the
residual itself. The plain residual rule separates the takes, and
its cutoff is measured from the control, not chosen. So the markov
part is gone. Longer tracks (optical-flow tracking instead of
per-frame re-detection) would let a temporal model do real work;
that is the clear next step, and it connects directly to Jinming's
Nov 5 question about leaving frames for reliability checking.
Chaining is fragile because SIFT re-detects slightly different
keypoints each frame on grass, so tracks break constantly.

## What is still unproven

- Wind vs parallax. The check flags features that do not follow the
  rigid motion. On 173701 that signal is labelled "non-rigid signal,
  not separated from parallax" in the feature table, and this check
  does not separate the two either. It detects non-rigidity, not wind.
- IMU agreement is designed but not implemented. The design: per
  pair, compare the gyro-integrated rotation with the visual
  rotation as a pair-level prior weight; on still takes, the
  rotation-only predicted pixel motion gives a per-feature residual
  that is wind plus noise (no parallax when the camera does not
  translate). That channel is specified here, not run.
- Only three takes were tested. The cutoff is measured from one
  control take; a second control take would be a better check.
- One bug was found and fixed during this work: the first belief
  curve plotted mean belief against track age, not time along the
  take. The current curve is per-pair fraction below the cutoff
  against time along the take.

## Files

- dataset-tools/reliability/reliability_hmm.py — the script, now the
  control-anchored version. The old markov version is kept as
  dataset-tools/reliability/reliability_hmm_v2_hmm.py.
- 174745_tracks.json, 173701_tracks.json, 080033_tracks.json — per-track
  median residual and compatibility, plus the per-take summary.
- 174745_hist.png, 173701_hist.png, 080033_hist.png — median residual
  histograms with the cutoff line, and the Rayleigh reference score.
  174745_curve.png, 173701_curve.png, 080033_curve.png — per-pair
  fraction below 0.5 px over time. All plots opened and judged
  before this summary was written.
