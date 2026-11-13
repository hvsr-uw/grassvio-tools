# Combined-correction withheld-motion test — final verdict: null

Date: 2026-11-10. Work area:
`analysis/audit-imu-combined/`.
Scripts: `combined_test.py` (fresh), `compare_combined.py`.
Bags read read-only. Nothing uploaded to GitHub. No message sent to
anyone. The full test design, pass/fail thresholds, takes, horizons,
baselines, and prior sweep were written in `audit-imu-combined/PROGRESS.md` before any
fit or prediction was run, and none was changed after seeing results.
Code written by hand; no browser use.

## Verdict up front

**The combined (gravity + accel-bias) correction does NOT pass its
withheld-motion test, so there is no usable short-term inertial
translation prior from this branch.** All three pre-specified pass
criteria fail on both loop takes:

- **P1 — beats constant velocity: FAIL.** The IMU-combined
  prediction loses to constant velocity at every horizon and every
  prior width on both loop takes. At the primary short horizons
  (0.167 s, 0.334 s) the fraction of starts beating CV is only
  0.18-0.38, and the median error is about 1.3-2x the CV error.
- **P2 — stable correction: FAIL.** The per-window effective
  correction wanders by up to 0.10 m/s^2 (080033) and 0.13 m/s^2
  (075947) from its own median across windows of the same take —
  two to three times the 0.05 tolerance. Overlapping windows
  disagree on the correction by as much as the correction itself
  in some components.
- **P3 — prior insensitivity: FAIL on the correction leg.**
  Weakening the prior from sigma 0.05 to 0.20 moves the fitted
  correction by 0.026 m/s^2 on both loop takes, over the 0.02
  tolerance. (The prediction leg passes: median errors change only
  3-10%.) The P3 fail is marginal, but P1 and P2 fail decisively,
  so the null does not hinge on this margin.

The fitted correction is not useless — it beats the uncorrected and
gravity-only IMU variants by about 3x at long horizons — but on
these slow, smooth takes constant velocity is the stronger predictor
at every horizon, and the correction absorbs fit-segment velocity
error rather than a sensor property. Do not use the combined
correction as a translation prior in the VO chain.

## What was tested

The accelerometer measures specific force f = a - g, so gravity and
accel bias enter only as a sum. The combined model refuses to
separate them: gravity direction is frozen at the fitted direction
from the init work (stable to ~0.5 deg), and one effective accel
correction is fitted per take as a single specific-force correction,
with the gyro bias frozen at the step-1 fitted value (0.004-0.011
rad/s per take). The correction is fitted on a fit segment only
(pairs 0-23 on the loop takes, pairs 0-3 on 155419) and used to
predict withheld motion — the last portion of each take — with no
visual updates, over horizons 0.167 / 0.334 / 1.001 s (1 / 2 / 6
stride-5 pairs; the 0.1 s horizon is realized as 1 pair since a
shorter whole-pair horizon does not exist), plus 0.667 / 1.334 s
(4 / 8 pairs) as secondary horizons. Comparators: constant velocity
(the key comparator), uncorrected IMU (prior-mean gravity, zero
accel bias), and gravity-only IMU (fitted gravity direction, zero
accel bias). The prior is swept over sigma = 0.05 / 0.20 / 0.80
m/s^2. Full design and thresholds are in audit-imu-combined/PROGRESS.md.

Frozen gravity directions (window-0 step-2 baseline fits):
080033 (-0.121, 0.280, 0.952); 075947 (-0.238, 0.087, 0.967);
155419 (0.276, 0.805, 0.526).

## Code checks (passed before any verdict was read)

- The per-window correction fit at window 0, sigma = 0.05,
  reproduces the step-2 baseline joint fit's accel bias to 3
  decimals on all three takes. With gravity and gyro bias frozen at
  the joint-fit values, the dedicated fit returns the joint fit's
  accel bias, as it must.
- Sanity gate PASSES: median 1-pair IMU-combined error 6.74 mm
  (080033) and 6.20 mm (075947), both under the pre-specified 10 mm
  gate. The propagation is correct; this null is a data/method
  result, not a code error.
- Zero skipped prediction starts on every horizon of every take.
- One bug was found and fixed during the smoke run, before any
  result was read: the prior-mean gravity direction was built from
  a 12-pair window on 155419, which has only 10 pairs. The window
  is now clamped to the take's pair count.

## Per-take effective correction (fit segment)

| take | fit segment | b_a at sigma 0.05 (m/s^2) | fit rms pos |
|---|---|---|---|
| 080033 | pairs 0-23 | (-0.015, 0.106, 0.358) | 1.79 mm |
| 075947 | pairs 0-23 | (-0.135, -0.004, 0.342) | 1.99 mm |
| 155419 | pairs 0-3 | (0.037, 0.150, 0.099) | 3.53 mm |

The z-component stays near +0.34 to +0.36 m/s^2 on both loop takes
at every prior width. Weakening the prior from 0.05 to 0.20 moves
the correction by 0.0265 (080033), 0.0258 (075947), and 0.2432
(155419) m/s^2; from 0.05 to 0.80 by 0.0283, 0.0275, and 0.2817.
The in-sample fit is good at every sigma (rms position 1.5-3.5
mm) — the fit fits; it is the transfer to withheld motion that
fails.

## Stability: per-window effective corrections (sigma = 0.05)

| take | windows | median b_a (m/s^2) | per-component range | max deviation from median |
|---|---|---|---|---|
| 080033 | 7 | (0.008, 0.125, 0.319) | (0.124, 0.098, 0.101) | 0.1001 m/s^2 |
| 075947 | 3 | (-0.175, -0.040, 0.325) | (0.117, 0.074, 0.014) | 0.1262 m/s^2 |
| 155419 | 1 | (0.076, 0.249, 0.166) | — (single window) | 0.0000 (vacuous) |

Per-window values, 080033 (starts 0, 12, 24, 36, 48, 60, 72):
(-0.033, 0.097, 0.329), (0.008, 0.096, 0.322),
(0.065, 0.126, 0.272), (0.055, 0.173, 0.245),
(0.023, 0.159, 0.228), (-0.060, 0.125, 0.319),
(-0.041, 0.075, 0.319). 075947 (starts 0, 12, 24):
(-0.070, 0.030, 0.325), (-0.187, -0.040, 0.311),
(-0.175, -0.044, 0.325). The x-component alone wanders over a
0.12 m/s^2 range within each loop take.

## Predictions (sigma = 0.05; median position error)

### 080033 (loop take)

| horizon | n | combined (med / p90) | CV (med / p90) | uncorrected | gravity-only | frac beats CV |
|---|---|---|---|---|---|---|
| 0.167 s (1 pr) | 64 | 6.74 / 12.18 mm | 3.58 / 7.65 mm | 10.45 mm | 10.34 mm | 0.19 |
| 0.334 s (2 pr) | 63 | 12.69 / 23.02 mm | 6.43 / 16.16 mm | 28.59 mm | 28.08 mm | 0.24 |
| 1.001 s (6 pr) | 59 | 64.14 / 94.47 mm | 20.55 / 37.17 mm | 210.12 mm | 206.87 mm | 0.05 |
| 0.667 s (4 pr) | 61 | 31.56 / 49.88 mm | 15.28 / 27.13 mm | 98.41 mm | 96.29 mm | 0.08 |
| 1.334 s (8 pr) | 57 | 109.97 / 150.72 mm | 29.76 / 55.21 mm | 360.79 mm | 352.48 mm | 0.02 |

### 075947 (loop take)

| horizon | n | combined (med / p90) | CV (med / p90) | uncorrected | gravity-only | frac beats CV |
|---|---|---|---|---|---|---|
| 0.167 s (1 pr) | 22 | 6.20 / 11.51 mm | 4.74 / 7.02 mm | 10.79 mm | 10.67 mm | 0.18 |
| 0.334 s (2 pr) | 21 | 12.48 / 20.83 mm | 9.11 / 16.03 mm | 29.02 mm | 30.44 mm | 0.33 |
| 1.001 s (6 pr) | 17 | 61.69 / 102.25 mm | 23.04 / 69.54 mm | 212.41 mm | 223.79 mm | 0.12 |
| 0.667 s (4 pr) | 19 | 33.22 / 52.54 mm | 17.16 / 43.89 mm | 101.14 mm | 106.07 mm | 0.21 |
| 1.334 s (8 pr) | 15 | 108.69 / 159.11 mm | 38.06 / 96.44 mm | 370.15 mm | 384.19 mm | 0.07 |

### 155419 (rotation in place — reported, not a pass gate)

| horizon | n | combined (med / p90) | CV (med / p90) | uncorrected | gravity-only | frac beats CV |
|---|---|---|---|---|---|---|
| 0.167 s (1 pr) | 6 | 9.69 / 14.65 mm | 9.82 / 11.73 mm | 11.11 mm | 11.75 mm | 0.67 |
| 0.334 s (2 pr) | 5 | 29.57 / 33.70 mm | 24.05 / 28.22 mm | 36.77 mm | 38.86 mm | 0.40 |
| 1.001 s (6 pr) | 1 | 116.99 mm | 113.89 mm | 289.77 mm | 204.05 mm | 0.00 |
| 0.667 s (4 pr) | 3 | 81.24 / 86.21 mm | 68.32 / 72.41 mm | 139.72 mm | 118.83 mm | 0.33 |
| 1.334 s (8 pr) | 0 | — (no starts) | — | — | — | — |

The H=1 median on 155419 is a tie (9.69 vs 9.82 mm) on n=6 — too
thin to claim anything; H=2 loses and the longer horizons have
n=1 or n=0.

### Prior sweep (predictions)

Median combined error barely moves with the prior width: at H=1 /
H=2, 080033 goes 6.74 / 12.69 mm (sigma 0.05) to 6.54 / 12.36 mm
(sigma 0.20) to 6.52 / 12.32 mm (sigma 0.80); 075947 goes 6.20 /
12.48 mm to 5.99 / 11.26 mm to 5.99 / 11.18 mm. The predictions are
insensitive to the prior; the fitted correction itself is not
(P3 leg 1 fails at 0.026 vs the 0.02 tolerance). At no sigma does
the combined model beat CV on either loop take.

## Mechanism (what the numbers support)

Two facts, both measured, point the same way. From the fit side,
the effective correction is not stable: overlapping windows of the
same take disagree on it by up to 0.13 m/s^2, and the same
component (b_a_x) wanders over a 0.12 m/s^2 range within each loop
take — the same instability the joint fit showed for the separated
accel bias. From the prediction side, the correction fitted on the
first 4 seconds does not transfer to the rest of the take: the
IMU-combined error grows to about 3x the CV error by 1.0-1.3 s, and
even at the shortest horizon (0.167 s) it is 1.3-1.9x the CV error.
The fitted correction absorbs fit-segment velocity error rather
than a sensor property, so it helps against the uncorrected and
gravity-only variants (about 3x better at long horizons) but
cannot beat a constant-velocity predictor on takes where the camera
moves at a near-constant ~5 cm/s with slowly changing velocity.
This is the same shallow-valley mechanism as the joint-fit null:
millimetre-scale visual motion with a free velocity at every
keyframe leaves the specific-force correction weakly constrained,
whatever the parametrization.

## Scope and what is NOT claimed

- No short-term translation prior is claimed from this branch.
  The combined correction must not be used in the VO chain.
- No whole-take inertial translation estimate and no loop closure
  were tested or claimed.
- The in-sample fit being good (1.5-3.5 mm rms) is not evidence
  for the model — the withheld-motion test is the evidence, and it
  fails.

## Data ask for Jinming (not part of this task)

Grounding the accel-bias prior in a measured turn-on bias still
needs NEW recordings and was not attempted from the existing bags:
10-20 s of static rest before each take, repeated over 5-10 cold
starts, so the per-take turn-on bias (mean accel per axis minus
expected gravity) and its across-take spread can be measured. The
same recordings should include deliberate roll and pitch changes
with gentle starts and stops, since the current takes barely change
attitude and gravity-direction error and accel bias may be
inseparable on them.

## Files

All under `analysis/audit-imu-combined/`:

- `audit-imu-combined/PROGRESS.md` — pre-specification (written before any fit) and
  the run log with the full result tables.
- `audit-imu-combined-summary.md` — this file.
- `combined_test.py` — the fresh test script (per-take and
  per-window correction fits, withheld predictions).
- `compare_combined.py` — metric tables and P1/P2/P3 evaluation
  from the run JSONs.
- `runs/<take>_combined.json` — per-take corrections, stability
  fits, and prediction metrics for all sigmas and horizons.
- `runs/compare_output.txt` — the full comparison output.
