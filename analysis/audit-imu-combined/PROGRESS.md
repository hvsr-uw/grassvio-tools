# Combined-correction withheld-motion test — progress log

Work area: `analysis/audit-imu-combined/`.
Python: `hog-work/.venv/bin/python` (numpy; scipy NOT installed in the
venv, the optimizer is numpy-only, as in the init work). Bags read
read-only. Nothing uploaded to GitHub. No message sent to anyone.
Code written by hand; no browser use.

## 2026-11-10 — PRE-SPECIFICATION (all choices fixed before any fitting)

Written before any fit or prediction is run. No threshold, segment,
horizon, or baseline is changed after seeing results. If a pass
criterion fails, the work stops at the null with the numbers.

### Background (from the init work, imu-init/SUMMARY.md)

- The joint visual-pose-constrained initialization is NULL: gravity
  direction is stable to ~0.5 deg, but the accel bias is not
  determined by the data (moves 0.05-0.11 m/s^2 under 4x weaker
  priors; shifted windows disagree by up to 0.10 m/s^2). The prior,
  not the data, chooses the accel bias.
- Step 1 gyro bias WORKS: 0.004-0.011 rad/s per take, both
  initializations converge; on 155419 the fitted bias keeps the path
  rotation at 20.53 deg against the ORB total of 20.81 deg.
- The fitted accel-bias z-component is about +0.31 to +0.33 m/s^2 on
  every window of both loop takes (6x its sigma = 0.05 prior).
- The old holdout code in step2_joint.py (tests (c) and (d)) was
  written but NEVER executed, and it does NOT match this design: it
  froze the window-0 joint triple (gravity, gyro bias, accel bias)
  fitted jointly on 12 pairs, used horizons 1/2/4/8 pairs, compared
  only IMU vs constant velocity, started predictions at keyframe 12,
  and returned "not_applicable" for 155419. This test is written
  FRESH in `combined_test.py`, reusing only the verified machinery in
  `common.py` (load_take, pair_samples, preintegrate, lm,
  imu_origin_poses) and `build_window` from step2_joint.py for the
  per-window pair lists and visual finite-difference velocities.
  The propagation math in the old `predict()` was checked against the
  preintegrate conventions in common.py and is correct (dR maps
  frame-1 vectors to frame-2 vectors; R_new = R_old @ dR.T; dv/dp are
  in the pair-start body frame, so R @ dv and R @ dp map them to
  world); the fresh prediction code follows the same verified math
  and is checked again by the sanity gate below.

### The combined model

The accelerometer measures specific force f = a - g, so gravity and
accel bias enter the measurement only as a sum. The combined model
does NOT try to separate them: gravity direction is FROZEN at the
fitted direction from the init work, and ONE effective accel
correction b_a is fitted per take (per window for the stability
check) as a single specific-force correction. The gyro bias is
fitted separately (step 1) and frozen.

### Shared choices

- Takes: 20261007_080033, 20261007_075947, 20260926_155419 (ORB
  trajectories exist for these three only). No new ORB-SLAM3 runs.
- Keyframes: stride-5 colour frames, same pairs as the preint and
  init work: 88 pairs (080033), 46 pairs (075947), 10 pairs (155419).
- Time sync: integration windows shifted -50 ms against the colour
  interval, as in preint.py and the init work.
- Gyro convention: R = Exp(-integral of (omega - b_g) dt),
  trapezoid-integrated incrementally, as in preint.py.
- Frames: rotation between IMU and depth frames is the identity;
  translation t = (-0.030220, 0.007400, 0.016020) m; absolute visual
  poses are expressed at the IMU origin as R_imu = R_cam and
  p_imu = p_cam - R_cam @ t (ORB-SLAM3 T_b_c1 convention), as in the
  init work.
- Pair duration (measured 2026-11-10 from the colour stamps, before
  any fit): median pair dt 0.1669 s (080033), 0.1668 s (075947),
  0.1668 s (155419). Nominal stride-5 at 30 fps is 0.1667 s.
  Horizon mapping, stated explicitly: H = 1 pair = 0.167 s,
  H = 2 pairs = 0.334 s, H = 6 pairs = 1.001 s, H = 4 pairs =
  0.667 s, H = 8 pairs = 1.334 s. The primary horizons are
  0.1 / 0.3 / 1.0 s, realized as 1 / 2 / 6 pairs (the 0.1 s horizon
  cannot be realized in whole pairs; 1 pair at 0.167 s is the
  closest). H = 4 and H = 8 pairs are secondary, kept for
  continuity with the old test-(d) design.

### Frozen gravity direction

- Source: the window-0 fitted gravity direction from the step-2
  baseline run, `runs/<take>_step2_baseline.json`, windows[0]
  ["g_dir"]. Window 0 is pairs 0-11, which lies inside the fit
  segment below, so the frozen direction is derived from fit-segment
  data only.
- Known caveat, recorded here and bounded: the window-0 fit used a
  prior mean (the preint stillest-window gravity direction) that is
  a whole-take statistic. The fitted direction moved only 0.46 /
  0.66 / 2.79 deg from that prior mean on the three takes, and the
  direction is stable to ~0.5 deg under every perturbation tried in
  the init work, so this is not material leakage. It is recorded,
  not hidden.

### Gyro bias

- Frozen at the step-1 fitted bias per take, from
  `runs/<take>_step1.json` ("fits" -> "from_zero" -> "bias_rad_s"):
  080033 (-0.00182, 0.00367, -0.00169), 075947 (-0.00191, 0.00274,
  -0.00210), 155419 (-0.00710, -0.00775, 0.00010) rad/s.

### Effective-correction fit

- For a fit segment (a set of consecutive pairs), fit
  x = [b_a (3), velocities at the keyframe boundaries (3K)] by
  Levenberg-Marquardt (numpy only, same lm as common.py),
  minimizing weighted velocity and position preintegration
  residuals (sigma_vel = 0.01 m/s, sigma_pos = 0.003 m, same as
  step 2) with gravity direction and gyro bias frozen, plus a prior
  residual b_a / sigma_ba with prior mean 0. Rotation residuals are
  excluded: with b_g frozen they are constant in x and cannot
  affect the optimum. LM details: forward-difference Jacobian,
  steps 1e-4 m/s^2 (b_a) and 1e-4 m/s (velocities), lambda0 1e-3,
  max 50 iterations. Initialization: b_a = 0, velocities from
  visual finite differences (build_window's v_init).
- Fit segments (fit on these pairs only; nothing after them is used
  for the fit):
  - Loop takes (080033, 075947): pairs 0-23 (24 pairs, keyframes
    0-24, 4.0 s nominal).
  - 155419: pairs 0-3 (4 pairs, keyframes 0-4, 0.67 s nominal).
    The take has only 10 pairs total, so the fit segment is short
    by necessity; this is recorded, and 155419 is reported, not a
    pass gate (see below).
- Prior sweep: sigma_ba in {0.05, 0.20, 0.80} m/s^2, prior mean 0.
  The per-take effective correction and the predictions are computed
  at each sigma.

### Per-window stability fits

- Same fit procedure on each primary 12-pair window (starts
  0, 12, 24, ... as in step 2; 155419 uses its single 10-pair
  window), at sigma_ba = 0.05 (primary) and sigma_ba = 0.20
  (prior-sensitivity check).
- Stability metric per take: max over windows of
  ||b_a_window - median(b_a_windows)||_2, where the median is the
  per-component median across the take's windows.
- Pass threshold (P2 below): <= 0.05 m/s^2 on both loop takes.
  Also report the per-component ranges.

### Prediction

- Withheld region: loop takes start keyframe k >= 24 (first
  keyframe after the fit segment); 155419 k >= 4. All prediction
  starts k in the withheld region with k + H < len(chosen) and
  k >= 2. A start is skipped if any pair in its horizon returns
  fewer than 2 gyro samples; skipped starts are counted.
- Initial velocity v0 from visual finite differences over the 2
  preceding keyframes only: v0 = (p_k - p_{k-2}) / (t_k - t_{k-2}),
  IMU-origin positions. No future visual pose is used.
- Propagation: per pair, preintegrate with (b_g, b_a) and advance
  p = p + v*dt + 0.5*g*dt^2 + R @ dp; v = v + g*dt + R @ dv;
  R = R @ dR.T (verified math, see above). dt per pair from the
  colour stamps.
- Horizons: primary H = 1, 2, 6 pairs (0.167 / 0.334 / 1.001 s);
  secondary H = 4, 8 pairs (0.667 / 1.334 s).
- Predictors (all from the same start keyframe k and the same v0):
  - IMU-combined (candidate): g = 9.81 * frozen g_dir; b_g =
    step-1 fitted; b_a = per-take effective correction fitted on
    the fit segment at the given sigma_ba.
  - CV (key comparator): p_pred = p_k + v0 * dt_total, dt_total
    from the stamps over the horizon.
  - IMU-uncorrected: g = 9.81 * prior-mean gravity direction
    (build_window's g0 at window 0, i.e. the preint stillest-window
    direction carried to keyframe 0); b_g = step-1 fitted; b_a = 0.
  - IMU-gravity-only: g = 9.81 * frozen g_dir; b_g = step-1 fitted;
    b_a = 0.
  - Note on the task's phrase "constant-velocity + uncorrected
    IMU": this is read as the uncorrected-IMU propagation above
    alongside CV, since a hybrid CV+IMU predictor is not a defined
    method and the isolating comparison (what each fitted piece
    contributes) is what the test needs. The interpretation is
    recorded here before running, not chosen after seeing results.

### Metrics (per horizon, per take, per predictor)

- Median and 90th-percentile |p_pred - p_orb| in metres, and
  n_starts (starts actually predicted; skipped starts counted
  separately).
- Fraction of starts where the IMU-combined error is smaller than
  the CV error (same start, same horizon).
- Report all four predictors and all five horizons on all three
  takes. No cherry-picked window, no mean alone.

### Sanity gate (code check, pre-specified)

- Median 1-pair IMU-combined error must be < 10 mm on each loop
  take. Rationale: per-pair fit residuals in the init work were
  ~2 mm, and the v0 finite-difference error is ~1 cm/s x 0.167 s
  ~ 2 mm, so a few mm is expected; above 10 mm indicates a
  propagation or convention error. If the gate trips: STOP, debug
  the propagation, report it as a code problem — do not report a
  null and do not proceed.

### Pass criteria (ALL must hold)

- P1 (beats constant velocity): at H = 1 and H = 2, median
  IMU-combined error < median CV error AND fraction of starts
  beating CV > 0.5, on BOTH loop takes (080033, 075947). H = 6
  (1.0 s) is reported but NOT required (longer horizon, harder).
- P2 (stable correction): max ||b_a_window - median b_a||_2 <=
  0.05 m/s^2 across the primary windows on both loop takes, at
  sigma_ba = 0.05.
- P3 (prior insensitivity): per take,
  ||b_a(sigma=0.20) - b_a(sigma=0.05)||_2 <= 0.02 m/s^2, AND the
  median IMU-combined prediction error at H = 1, 2 changes by less
  than 25% relative between sigma = 0.05 and sigma = 0.20, on both
  loop takes. (sigma = 0.80 is reported for completeness.)
- 155419 is REPORTED on all metrics but is NOT a pass gate:
  rotation in place, position nearly fixed, CV is near-perfect
  there, so a win over CV is not expected and not required.
- What does NOT settle it: in-sample fit improvement, a win on one
  window or one take only, or a correction that drifts with the
  window. The criteria above exclude all three.

### Scope

- If all criteria pass, the claim is a usable SHORT-TERM translation
  prior only (the next few centimetres / seconds) in a smoother
  carrying velocity and bias forward, using IMU translation only
  between visual updates. No whole-take inertial translation
  estimate, no loop closure.
- If any criterion fails: report the null with the numbers and stop.
- NOT part of this task: grounding the bias prior in a measured
  turn-on bias (10-20 s static rest before each take, 5-10 cold
  starts). That needs NEW recordings; it is flagged in SUMMARY.md
  as a data ask for Jinming, not attempted from the existing bags.
- The old test (c) (velocity-only refit on withheld windows) is NOT
  run: this prediction test replaces it.

## 2026-11-10 — test run DONE: NULL, all three criteria fail

Script `combined_test.py` (fresh; old holdout code not used — see
pre-specification for why). Comparison script `compare_combined.py`.
Outputs `runs/<take>_combined.json` and `runs/compare_output.txt`.
Zero skipped prediction starts on every horizon of every take.

### Code-correctness checks (passed before any verdict was read)

- The per-window stability fit at window 0, sigma = 0.05, reproduces
  the step-2 baseline joint fit's b_a to 3 decimals on all three
  takes (080033 [-0.033, 0.097, 0.329] vs [-0.034, 0.097, 0.329];
  075947 [-0.070, 0.030, 0.325] vs [-0.071, 0.030, 0.324];
  155419 [0.076, 0.249, 0.166] vs [0.076, 0.249, 0.166]). With
  gravity and gyro bias frozen at the joint-fit values, the
  dedicated correction fit returns the joint fit's accel bias, as
  it must — the new fit code is consistent with the verified joint
  fit.
- Sanity gate PASSES: median 1-pair IMU-combined error 6.74 mm
  (080033) and 6.20 mm (075947), both under the 10 mm gate. The
  propagation is correct; the null below is a data/method result,
  not a code error.
- One bug found and fixed during the smoke run (before any result
  was read): build_window was called with 12 pairs on 155419, which
  has only 10 pairs (IndexError). The prior-mean gravity direction
  is now taken from a window clamped to the take's pair count. No
  result was produced before the fix.

### Per-take effective correction (fit segment, sigma = 0.05)

| take | fit segment | b_a (m/s^2) | rms pos |
|---|---|---|---|
| 080033 | pairs 0-23 | (-0.015, 0.106, 0.358) | 1.79 mm |
| 075947 | pairs 0-23 | (-0.135, -0.004, 0.342) | 1.99 mm |
| 155419 | pairs 0-3 | (0.037, 0.150, 0.099) | 3.53 mm |

Prior sweep |b_a(0.20) - b_a(0.05)|: 0.0265 (080033), 0.0258
(075947), 0.2432 (155419). |b_a(0.80) - b_a(0.05)|: 0.0283,
0.0275, 0.2817. The z-component stays near +0.34 to +0.36 on both
loop takes at every sigma.

### Stability (sigma = 0.05)

- 080033, 7 windows: median b_a (0.008, 0.125, 0.319);
  per-component ranges (0.124, 0.098, 0.101); max deviation from
  the median 0.1001 m/s^2.
- 075947, 3 windows: median b_a (-0.175, -0.040, 0.325);
  per-component ranges (0.117, 0.074, 0.014); max deviation
  0.1262 m/s^2.
- 155419, single window: vacuous (0.0000), not a gate.

### Predictions (sigma = 0.05; median position error, mm)

| take | H | n | combined | CV | uncorrected | gravity-only | frac beats CV |
|---|---|---|---|---|---|---|---|
| 080033 | 1 | 64 | 6.74 | 3.58 | 10.45 | 10.34 | 0.19 |
| 080033 | 2 | 63 | 12.69 | 6.43 | 28.59 | 28.08 | 0.24 |
| 080033 | 6 | 59 | 64.14 | 20.55 | 210.12 | 206.87 | 0.05 |
| 080033 | 4 | 61 | 31.56 | 15.28 | 98.41 | 96.29 | 0.08 |
| 080033 | 8 | 57 | 109.97 | 29.76 | 360.79 | 352.48 | 0.02 |
| 075947 | 1 | 22 | 6.20 | 4.74 | 10.79 | 10.67 | 0.18 |
| 075947 | 2 | 21 | 12.48 | 9.11 | 29.02 | 30.44 | 0.33 |
| 075947 | 6 | 17 | 61.69 | 23.04 | 212.41 | 223.79 | 0.12 |
| 075947 | 4 | 19 | 33.22 | 17.16 | 101.14 | 106.07 | 0.21 |
| 075947 | 8 | 15 | 108.69 | 38.06 | 370.15 | 384.19 | 0.07 |
| 155419 | 1 | 6 | 9.69 | 9.82 | 11.11 | 11.75 | 0.67 |
| 155419 | 2 | 5 | 29.57 | 24.05 | 36.77 | 38.86 | 0.40 |
| 155419 | 6 | 1 | 116.99 | 113.89 | 289.77 | 204.05 | 0.00 |
| 155419 | 4 | 3 | 81.24 | 68.32 | 139.72 | 118.83 | 0.33 |
| 155419 | 8 | 0 | - | - | - | - | - |

p90 values are in runs/compare_output.txt and in SUMMARY.md.

### Pass/fail

- P1 (beats CV at H=1, H=2, median and frac > 0.5, both loop
  takes): FAIL on both. 080033 frac 0.19 / 0.24; 075947 frac
  0.18 / 0.33. The combined model loses at every horizon and every
  sigma on both loop takes.
- P2 (stability <= 0.05 m/s^2): FAIL on both. 0.1001 (080033),
  0.1262 (075947).
- P3 (prior insensitivity): FAIL on both, on the |db_a| leg only:
  |db_a| 0.05 -> 0.20 is 0.0265 / 0.0258, over the 0.02
  threshold. The prediction leg passes (median error changes 3.0%
  / 2.6% on 080033 and 3.4% / 9.8% on 075947 at H=1 / H=2).
  Note: the P3 fail is marginal (0.026 vs 0.02), but P1 and P2 fail
  decisively, so the overall null does not hinge on this margin.
- 155419 (reported, not a gate): H=1 is a median tie (9.69 vs
  9.82 mm, frac 0.67 on n=6 — too thin to claim anything); H=2
  loses; H=6 has n=1; H=8 has n=0.

### Verdict

NULL. The combined (gravity + accel-bias) correction does not
transfer to withheld motion and does not beat constant velocity on
these takes. Do not use it as a short-term translation prior in the
VO chain. The fitted correction does carry real signal — it beats
the uncorrected and gravity-only variants by about 3x at long
horizons — but on these slow, smooth takes constant velocity is the
stronger predictor at every horizon, and the correction absorbs
fit-segment velocity error rather than a sensor property (the
stability failure is the same fact from the fit side).
