# Visual-pose-constrained inertial initialization — progress log

Work area: `analysis/audit-imu-init/`.
Python: `hog-work/.venv/bin/python` (numpy 2.5.3, cv2; scipy NOT installed
in the venv, verified 2026-11-10 — the optimizer is numpy-only).
Bags read read-only. Nothing uploaded to GitHub. No message sent to anyone.
Code written by hand; no browser use.

## 2026-11-10 — calibration verified from the bags (before any fitting)

Read the `tf/ref_0` string topics of all three bags directly
(080033, 075947, 155419 — identical values on all three):

- Gyro tf: rotation identity, translation (-0.030220, 0.007400, 0.016020) m.
- Accel tf: rotation identity, translation (-0.030220, 0.007400, 0.016020) m.
- Depth tf: rotation identity, translation (0, 0, 0) — depth is the reference.
- Colour tf (080033): rotation within 0.0039 of identity (max ~0.22 deg),
  translation (-0.059037, 0.000078, 0.000128) m — the 5.9 cm colour baseline.
- `imu_intrinsic` (gyro and accel): identity scale, zero bias/noise variances
  (no information there).

Transform used, stated exactly:

- Rotation between IMU and depth frames is the identity: R_imu = R_depth.
  The ORB poses are depth-frame poses (depth used as-is, depth intrinsics
  in D455F.yaml), so the visual relative rotations need no rotation change.
- Translation t = (-0.030220, 0.007400, 0.016020) m is the IMU origin
  expressed in the depth frame (bag tf topic, confirmed by D455F.yaml
  `IMU.T_b_c1`, body-to-camera). Absolute visual poses are expressed at
  the IMU origin as: R_imu,k = R_cam,k and
  p_imu,k = p_cam,k - R_cam,k @ t (ORB-SLAM3 T_b_c1 convention,
  T_wb = T_wc @ inv(T_bc)).
- Step 1 (rotation only) does NOT apply the translation: R is unchanged by
  it, and the visual poses stay at the camera origin for that step.
- Step 2 applies the translation as above, so visual poses are expressed
  at the IMU origin.
- Note on preint.py: its `conjugate_to_depth` used the opposite sign
  (t_depth = t_imu + t - R t, versus t_imu + R t - t under the convention
  above). The per-pair difference is 2|R t - t|, at most ~1.4 mm, and the
  residuals here use position differences, where the lever-arm term
  |(R_j - R_i) t| is at most ~0.7 mm per pair — smaller than the accel-bias
  signal being tested. The convention above is the one used here; the
  preint results are unaffected at the scale of their verdicts.

## 2026-11-10 — PRE-SPECIFICATION (all choices fixed before any fitting)

Every choice below was written before any bias or joint fit was run, and
will not be changed after seeing results. If a test fails, the work stops
at the null with the numbers.

### Shared

- Takes: 20261007_080033, 20261007_075947, 20260926_155419 (ORB trajectories
  exist for these three only). No new ORB-SLAM3 runs.
- Keyframes: stride-5 colour frames, the same pairs as the preint work:
  88 pairs (080033), 46 pairs (075947), 10 pairs (155419).
- Time sync: integration windows shifted -50 ms against the colour interval,
  as in preint.py.
- Gyro convention: R_points = Exp(-integral of (omega - b_g) dt),
  trapezoid-integrated incrementally, as in preint.py.
- ORB pose alignment: ORB absolute poses are indexed by paired colour frame
  in the same order as preint.py; counts are 440 poses / 441 frames (080033),
  235 / 235 (075947), 54 / 55 (155419). Chosen keyframe indices are all
  inside the pose lists; if one ever exceeded the list the pair would be
  dropped and recorded.
- Optimizer: hand-rolled Levenberg-Marquardt, numpy only. No scipy.

### Step 1 — gyro bias from visual relative rotations

- Visual relative rotation per pair: R_vis = (inv(A_j) @ A_i)[:3,:3] from
  the ORB absolute poses (R is the same in the IMU frame).
- Gyro relative rotation R_gyro(b): trapezoid integration of (omega - b)
  over the shifted pair window, incremental composition (preint machinery).
- Fit: LM on 3 parameters (b_g), residuals rotvec(R_vis^T R_gyro(b)) in
  radians, plain least squares, no robust loss, no outlier rejection.
- Primary initialization b = 0. Second run initialized at the per-take
  median gyro bias, as a convergence check; both reported.
- Report per take: fitted b_g, |b_g|, residual rotation error
  (median / RMS / max) at b = 0, at the fitted b, and at the median bias;
  gyro path rotation with the fitted bias against the ORB total rotation.
- Key check on 155419: the fitted |b_g| must be far below the 0.1911 rad/s
  median, and the fitted-bias path rotation must stay near the ORB total
  of 23.2 deg (the median bias collapses it to 7.05 deg).

### Step 2 — joint refinement of gravity direction, biases, velocities

- Windows: 12 consecutive stride-5 pairs (60 colour frames, 2.0 s nominal).
  080033: 7 windows (pairs 0-83; pairs 84-87 unused). 075947: 3 windows
  (pairs 0-35; pairs 36-45 unused). 155419: a single window of all 10 pairs
  (1.67 s). Total 11 windows.
- State per window: gravity direction (2 parameters, unit vector in the
  world frame, parametrized in the tangent plane at the prior-mean
  direction), gyro bias b_g (3), accel bias b_a (3), and one velocity
  vector v_k at each keyframe boundary in the window (13 for a 12-pair
  window, 11 for 155419). 47 parameters (12-pair) / 41 (155419). Visual
  rotations and positions are held fixed.
- Gravity magnitude fixed at 9.81 m/s^2; only its direction is estimated.
- Preintegration per pair is gravity-free (unlike preint.py, which
  subtracted gravity inside the integration): accel samples are
  interpolated onto the gyro grid, bias-corrected in the body frame
  (f_k - b_a), rotated into the pair-start frame with the incrementally
  integrated gyro rotation, and trapezoid-integrated to dvel/dpos.
  Rotation part identical to preint.py.
- Residuals per pair (standard preintegration residuals, visual fixed):
  - rotation: r_rot = Log(R_vis^T R_pred), R_vis = R_j^T R_i from ORB,
    R_pred = gyro preintegrated rotation with b_g.
  - velocity: r_vel = R_i^T (v_j - v_i - g dt) - dvel, g = 9.81 * g_dir.
  - position: r_pos = R_i^T (p_j - p_i - v_i dt - 0.5 g dt^2) - dpos,
    with p at the IMU origin (translation applied as above).
- Residual weights, fixed: sigma_rot = 0.003 rad, sigma_vel = 0.01 m/s,
  sigma_pos = 0.003 m.
- Priors (baseline), fixed:
  - b_g: mean = step-1 fitted bias for that take, sigma = 0.01 rad/s/axis.
  - b_a: mean = 0, sigma = 0.05 m/s^2/axis.
  - gravity direction: mean = preint stillest-window gravity direction,
    carried into the world frame with the ORB pose at the window's first
    keyframe (gyro-chained from the preint window start with the D-table),
    magnitude reset to 9.81; sigma = 5 deg (0.0873 rad) on the tangent
    parameters.
  - velocities: no prior. Initialized from visual finite differences
    (central differences of the IMU-origin positions; one-sided at the
    window ends).
- LM details: forward-difference Jacobian; steps 1e-4 rad (gravity
  tangent), 1e-5 rad/s (b_g), 1e-4 m/s^2 (b_a), 1e-4 m/s (velocities).
  Damping lambda0 = 1e-3, x10 on rejection, /10 on acceptance, max
  50 iterations; stop when relative cost change < 1e-9 or max |step| < 1e-10.
  Initialization: gravity at prior mean, b_g at step-1 value, b_a = 0,
  velocities from visual finite differences.
- Constant biases over each window only. No per-frame bias.

### Observability tests (thresholds fixed now; a fail stops the work)

- Test (a) — same answer from different starts and windows. (a1) Re-run
  every window from a perturbed start: gravity direction tilted 10 deg
  about the world x-axis from the primary start, b_g = 0, b_a = 0,
  velocities from visual finite differences. Pass per window: final
  gravity direction within 2.0 deg of the primary run, |delta b_a| <=
  0.02 m/s^2 (norm), |delta b_g| <= 0.005 rad/s (norm). (a) passes only if
  (a1) passes on at least 9 of 11 windows AND (a2) passes: (a2) shifted
  windows starting at pairs 6, 18, 30, ... (12 pairs long, where they fit)
  give fitted (gravity, b_a, b_g) whose median disagreement with the
  nearest primary window (by centre pair) is within the same tolerances
  (2.0 deg / 0.02 m/s^2 / 0.005 rad/s). If (a) fails: STOP, report the
  null with the numbers, do not proceed to (b)-(d) as if the estimates
  meant something.
- Test (b) — weaken the bias priors: sigma_bg 0.01 -> 0.04 rad/s,
  sigma_ba 0.05 -> 0.20 m/s^2 (gravity prior unchanged at 5 deg). Re-fit
  all windows. Pass: median across windows of |delta gravity| <= 2.0 deg
  and |delta b_a| <= 0.02 m/s^2 versus the baseline fits. If (b) fails:
  STOP, report the null.
- Test (c) — fit on one window, test on withheld motion. Per take, freeze
  (gravity, b_g, b_a) from the FIRST window (pairs 0-11) and, on the NEXT
  window (pairs 12-23; withheld), re-optimize velocities only and record
  the weighted RMS residual. Compare against the same quantity computed
  with baseline parameters (preint gravity direction, step-1 b_g,
  b_a = 0). Also report the withheld window with the largest tilt range
  separately, where tilt_k = angle between the camera z-axis in world
  coordinates and the fitted gravity direction. This test is evidence, not
  a stop gate.
- Test (d) — constant-velocity baseline. Predictions start at keyframes in
  the withheld region (all keyframes after the first fit window, per take).
  Horizons H = 1, 2, 4, 8 pairs (0.17 / 0.33 / 0.67 / 1.33 s). Initial
  velocity from visual finite differences over the 2 preceding keyframes:
  v_init = (p_k - p_{k-2}) / (t_k - t_{k-2}), IMU-origin positions.
  IMU prediction propagates with the frozen first-window parameters
  (gravity, b_g, b_a). Baseline: constant velocity, p_pred = p_k +
  v_init * dt. Future visual poses are excluded from each prediction.
  Metric: median |p_pred - p_orb| over prediction starts, per horizon.
  Pass: IMU beats constant velocity at H = 4 and H = 8.
- Fallback (only if the observability tests fail): a visually anchored
  sliding-window smoother as a short-term translation prior, as a bounded
  test under the same honest tests; if that also fails, stop at the null.

## 2026-11-10 — step 1 DONE: gyro bias from visual relative rotations

Script `step1_bias.py`, outputs `runs/<take>_step1.json`. Both
initializations (b = 0 and b = median bias) converge to the same bias on
all three takes (3 LM iterations each).

| take | fitted b_g (rad/s) | \|b_g\| | median bias | residual median deg: b=0 / fitted / median |
|---|---|---|---|---|
| 080033 | (-0.00182, 0.00367, -0.00169) | 0.00443 | 0.01685 | 0.0838 / 0.0772 / 0.1845 |
| 075947 | (-0.00191, 0.00274, -0.00210) | 0.00394 | 0.01630 | 0.0995 / 0.0971 / 0.1824 |
| 155419 | (-0.00710, -0.00775, 0.00010) | 0.01051 | 0.19109 | 0.1553 / 0.1119 / 1.8611 |

155419 key check PASSES: the fitted |b_g| = 0.01051 rad/s is far below
the 0.1911 rad/s median, the fitted-bias path rotation is 20.53 deg
against the ORB total of 20.81 deg (median bias collapses it to 7.05 deg),
and the residual median drops from 1.8611 deg (median bias) to 0.1119 deg.
The per-take median gyro vector is NOT sensor bias — even b = 0 beats it
on every take. The gyro bias is genuinely small (0.004-0.011 rad/s) and
the visual-fit bias is used as the step-2 gyro-bias prior mean.

One refinement to the pre-specified step-2 gravity prior (recorded here
before step 2 runs): the D-table chaining that carries the preint
stillest-window gravity direction to each window start uses the STEP-1
fitted bias, not the per-take median bias. Reason: the median bias is
now measured to be wrong (it eats real rotation on 155419, where
chaining 0.75 s with it would tilt the carried gravity by ~8 deg).
The gravity vector itself (minus mean accel over the stillest window)
is unchanged; only the rotation carrying it uses the better bias.

## 2026-11-10 — step 2 DONE: joint fit + observability tests (a), (b)

Scripts `step2_joint.py` (modes baseline / perturbed / weakprior /
shifted) and `compare_tests.py`. Outputs `runs/<take>_step2_*.json`
and `runs/observability_comparison.json`. All 11 windows (7 + 3 + 1)
fit in 4-5 LM iterations each.

Baseline estimates (gravity tilt from the preint prior mean; full
vectors in the JSONs):

| take | window | g tilt (deg) | b_g (rad/s) | b_a (m/s^2) | rms rot / vel / pos |
|---|---|---|---|---|---|
| 080033 | @0 | 0.46 | (-0.00124, 0.00217, -0.00159) | (-0.034, 0.097, 0.329) | 0.024 deg / 0.0050 m/s / 2.18 mm |
| 080033 | @12 | 1.00 | (-0.00253, 0.00306, -0.00283) | (0.046, 0.108, 0.313) | 0.049 / 0.0049 / 1.53 |
| 080033 | @24 | 1.29 | (-0.00317, 0.00423, -0.00194) | (0.137, 0.115, 0.236) | 0.059 / 0.0043 / 1.56 |
| 080033 | @36 | 0.76 | (-0.00369, 0.00312, -0.00123) | (0.073, 0.180, 0.234) | 0.072 / 0.0042 / 1.70 |
| 080033 | @48 | 1.25 | (-0.00110, 0.00306, -0.00229) | (0.027, 0.180, 0.211) | 0.081 / 0.0048 / 2.12 |
| 080033 | @60 | 1.04 | (0.00277, 0.00573, 0.00002) | (-0.116, 0.107, 0.306) | 0.242 / 0.0056 / 2.85 |
| 080033 | @72 | 0.48 | (-0.00389, 0.00423, -0.00191) | (-0.065, 0.084, 0.312) | 0.229 / 0.0063 / 3.18 |
| 075947 | @0 | 0.66 | (-0.00127, 0.00035, -0.00319) | (-0.071, 0.030, 0.324) | 0.058 / 0.0051 / 2.10 |
| 075947 | @12 | 0.18 | (0.00075, -0.00137, -0.00124) | (-0.023, 0.033, 0.317) | 0.069 / 0.0049 / 1.63 |
| 075947 | @24 | 0.76 | (-0.00283, 0.00685, -0.00146) | (-0.049, 0.063, 0.326) | 0.085 / 0.0051 / 1.59 |
| 155419 | @0 | 2.79 | (-0.00710, -0.00776, 0.00010) | (0.076, 0.249, 0.166) | 0.097 / 0.0056 / 2.29 |

Two features stand out. First, b_a_z is about +0.31 to +0.33 m/s^2 on
every window of the two loop takes (and +0.17 on 155419), roughly 6x
its prior sigma of 0.05 — the data pulls it there against the prior,
and its physical meaning (true bias vs accel scale error vs gravity
magnitude error) is not separated by this fit. Second, the other
accel-bias components wander: on 080033 b_a_x ranges -0.116 to +0.137
m/s^2 across windows of the same take.

Observability tests (thresholds 2.0 deg / 0.02 m/s^2 / 0.005 rad/s):

- (a1) perturbed start (gravity +10 deg, zero biases): PASSES on 11/11
  windows — the perturbed run converges to the identical minimum
  (max delta gravity 0.0001 deg, delta b_a 0.00001 m/s^2). The
  optimizer is not the problem; the minimum is unique given the priors.
- (a2) shifted windows: gravity PASSES (median delta 0.14 deg, max
  0.55 deg) and gyro bias passes (median 0.0027 rad/s), but accel bias
  FAILS: median |delta b_a| 0.040 m/s^2, max 0.105, over the 0.02
  tolerance on 8 of 9 real shifted windows (the 155419 shifted entry
  is the same single window, trivially identical). Overlapping windows
  of the same take disagree on b_a by up to 0.10 m/s^2.
- (b) weakened bias priors (sigma_bg 0.01 -> 0.04, sigma_ba 0.05 ->
  0.20): gravity PASSES (median delta 0.23 deg, max 0.50 deg) and gyro
  bias passes (median 0.0005 rad/s), but accel bias FAILS decisively:
  median |delta b_a| 0.072 m/s^2, range 0.051-0.107, over tolerance on
  all 11 windows. The data residuals improve under weak priors
  (velocity RMS roughly halves), so the data prefers a larger b_a and
  the prior was holding it back: the prior, not the data, chooses the
  accel bias.

Verdict: STOP at the null per the pre-specified rules — (a) fails on
its accel-bias leg and (b) fails on accel bias. Tests (c) and (d) were
NOT run (the gate failed first; the code for them exists in
step2_joint.py holdout mode but was not executed). The fallback
smoother was NOT built: that would be a new method with new choices
after seeing results, which the task forbids improvising.

Mechanism check (measured, not assumed): the failure is NOT explained
by "rotations mostly about the gravity axis". On 080033 the camera
z-axis spreads 58 deg across keyframes and the median pair-rotation
axis has |axis . g| = 0.26 (rotations mostly about horizontal axes),
yet b_a still wanders. What the numbers support: with a free velocity
at every keyframe and visual steps of only millimetres, the
preintegration residuals are satisfied by a wide range of (b_a,
velocity-sequence) combinations, so the data pins the gravity
direction (stable to ~0.5 deg under every perturbation tried) but
leaves the accel bias uncertain at the 0.05-0.10 m/s^2 level — the
prior picks the point on a shallow valley. The fitted gravity
direction is stable; the joint (gravity, accel-bias) split is not
determined by these residuals, and an inertial initialization needs
both, so the joint initialization does not pass.
