# Visual-pose-constrained inertial initialization — final verdict: null

Date: 2026-11-10. Work area: `analysis/audit-imu-init/`.
Scripts: `common.py`, `step1_bias.py`, `step2_joint.py`, `compare_tests.py`.
Bags read read-only. Nothing uploaded to GitHub. No message sent to
anyone. All choices (priors, windows, test thresholds) were written in
`audit-imu-init/PROGRESS.md` before any fit was run, and none was changed after seeing
results. Code written by hand; no browser use.

## Verdict up front

**The joint visual-pose-constrained initialization does NOT pass its
observability gate, so there is no usable inertial initialization from
this branch.** The gyro bias from visual rotations (step 1) works well
and the gravity direction from the joint fit is stable to about half a
degree, but the accel bias is not determined by the data: it moves by
0.05–0.11 m/s² when the bias priors are weakened 4×, and overlapping
windows of the same take disagree on it by up to 0.10 m/s². The prior,
not the data, chooses the accel bias. An inertial initialization needs
both gravity and accel bias, so the joint estimate cannot be trusted
as an initialization. Per the pre-specified stop rule, tests (c) and
(d) were not run and the fallback smoother was not built.

- **Step 1 works.** The gyro bias fitted against ORB relative rotations
  is 0.004–0.011 rad/s per take, both initializations converge to the
  same value, and the residual rotation error drops to 0.08–0.11 deg
  median. On 155419 the fitted bias (0.01051 rad/s) decisively rejects
  the 0.1911 rad/s per-take median: path rotation stays at 20.53 deg
  against the ORB total of 20.81 deg, where the median bias collapses
  it to 7.05 deg.
- **Gravity direction is stable.** Across perturbed starts, shifted
  windows, and 4× weaker bias priors, the fitted gravity direction
  moves by at most 0.55 deg (median 0.14–0.23 deg). It sits 0.2–1.3 deg
  from the preint stillest-window direction on the loop takes and
  2.79 deg away on 155419.
- **Accel bias is unobservable at the level needed.** Baseline fits put
  its z-component stays positive on every window,
  +0.21 to +0.33 m/s² on 080033 and +0.31 to +0.33 on 075947
  (6× its prior sigma), yet weakening the prior moves the
  whole vector by a median 0.072 m/s², and its x-component wanders over
  a 0.25 m/s² range across windows of 080033 alone. Whether that
  +0.32 m/s² is a true bias, an accel scale error, or a gravity
  magnitude error is not separated by this fit.
- **Tests (c) and (d) not run.** The (a)/(b) gate failed first, so the
  withheld-motion test and the constant-velocity-baseline comparison
  were not executed, as the task's stop rule requires. No claim is made
  about them either way.

## Method, as run

### Calibration and frames (verified from the bags)

The `tf/ref_0` string topics were read directly from all three bags
(identical values on all three). Gyro and accel tf: rotation identity,
translation (-0.030220, 0.007400, 0.016020) m. Depth tf: identity, zero
translation — depth is the reference frame. Colour tf: rotation within
0.0039 of identity, translation (-0.059037, 0.000078, 0.000128) m (the
5.9 cm colour baseline). `imu_intrinsic` carries identity scale and
zero bias/noise variances (no information there).

Transform used, stated exactly:

- Rotation between IMU and depth frames is the identity: R_imu = R_depth.
  The ORB poses are depth-frame absolute poses (depth used as-is), so
  visual relative rotations need no rotation change.
- Translation t = (-0.030220, 0.007400, 0.016020) m is the IMU origin
  expressed in the depth frame (bag tf topic, confirmed by D455F.yaml
  `IMU.T_b_c1`, body-to-camera). Absolute visual poses are expressed at
  the IMU origin as R_imu,k = R_cam,k and
  p_imu,k = p_cam,k - R_cam,k @ t (ORB-SLAM3 T_b_c1 convention).
- Step 1 (rotation only) does NOT apply the translation: R is unchanged
  by it, and the visual poses stay at the camera origin for that step.
  Step 2 applies the translation as above, so visual poses are expressed
  at the IMU origin.
- Note on preint.py: its `conjugate_to_depth` used the opposite sign
  (t_depth = t_imu + t - R t, versus t_imu + R t - t under the convention
  above). The per-pair difference is at most ~1.4 mm, and the residuals
  here use position differences, where the lever-arm term is at most
  ~0.7 mm per pair. The preint verdicts are unaffected at the scale of
  their conclusions.

### Shared choices

- Takes: 20261007_080033, 20261007_075947, 20260926_155419 (the three
  with ORB trajectories). No new ORB-SLAM3 runs.
- Keyframes: stride-5 colour frames, the same pairs as the preint work:
  88 pairs (080033), 46 pairs (075947), 10 pairs (155419). ORB pose
  lists have one pose per paired colour frame (440 + 1 interpolated,
  235, 54 + 1 interpolated), so index alignment holds throughout.
- Time sync: integration windows shifted -50 ms against the colour
  interval, as in preint.py. Gyro convention R = Exp(-integral of
  (omega - b_g) dt), trapezoid-integrated incrementally.
- Optimizer: hand-rolled Levenberg-Marquardt, numpy only (scipy is not
  installed in the venv). Preintegration machinery (loading, rotation
  helpers, D-table) reused from preint.py by import; the joint fit uses
  a new gravity-free preintegrator (accel bias-corrected in the body
  frame, no gravity subtracted inside the integration).

### Step 1 — gyro bias from visual relative rotations

Per pair, the ORB relative rotation R_vis = R_j^T R_i is compared with
the gyro-integrated relative rotation as a function of a constant gyro
bias b. A 3-parameter LM fit minimizes the rotation-vector residuals
rotvec(R_vis^T R_gyro(b)), plain least squares, no robust loss, no
outlier rejection. Primary initialization b = 0; second run from the
per-take median bias as a convergence check.

### Step 2 — joint refinement

Windows of 12 consecutive stride-5 pairs (60 colour frames, 2.0 s):
080033 uses 7 windows (pairs 0–83), 075947 uses 3 (pairs 0–35),
155419 is a single 10-pair window (1.67 s). State per window: gravity
direction (2 tangent parameters at the prior-mean direction, |g| fixed
at 9.81 m/s²), gyro bias (3), accel bias (3), and one velocity vector
per keyframe boundary (13, or 11 for 155419). Constant biases over each
window only; visual rotations and positions held fixed.

Residuals per pair (standard preintegration residuals): rotation
Log(R_vis^T R_pred); velocity R_i^T (v_j - v_i - g dt) - dvel;
position R_i^T (p_j - p_i - v_i dt - 0.5 g dt²) - dpos, with positions
at the IMU origin. Residual weights: sigma_rot = 0.003 rad,
sigma_vel = 0.01 m/s, sigma_pos = 0.003 m. Priors (baseline): gyro bias
mean = step-1 fitted bias, sigma = 0.01 rad/s/axis; accel bias mean 0,
sigma = 0.05 m/s²/axis; gravity direction mean = preint stillest-window
direction carried into the world frame with the ORB pose at the window's
first keyframe (gyro-chained from the preint window start with the
D-table using the step-1 bias, not the median bias — the median bias
was already measured to be wrong), sigma = 5 deg; velocities no prior,
initialized from visual finite differences. LM forward-difference
Jacobian, steps 1e-4 rad / 1e-5 rad/s / 1e-4 m/s² / 1e-4 m/s,
lambda0 = 1e-3, max 50 iterations.

### Observability thresholds (fixed before running)

- (a1) perturbed start (gravity tilted 10 deg about world x, zero
  biases) must match the primary run within 2.0 deg / 0.02 m/s² /
  0.005 rad/s on at least 9 of 11 windows, and (a2) shifted windows
  (starts at pairs 6, 18, 30, ...) must match the nearest primary
  window within the same tolerances at the median.
- (b) bias priors weakened 4× (sigma_bg 0.01 -> 0.04 rad/s, sigma_ba
  0.05 -> 0.20 m/s²) must move the estimates by at most 2.0 deg /
  0.02 m/s² at the median.
- (c) fit on the first window, velocity-only refit on withheld windows,
  frozen parameters vs baseline parameters; (d) short predictions
  (horizons 1/2/4/8 pairs) vs a constant-velocity baseline from the same
  preceding visual frames. Both not run: the gate failed first.

## Step 1 results: gyro bias per take

Both initializations converge to the same bias on all three takes
(3 LM iterations each). Residual rotation error is the angle between
the ORB and gyro relative rotations per pair.

| take | fitted b_g (rad/s) | \|b_g\| (rad/s) | per-take median bias (rad/s) | residual median deg: b=0 / fitted / median bias | path rotation deg: fitted / ORB total / median bias |
|---|---|---|---|---|---|
| 080033 | (-0.00182, 0.00367, -0.00169) | 0.00443 | 0.01685 | 0.0838 / 0.0772 / 0.1845 | 159.54 / 160.02 / 159.36 |
| 075947 | (-0.00191, 0.00274, -0.00210) | 0.00394 | 0.01630 | 0.0995 / 0.0971 / 0.1824 | 40.23 / 38.98 / 39.82 |
| 155419 | (-0.00710, -0.00775, 0.00010) | 0.01051 | 0.19109 | 0.1553 / 0.1119 / 1.8611 | 20.53 / 20.81 / 7.05 |

The 155419 check is decisive: the fitted bias is far below the
0.1911 rad/s median, the path rotation stays near the ORB total, and
the residual median drops from 1.8611 deg (median bias) to 0.1119 deg.
The per-take median gyro vector is not sensor bias — even b = 0 beats
it on every take (median 0.084 / 0.100 / 0.155 deg). The gyro bias is
genuinely small, 0.004–0.011 rad/s, and the step-1 fitted bias is the
right gyro-bias prior mean for step 2. (RMS residuals: fitted
0.232 / 0.133 / 0.168 deg; max 1.097 / 0.297 / 0.345 deg. The 080033
max of ~1.1 deg is one outlying pair; no outlier rejection was applied,
as pre-specified.)

## Step 2 results: joint estimates per take

Baseline fits (all windows converged in 4–5 LM iterations). Gravity
tilt is measured from the preint stillest-window direction. Full
vectors, costs, and per-type residuals are in
`runs/<take>_step2_baseline.json`.

| take | window | g tilt (deg) | b_g (rad/s) | b_a (m/s²) | rms rot / vel / pos |
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

The data residuals at the fit are small on every window (rotation
0.02–0.24 deg, velocity ~0.005 m/s, position 1.5–3.2 mm) — the fit fits.
But the fitted accel bias is not stable: on 080033 its x-component
ranges from -0.116 to +0.137 m/s² across windows of the same take
(0.25 m/s² range), while its z-component sits at +0.31 to +0.33 m/s²
on every window of both loop takes, about 6× its prior sigma. The
late 080033 windows (@60, @72) fit rotation noticeably worse
(0.23–0.24 deg vs 0.02–0.08 deg elsewhere); the cause was not chased,
because the gate had already failed.

## Observability test results

Full per-window numbers are in `runs/observability_comparison.json`.

- **(a1) perturbed start: PASSES, 11/11 windows.** Starting from a
  gravity direction tilted 10 deg with zero biases, every window
  converges to the identical minimum (max delta gravity 0.0001 deg,
  delta b_a 0.00001 m/s², delta b_g 0.000000 rad/s). The optimizer is
  not the problem; given the priors, the minimum is unique.
- **(a2) shifted windows: FAILS on accel bias.** Gravity passes
  (median delta 0.14 deg, max 0.55 deg) and gyro bias passes (median
  0.0027 rad/s), but accel bias fails: median |delta b_a| 0.040 m/s²,
  max 0.105 m/s², over the 0.02 m/s² tolerance on 8 of 9 real shifted
  windows. (The 155419 shifted entry is the same single window, so it
  is trivially identical and is not counted.) Overlapping windows of
  the same take disagree on b_a by up to 0.10 m/s².
- **(b) weakened bias priors: FAILS on accel bias.** Gravity passes
  (median delta 0.23 deg, max 0.50 deg) and gyro bias passes (median
  0.0005 rad/s, max 0.0014 rad/s), but accel bias fails decisively:
  median |delta b_a| 0.072 m/s², range 0.051–0.107 m/s², over tolerance
  on all 11 windows. Under weak priors the velocity residuals roughly
  halve, so the data prefers a larger accel bias and the prior was
  holding it back. The prior, not the data, chooses the accel bias.
- **(c) withheld motion: NOT RUN.** The gate failed first.
- **(d) constant-velocity baseline: NOT RUN.** The gate failed first.
  No prediction comparison is claimed.

## Final verdict: null, with mechanism

The joint (gravity direction, accel bias) split is not determined by
preintegration residuals against these visual trajectories. The
gravity direction is pinned to about half a degree — it enters every
pair's residual coherently — but the accel bias sits on a shallow
valley: the data is satisfied about equally well by accel biases
differing by 0.05–0.10 m/s², with the free per-keyframe velocities
absorbing the difference, so the prior picks the point. The fitted
+0.32 m/s² z-component is stable across takes and windows, but what it
is (true bias, accel scale error of ~3%, or gravity magnitude error)
is not separated by this fit, and the perpendicular components wander
by up to 0.25 m/s² within one take. Since an inertial initialization
needs the accel bias as well as gravity, the joint initialization
does not pass and must not be used as a prior in the VO chain.

One candidate mechanism was checked and ruled out: the failure is not
because the rotations are mostly about the gravity axis. On 080033
the camera z-axis spreads 58 deg across keyframes and the median
pair-rotation axis has |axis · g| = 0.26 (rotations mostly about
horizontal axes), yet the accel bias still wanders. (075947: z-spread
16 deg, median |axis · g| 0.37. 155419: z-spread 19 deg, median
|axis · g| 0.87 — that take is mostly yaw about gravity, as expected
for a rotation in place.) The measured mechanism is the one above:
millimetre-scale visual motion with a free velocity at every keyframe
leaves the accel bias weakly constrained, whatever the rotation axes.

## Open design question

The fallback named in the task — a visually anchored sliding-window
smoother used as a short-term translation prior between visual
updates — was not built, because building it now would mean inventing
a new method and new test thresholds after seeing these results. The
exact question, for a separately pre-specified task, is: can a
smoother that does NOT try to separate gravity direction from accel
bias — holding gravity at the fitted direction (stable to ~0.5 deg)
and using the fitted (gravity, accel-bias) combination as one
effective specific-force model — produce short-horizon position
predictions that beat a constant-velocity baseline on withheld motion?
The deciding test is the test (d) design already written here
(horizons 1/2/4/8 pairs, velocity from the same preceding visual
frames, future visual poses excluded), and the code for it exists in
`step2_joint.py` holdout mode, but it was not run under this task
because the (a)/(b) gate failed first. A second, separate question in
the same task: the fitted accel-bias z-component of about +0.32 m/s²
is 6× its prior sigma on every window — is the sigma = 0.05 m/s²
accel-bias prior simply wrong for this sensor, and would a bias prior
grounded in a measured BMI085 turn-on bias change the (b) verdict?
That also must be settled from a measurement, not by tuning after
seeing these fits.

## Files

All under `analysis/audit-imu-init/`:

- `audit-imu-init-summary.md` — this file.
- `audit-imu-init/PROGRESS.md` — checkpoint log, including the full pre-specification
  (written before any fit), the step-1 results, and the step-2 results
  with the observability verdicts.
- `common.py` — shared loading, gravity-free preintegration, and the
  numpy-only LM optimizer.
- `step1_bias.py` — step-1 gyro-bias fit.
- `step2_joint.py` — step-2 joint fit (baseline / perturbed /
  weakprior / shifted modes) and the holdout code for tests (c) and (d)
  (written, not executed).
- `compare_tests.py` — observability comparison for tests (a1), (a2),
  (b).
- `runs/<take>_step1.json` — step-1 fits.
- `runs/<take>_step2_baseline.json`, `runs/<take>_step2_perturbed.json`,
  `runs/<take>_step2_weakprior.json`, `runs/<take>_step2_shifted.json`
  — step-2 fits per mode.
- `runs/observability_comparison.json` — test (a1), (a2), (b) numbers.
