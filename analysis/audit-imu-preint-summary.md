# Full IMU pre-integration prior — final verdict: does NOT help

Date: 2026-11-10. Work area: `analysis/audit-imu-preint/`.
Script: `preint.py`. One bounded run per take, no parameter tuning after
seeing results, no new method invented. Bags read read-only. Nothing
uploaded to GitHub. No message sent to anyone.

## Verdict up front

**The full pre-integration prior does NOT help the VO chain, and it
was not tested as a prior.** Step 2 (standalone evaluation) fails the
gate in the task: rotation only ties gyro-only, and translation has
no usable signal. Per the stop rule, step 3 (VO-chain prior test)
was not run. Do not use this pre-integration as a prior in the VO
chain as it stands.

- **Rotation: tie with gyro-only, not better.** The per-pair
  rotations are the same gyro integration as the gyro-prior work
  (080033 median pair rotation 1.1735 deg, path rotation 159.36 deg
  against 159.35 deg there). End rotation on 080033 reads better
  than the gyro-only replace mode (13.82 deg against 18.92 deg)
  only because that chain held the pose on two PnP-failed pairs
  (pairs 71 and 75, carrying 1.02 and 7.87 deg of gyro rotation);
  the integration itself adds nothing to rotation over the gyro
  prior.
- **Translation: no usable signal.** The velocity-propagated chain
  ends 0.30 m to 80.43 m from the truth on takes where the truth is
  about zero. The zero-velocity-reset chain ends closer (0.016 m to
  1.27 m) but its per-step translation error against ORB is as large
  as the step itself, so the good end numbers are partly
  cancellation, not a per-step signal.
- **Limiting factor: the gravity direction.** No take has a truly
  still stretch to estimate gravity from (stillest-window median
  gyro 0.036–0.076 rad/s; 080033 and 155419 have no window below
  the 0.05 rad/s still threshold at all). On 080033 the residual is
  about 1.0 m/s², i.e. the gravity direction is off by about 6 deg.
  That constant leak integrates twice: velocity grows linearly
  (reconstructed max 15.69 m/s on 080033) and position grows
  quadratically (80.43 m end error). Even on the cleanest take
  (102631c, median |dvel| 0.016 m/s per pair) the propagated chain
  ends 0.30 m out on a camera that did not move.
- **Second failure, on short takes: the bias convention.** On
  155419 (1.8 s rotation in place) the per-take median gyro vector
  (0.1911 rad/s) is the real rotation, not sensor bias, so
  subtracting it removes most of the signal: path rotation comes
  out 7.05 deg against a gyro total of 23.0 deg and an ORB total of
  23.2 deg. Any later work using the per-take median bias on such
  takes will hit the same failure. This was not re-run with a
  different bias (stop rule).
- **The -50 ms sync correction is not the limiter.** Shift-0
  sensitivity: end rotation moves by at most 0.75 deg and end
  translation by at most 0.44 m — small next to the gravity-driven
  errors on every take.

## Method, as run

Per stride-5 pair (the same pairs the VO uses): gyro
trapezoid-integrated with the per-take median gyro vector removed
as bias, inverse rotation convention R = Exp(-integral omega dt),
integration window shifted -50 ms against the colour interval.
Accel linearly interpolated onto the gyro sample grid (gyro about
200 Hz, accel about 100 Hz), rotated into the pair-start frame
with the incrementally integrated gyro rotation, gravity
subtracted, trapezoid integrated to velocity and position deltas.
Velocity propagated across pairs from zero at take start. Each
pair pose conjugated into the depth frame with the IMU-to-depth
translation (-0.030220, 0.007400, 0.016020), taken as the IMU
origin expressed in the depth frame with aligned axes (R unchanged,
t to t + t_imu - R t_imu). Chained exactly like the odometry chain,
A[i+1] = A[i] @ inv(T).

Gravity: minus the mean accelerometer vector over the take's
stillest 1-second window (sliding 0.25 s steps, ranked by median
bias-removed gyro magnitude), rotated into each pair-start frame
with the gyro-chained rotation from the window start. Gravity
magnitudes came out 9.711–9.772 m/s². Windows used: 080033
0.00–1.00 s, 075947 2.75–3.75 s, 155419 0.75–1.75 s, 102631c
2.25–3.25 s, 174745 2.75–3.75 s, 173250 4.75–5.75 s. No take had a
still first second.

Rotation cross-check (incremental composition against Exp(-rv)):
median 0.0002–0.0019 deg per take, max 0.141 deg (080033). The
rotation integration is internally consistent.

## Per-take standalone results

End rotation and end translation are the chained pre-integration
against its start pose. Truth is about zero end pose on the three
loop takes and the two still takes. ORB end is the ORB-SLAM3
RGB-D-only end pose from the same start. Per-step errors are
medians against the ORB relative poses between the same frames.
All numbers below were checked again against the
per-take JSONs (end poses, per-step errors, and the zero-reset
per-step errors recomputed independently from the stored pairs
and ORB poses: 0.0142 / 0.0076 / 0.0212 m).

| take | preint end rot (deg) | preint end trans, primary (m) | preint end trans, zero-reset (m) | reference end rot / trans | end rot error vs ref (deg) | end trans error vs ref (m) | per-step rot err vs ref, median (deg) | per-step trans err vs ref, median (m) |
|---|---|---|---|---|---|---|---|---|
| 20261030_174745 (still) | 1.58 | 1.308 | 0.053 | truth 0 / 0 | 1.58 | 1.308 | — (per-step rot magnitude median 0.40 deg) | — (per-step trans magnitude median 0.025 m) |
| 20260926_102631c (still) | 0.65 | 0.299 | 0.016 | truth 0 / 0 | 0.65 | 0.299 | — (per-step rot magnitude median 0.21 deg) | — (per-step trans magnitude median 0.014 m) |
| 20261007_080033 (loop) | 13.82 | 80.425 | 1.270 | ORB 13.00 deg / 0.061 m | 13.00 | 80.412 | 0.185 | 0.732 |
| 20261007_075947 (loop) | 10.88 | 3.475 | 0.022 | ORB 8.84 deg / 0.188 m | 6.90 | 3.652 | 0.182 | 0.088 |
| 20260926_155419 (loop) | 1.42 | 1.097 | 0.078 | ORB 20.26 deg / 0.135 m | 19.31 | 1.019 | 1.861 | 0.118 |
| 20261026_173250 (ordinary) | 12.61 | 14.643 | 0.121 | none (VO 63.47 deg / 1.611 m, not a reference) | 58.35 vs VO | 14.978 vs VO | 1.054 vs VO | 0.301 vs VO |

Loop-closure errors (end pose against start, truth about zero):
080033 13.82 deg / 80.43 m primary (1.27 m zero-reset), 075947
10.88 deg / 3.48 m primary (0.022 m zero-reset), 155419 1.42 deg /
1.10 m primary (0.078 m zero-reset). The rotation loop errors are
the same order as ORB's own (13.00, 8.84, 20.26 deg); the
translation loop errors are 5 to 1000 times ORB's (0.061, 0.188,
0.135 m) on the primary chain.

## Translation: the mechanism, measured

The velocity-propagated chain diverges on every take because the
per-pair velocity change carries a systematic residual
acceleration, and velocity propagation integrates it twice:

| take | per-pair \|dvel\| median (m/s) | per-pair \|dpos\| median (mm) | max \|v\| on chain (m/s) | primary end trans (m) |
|---|---|---|---|---|
| 174745 | 0.029 | 2.5 | 0.68 | 1.308 |
| 102631c | 0.016 | 1.5 | 0.15 | 0.299 |
| 080033 | 0.172 | 14.3 | 15.69 | 80.425 |
| 075947 | 0.034 | 3.1 | 0.69 | 3.475 |
| 155419 | 0.119 | 9.7 | 0.99 | 1.097 |
| 173250 | 0.061 | 5.4 | 2.11 | 14.643 |

The zero-velocity-reset chain bounds the damage (end translation
0.016 m on 102631c, 0.022 m on 075947) but it is not a motion
model — it assumes the camera stops at every pair — and its
per-step translation error against ORB is 0.0142 m on 080033
(ORB step median 0.0040 m), 0.0076 m on 075947 (ORB step median
0.0049 m), and 0.0213 m on 155419 (ORB step median 0.0167 m).
The error is as large as the step it claims to measure, so the
per-pair position deltas do not carry a usable translation signal
either. The good end numbers on 075947 and 102631c come partly
from errors cancelling around the loop, not from accurate steps.

## Step 3: not run (stop rule)

Step 2 fails the gate (rotation ties gyro-only; no usable
translation signal), so the VO-chain prior test was not run. The
gyro-only result stands as the best IMU prior for rotation
(86.33 to 0.21 deg still, 67.62 to 18.92 deg loop, translation
untouched), and the pre-integration adds nothing to it.

## What would be needed for this branch to work

A gravity direction good to well under a degree on a take with no
still stretch, and a velocity treatment that does not let a
constant residual acceleration integrate twice — e.g. a proper
bias/velocity state with visual constraints, not open-loop
propagation from zero. Both are a different, larger piece of work
than a per-pair prior, and neither was attempted here (stop rule).
The per-take median gyro bias also needs replacing on short
rotation-dominated takes (155419), where it eats the signal.

## Reference gaps, stated plainly

- No ORB-SLAM3 RGB-D-only trajectory exists for 102631c, 174745,
  or 173250. For the two still takes the reference is the truth
  that the camera did not move, which is solid. For 173250 there
  is no absolute reference at all: the VO chain (63.47 deg /
  1.611 m end) is a consistency check only, and it is itself the
  drifting chain this work is trying to fix, so the 173250
  per-step numbers against VO measure disagreement with a
  drifting reference, not pre-integration error.
- ORB trajectories for 080033 and 075947 were converted to
  per-frame absolute poses with the existing adapter script from
  the existing trajectory files (no new ORB-SLAM3 run). Their
  end-to-start values reproduce the known numbers (080033 0.061 m,
  075947 0.188 m). 155419 uses the existing orb-maps trajectory
  (ORB path rotation recomputed at 23.32 deg).

## Files

All under `analysis/audit-imu-preint/`:

- `audit-imu-preint-summary.md` — this file.
- `audit-imu-preint/STEP2_VERDICT.md` — the step-2 verdict and per-take
  table (all numbers checked again).
- `audit-imu-preint/PROGRESS.md` — checkpoint log, one entry per take, with bag
  resolution and calibration probes.
- `audit-imu-preint/preint.py` — the step-1 script (integration, chaining,
  comparison).
- `audit-imu-preint/runs/20261007_080033_preint.json`, `audit-imu-preint/runs/20261007_075947_preint.json`,
  `audit-imu-preint/runs/20260926_155419_preint.json`, `audit-imu-preint/runs/20260926_102631c_preint.json`,
  `audit-imu-preint/runs/20261030_174745_preint.json`, `audit-imu-preint/runs/20261026_173250_preint.json`
  — per-take JSONs: per-pair deltas (R, t in IMU and depth frames,
  velocity/position deltas, zero-shift variants), chained absolute
  poses (primary, zero-reset, shift-0), gravity and bias records,
  comparison numbers.
- `audit-imu-preint/runs/080033_orb_poses.json`, `audit-imu-preint/runs/075947_orb_poses.json` —
  adapter output from the existing ORB trajectories.
- `audit-imu-preint/runs/173250_vo_poses.json` and
  `audit-imu-preint/runs/173250_vo_orthomosaic.png` — the VO baseline run once for
  the 173250 consistency check.
