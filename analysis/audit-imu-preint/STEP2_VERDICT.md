# Step-2 verdict: standalone IMU pre-integration

Date: 2026-11-10. Work area: `analysis/audit-imu-preint/`.
Script: `preint.py`. One bounded run per take, no parameter tuning after
seeing results, no new method invented. Bags read read-only. Nothing
uploaded to GitHub. No message sent to anyone.

## Verdict

**No. The pre-integration does not pass step 2.**

- Rotation: it ties gyro-only, it does not beat it. The per-pair
  rotations are the same gyro integration as the gyro-prior work
  (median pair rotation 1.1735 deg on 080033, identical to four
  decimals; path rotation 159.36 deg against 159.35 deg). The end
  rotation on 080033 reads better than the gyro-only replace mode
  (13.82 deg against 18.92 deg) only because that chain held the
  pose on two PnP-failed pairs; the integration itself adds nothing
  to rotation over the gyro prior.
- Translation: there is no usable signal. The velocity-propagated
  chain ends 0.30 m to 80.43 m from the truth on takes where the
  truth is about zero. The zero-velocity-reset chain is better at
  the end (0.016 m to 1.27 m) but its per-step translation error
  against ORB is as large as the step itself on two of the three
  ORB takes, so the good end numbers are partly cancellation, not
  a per-step signal.
- Do not use this pre-integration as a prior in the VO chain as it
  stands. The translation branch needs a better gravity and
  velocity treatment first (see the mechanism below).

## Per-take table

End rotation and end translation are the chained pre-integration
against its start pose. Truth is about zero end pose on the three
loop takes and the two still takes. ORB end is the ORB-SLAM3
RGB-D-only end pose from the same start. Per-step errors are
medians against the ORB relative poses between the same frames.

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

## Rotation against gyro-only, take by take

- 080033 is the only take with a gyro-only number (replace mode
  18.92 deg end rotation, still take 085611 0.21 deg). Pre-integration
  gives 13.82 deg. The gap is the two held pairs, not the
  integration: the per-pair rotations match the gyro-prior work
  exactly (median 1.1735 deg, path 159.36 deg against 159.35 deg),
  and the gyro-prior chain held the pose on pairs 71 and 75, which
  carry 1.02 and 7.87 deg of gyro rotation. Rotation verdict: tie.
- 075947 has no gyro-only number. Pre-integration ends 10.88 deg
  against ORB's 8.84 deg, with a 6.90 deg end error against ORB.
  Per-step rotation error against ORB is median 0.182 deg.
- 155419 has no gyro-only number, and the comparison is void: the
  specified per-take-median gyro bias (0.1911 rad/s) is the real
  rotation of this 1.8 s rotation-in-place take, not sensor bias,
  so the bias subtraction removes most of the signal. Path rotation
  comes out 7.05 deg against a gyro total of 23.0 deg and an ORB
  total of 23.2 deg. This is a failure of the bias convention on
  short rotation-dominated takes, stated here, not re-run (stop
  rule). Any later work that uses the per-take median bias on such
  takes will hit the same failure.
- The still takes (174745 1.58 deg, 102631c 0.65 deg) cannot be
  compared with the gyro-only still number (0.21 deg on 085611):
  different takes, and both of these cameras drift (per-step
  rotation magnitude median 0.40 and 0.21 deg, real drift confirmed
  by vision in the earlier IMU work).

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

On 080033 the residual is about 1.0 m/s^2, i.e. the gravity
direction is off by about 6 deg. The gravity estimate comes from
the take's stillest 1-second window, and no take has a truly still
window: the stillest-window median gyro magnitudes are 0.036
(102631c), 0.036 (174745), 0.049 (173250), 0.051 (075947), 0.072
(080033), 0.076 (155419) rad/s, and 080033 and 155419 have no
window below the 0.05 rad/s still threshold at all. A gravity
direction that is off by a few degrees leaks a constant
acceleration into every pair, velocity grows linearly, and position
grows quadratically. Even on the cleanest take (102631c, median
|dvel| 0.016 m/s per pair) the propagated chain ends 0.30 m out
on a camera that did not move.

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

## Sync sensitivity (0 ms shift, one number per take)

End rotation / end translation with the shift at 0 ms, against
the -50 ms primary: 174745 1.66 deg / 1.329 m (was 1.58 / 1.308),
102631c 0.70 / 0.272 (was 0.65 / 0.299), 080033 14.01 / 80.87
(was 13.82 / 80.42), 075947 10.13 / 3.39 (was 10.88 / 3.48),
155419 1.14 / 1.13 (was 1.42 / 1.10), 173250 12.46 / 14.47
(was 12.61 / 14.64). The shift changes the end rotation by at
most 0.75 deg and the end translation by at most 0.44 m — small
next to the gravity-driven errors on every take. The -50 ms
correction is not what limits this branch.

## Method, as run

- Per stride-5 pair, gyro trapezoid-integrated with the per-take
  median gyro vector removed, inverse rotation convention
  R = Exp(-integral omega dt), integration window shifted -50 ms
  against the colour interval. Accel linearly interpolated onto
  the gyro sample grid (gyro about 200 Hz, accel about 100 Hz),
  rotated into the pair-start frame with the incrementally
  integrated gyro rotation, gravity subtracted, trapezoid
  integrated to velocity and position deltas. Velocity propagated
  across pairs from zero at take start. Each pair pose conjugated
  into the depth frame with the IMU-to-depth translation
  (-0.030220, 0.007400, 0.016020), taken as the IMU origin
  expressed in the depth frame with aligned axes (R unchanged,
  t to t + t_imu - R t_imu). Chained exactly like the odometry
  chain, A[i+1] = A[i] @ inv(T).
- Gravity: minus the mean accelerometer vector over the take's
  stillest 1-second window (sliding 0.25 s steps, ranked by
  median bias-removed gyro magnitude), rotated into each
  pair-start frame with the gyro-chained rotation from the window
  start. Gravity magnitudes came out 9.711-9.772 m/s^2.
  No take had a still first second; the windows used were
  080033 0.00-1.00 s, 075947 2.75-3.75 s, 155419 0.75-1.75 s,
  102631c 2.25-3.25 s, 174745 2.75-3.75 s, 173250 4.75-5.75 s.
- Rotation cross-check (incremental composition against
  Exp(-rv)): median 0.0002-0.0019 deg per take, max 0.141 deg
  (080033). The rotation integration is internally consistent.
- 173250 was picked as the ordinary Oct-26 take because it is the
  noisiest take in the inlier-rate table (non-rigid signal, pooled
  median residual 0.412 px against a 0.185 px control).

## Reference gaps, stated plainly

- No ORB-SLAM3 RGB-D-only trajectory exists for 102631c, 174745,
  or 173250. For the two still takes the reference is the truth
  that the camera did not move, which is solid. For 173250 there
  is no absolute reference at all: the VO chain (63.47 deg /
  1.611 m end) is a consistency check only, and it is itself the
  drifting chain this work is trying to fix, so the 173250
  per-step numbers against VO (rotation median 1.054 deg,
  translation median 0.301 m) measure disagreement with a
  drifting reference, not pre-integration error.
- ORB trajectories for 080033 and 075947 were converted to
  per-frame absolute poses with the existing adapter script from
  the existing trajectory files (no new ORB-SLAM3 run). Their
  end-to-start values reproduce the known numbers (080033 0.061 m,
  075947 0.188 m).

## Files

All under `analysis/audit-imu-preint/`:

- `preint.py` — the step-1 script (integration, chaining,
  comparison).
- `STEP2_VERDICT.md` — this file.
- `PROGRESS.md` — checkpoint log, one entry per take.
- `runs/20261007_080033_preint.json`, `runs/20261007_075947_preint.json`,
  `runs/20260926_155419_preint.json`, `runs/20260926_102631c_preint.json`,
  `runs/20261030_174745_preint.json`, `runs/20261026_173250_preint.json`
  — per-take JSONs: per-pair deltas (R, t in IMU and depth frames,
  velocity/position deltas, zero-shift variants), chained absolute
  poses (primary, zero-reset, shift-0), gravity and bias records,
  comparison numbers.
- `runs/080033_orb_poses.json`, `runs/075947_orb_poses.json` —
  adapter output from the existing ORB trajectories.
- `runs/173250_vo_poses.json` and
  `runs/173250_vo_orthomosaic.png` — the VO baseline run once for
  the 173250 consistency check.
