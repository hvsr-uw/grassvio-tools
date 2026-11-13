# IMU pre-integration — progress log

Date: 2026-11-10. Work area: `analysis/audit-imu-preint/`.
Steps 1-2 of the GrassVIO IMU pre-integration task. Bags read read-only.
Nothing uploaded to GitHub. No message sent to anyone.

## Method (one line)

Per stride-5 pair: trapezoid-integrate bias-removed gyro to a rotation
delta (inverse convention, -50 ms window shift); rotate accel into the
pair-start frame, subtract gravity estimated from the take's stillest
1-second window, trapezoid-integrate to velocity/position deltas;
propagate velocity across pairs (zero initial velocity); conjugate each
pair pose into the depth frame with the IMU-to-depth translation.

## Bag resolution (exact filename stamp, never glob order)

| take | bag file | size (bytes) |
|---|---|---|
| 20261007_080033 | sess-1861756418/browser-download-20261027T081611.443515079Z-1-20261007_080033.db3 | 680960000 |
| 20261007_075947 | sess-1861756418/browser-download-20261027T083540.215410135Z-6-20261007_075947.db3 | 363212800 |
| 20260926_155419 | sess-1861756418/browser-download-20261027T082609.603141732Z-3-20260926_155419.db3 | 84410368 |
| 20260926_102631c | sess-1861756418/browser-download-20261027T084630.779088503Z-13-20260926_102631c.db3 | 191655936 |
| 20261030_174745 | sess-rotation-nov01/box-20261030_174745.db3 | 309731328 |
| 20261026_173250 | sess-1191256009/browser-download-20261027T073314.009820058Z-5-20261026_173250.db3 | 443572224 |

Note: the 102631c stamp is 16 characters (`20260926_102631c`), so the
15-character stamp rule in the task brief does not match it; it was
resolved by full filename instead. The 080033 bag md5 was checked
against the gyro-prior run (see the 080033 entry below). Bag md5s
verified: 080033 6110426113a8ec93fa9d676397dbc104, 102631c
a2ad6e6ff469ea10c5575bc80cfc5dd5.

## Reference availability (checked before running)

- ORB-SLAM3 RGB-D-only trajectories exist for 080033 and 075947
  (leftover-three runs) and 155419 (orb-maps runs). The 080033/075947
  trajectories were converted to per-frame absolute poses with the
  existing adapter script (no new ORB-SLAM3 run).
- No ORB-SLAM3 trajectory exists for 102631c, 174745, or 173250.
  For the two still takes the reference is the truth that the camera
  did not move (end pose about zero). For 173250 there is no absolute
  reference; it is reported against the gyro total and the VO chain
  as a consistency check, and this is stated in the verdict.

## Calibration probes (measured before the integration runs)

- Gyro about 200 Hz, accel about 100 Hz, separate timestamps (nearest
  accel sample up to about 8 ms from a gyro sample). Accel is linearly
  interpolated onto the gyro sample grid inside each pair window.
- First-second median bias-removed gyro magnitude: 080033 0.072,
  075947 0.092, 155419 0.093, 102631c 0.055, 174745 0.075,
  173250 0.076 rad/s. No take has a still first second at the 0.05
  rad/s threshold, so gravity comes from the stillest 1-second window
  in each take (see the per-take entries for the window used).
- Stillest 1-second window median gyro magnitude: 080033 0.072 (at the
  start), 075947 0.051, 155419 0.076, 102631c 0.037, 174745 0.036,
  173250 0.049 rad/s. 080033 and 155419 have no window below 0.05.
- Per-take median gyro vector (the bias estimate): 080033 0.0169,
  075947 0.0163, 155419 0.1911, 102631c 0.0090, 174745 0.0045,
  173250 0.0420 rad/s. On 155419 the median is dominated by the real
  rotation of the 1.8 s rotation-in-place take, not by sensor bias;
  the specified per-take-median convention is still used as the
  primary, and this is flagged in the verdict.

## Takes done

### 20261030_174745 (still take, truth end pose ~0) — DONE

- Bag: 200 paired frames, 40 chosen at stride 5, 39 pairs, all
  integrated. Gyro bias 0.0045 rad/s. Gravity |g| 9.772 m/s^2 from
  the stillest window 2.75-3.75 s (median gyro 0.0356 rad/s, passes
  the 0.05 threshold). Rotation cross-check Exp(-rv) vs incremental:
  median 0.0003 deg.
- Primary chain (velocity propagated): end rotation 1.58 deg, end
  translation 1.308 m (vector -0.693, -0.436, 1.020 m). Path rotation
  16.90 deg, path translation 1.396 m.
- Zero-velocity-reset chain: end rotation 1.58 deg, end translation
  0.053 m, path translation 0.103 m.
- Shift-0 sensitivity: end rotation 1.66 deg (vs 1.58), end
  translation 1.329 m (vs 1.308). The -50 ms shift changes the end
  rotation by 0.08 deg and the end translation by 0.02 m.
- Per-pair noise floor on this still take: median |dpos| 2.5 mm,
  median |dvel| 0.029 m/s (truth 0 for both). Per-step rotation
  magnitude median 0.40 deg (the camera drifts; matches the visual
  0.37 deg/pair from the earlier IMU work). The propagated velocity
  turns the 3 cm/s per-pair velocity noise into 1.31 m of end drift.
- File: `runs/20261030_174745_preint.json`.

### 20261007_080033 (loop take, ORB reference) — DONE

- Bag md5 6110426113a8ec93fa9d676397dbc104, identical to the
  gyro-prior run. 441 paired frames, 89 chosen at stride 5, 88 pairs,
  all integrated. Gyro bias 0.0169 rad/s. Gravity |g| 9.745 m/s^2
  from the stillest window 0.00-1.00 s (median gyro 0.0722 rad/s,
  does NOT pass the 0.05 still threshold — no still stretch exists
  on this take). Rotation cross-check Exp(-rv) vs incremental:
  median 0.0019 deg, max 0.141 deg.
- Rotation: primary end rotation 13.82 deg (truth about 0). ORB end
  rotation 13.00 deg. End rotation error vs ORB 13.00 deg (similar
  size, different axis). Per-step rotation error vs ORB median
  0.185 deg (mean 0.222, p90 0.282, max 1.000). The per-pair
  rotations are the same gyro integration as the gyro-prior work:
  path rotation 159.36 deg against its 159.35 deg, median pair
  rotation 1.174 deg both. The end-rotation gap to the gyro-only
  replace mode (18.92 deg) is the two PnP-held pairs in that chain
  (pairs 71 and 75, 1.02 and 7.87 deg of gyro rotation), not the
  integration — rotation ties gyro-only.
- Translation: primary end translation 80.42 m (truth about 0, ORB
  0.061 m). Velocity grows to 15.69 m/s along the chain. Per-pair
  median |dvel| 0.172 m/s and median |dpos| 14.3 mm, both systematic:
  about 1.0 m/s^2 of residual acceleration, i.e. the gravity
  direction is off by about 6 deg (the gravity window rotates
  during its own second). Per-step translation error vs ORB median
  0.73 m (dominated by the diverged velocity).
- Zero-velocity-reset chain: end rotation 13.82 deg, end
  translation 1.27 m (still 20x the ORB 0.061 m).
- Shift-0 sensitivity: end rotation 14.01 deg (vs 13.82), end
  translation 80.87 m (vs 80.42). The shift is irrelevant next to
  the gravity error.
- Files: `runs/20261007_080033_preint.json`,
  ORB poses `runs/080033_orb_poses.json` (adapter output, 441 poses,
  end-to-start 0.061 m, matches the known value).

### 20261007_075947 (loop take, ORB reference) — DONE

- 235 paired frames, 47 chosen at stride 5, 46 pairs, all
  integrated. Gyro bias 0.0163 rad/s. Gravity |g| 9.730 m/s^2 from
  the stillest window 2.75-3.75 s (median gyro 0.0507 rad/s, just
  above the 0.05 still threshold — no still stretch exists on this
  take either). Rotation cross-check median 0.0008 deg.
- Rotation: primary end rotation 10.88 deg (truth about 0). ORB end
  rotation 8.84 deg. End rotation error vs ORB 6.90 deg. Per-step
  rotation error vs ORB median 0.182 deg (mean 0.196, p90 0.291,
  max 0.378). Path rotation 39.82 deg. No gyro-only number exists
  for this take (the gyro-prior work ran only 085611 and 080033).
- Translation: primary end translation 3.475 m (truth about 0, ORB
  0.188 m); velocity grows to 0.69 m/s. Per-pair median |dvel|
  0.034 m/s, median |dpos| 3.1 mm. Per-step translation error vs
  ORB median 0.088 m (primary, dominated by the diverged velocity).
- Zero-velocity-reset chain: end translation 0.022 m — better than
  the ORB 0.188 m end translation — but per-step translation error
  vs ORB median 0.0076 m against an ORB per-pair step of median
  0.0049 m, so the per-step signal is weak and the good end number
  is partly cancellation on the loop. Path translation 0.157 m
  against an ORB path of 0.327 m.
- Shift-0 sensitivity: end rotation 10.13 deg (vs 10.88), end
  translation 3.390 m (vs 3.475). The shift moves the end rotation
  by 0.75 deg.
- Files: `runs/20261007_075947_preint.json`,
  ORB poses `runs/075947_orb_poses.json` (adapter output, 235 poses,
  end-to-start 0.188 m, matches the known value).

### 20260926_155419 (loop take, rotation in place, ORB reference) — DONE

- 55 paired frames, 11 chosen at stride 5, 10 pairs, all
  integrated. Gyro bias 0.1911 rad/s — the per-take median is
  dominated by the real rotation of this 1.8 s rotation-in-place
  take, as flagged before the run. Gravity |g| 9.711 m/s^2 from
  the stillest window 0.75-1.75 s (median gyro 0.0760 rad/s, does
  NOT pass the still threshold).
- Rotation: primary end rotation 1.42 deg, path rotation only
  7.05 deg — against a gyro total of 23.0 deg and an ORB total
  rotation of 23.2 deg. The median-bias subtraction removed most
  of the real rotation. Per-step rotation error vs ORB median
  1.861 deg (mean 1.820, p90 1.964, max 1.968). End rotation error
  vs ORB 19.31 deg. This take does not test pre-integration; it
  tests the bias convention, and the convention fails here. No
  re-run with a different bias was attempted (stop rule).
- Translation: primary end translation 1.097 m (ORB end 0.135 m,
  ORB path 0.23 m). Per-pair median |dvel| 0.119 m/s, median
  |dpos| 9.7 mm. Per-step translation error vs ORB median 0.118 m.
- Zero-velocity-reset chain: end translation 0.078 m, per-step
  translation error vs ORB median 0.0213 m against an ORB per-pair
  step of median 0.0167 m.
- Shift-0 sensitivity: end rotation 1.14 deg (vs 1.42), end
  translation 1.131 m (vs 1.097).
- Files: `runs/20260926_155419_preint.json`.
  ORB poses are the existing `../orb-maps/runs/155419_orb_poses.json`.

### 20260926_102631c (still control take, truth end pose ~0) — DONE

- 124 paired frames, 25 chosen at stride 5, 24 pairs, all
  integrated. Gyro bias 0.0090 rad/s. Gravity |g| 9.725 m/s^2 from
  the stillest window 2.25-3.25 s (median gyro 0.0366 rad/s, passes
  the 0.05 threshold). Rotation cross-check median 0.0002 deg.
- Primary chain: end rotation 0.65 deg, end translation 0.299 m
  (truth 0 for both). Path rotation 5.62 deg, path translation
  0.305 m. Per-pair median |dpos| 1.5 mm, median |dvel| 0.0156 m/s
  — the cleanest per-pair numbers of the six takes, as expected
  for the control — and still enough to grow 0.30 m of end drift
  once velocity is propagated.
- Zero-velocity-reset chain: end translation 0.016 m, path
  translation 0.036 m.
- Shift-0 sensitivity: end rotation 0.70 deg (vs 0.65), end
  translation 0.272 m (vs 0.299).
- No ORB-SLAM3 trajectory exists for this take; the reference is
  the truth that the camera did not move.
- Files: `runs/20260926_102631c_preint.json`.

### 20261026_173250 (ordinary Oct-26 take, no loop, no ORB) — DONE

- Picked because it is the noisiest take in the inlier-rate table
  (non-rigid signal, pooled median residual 0.412 px against a
  0.185 px control). 287 paired frames, 58 chosen at stride 5,
  57 pairs, all integrated. Gyro bias 0.0420 rad/s. Gravity
  |g| 9.726 m/s^2 from the stillest window 4.75-5.75 s (median
  gyro 0.0494 rad/s, passes the 0.05 threshold only just).
  Rotation cross-check median 0.0008 deg.
- Rotation: primary end rotation 12.61 deg, path rotation
  52.77 deg, gyro net rotation 12.78 deg (the chain end matches
  the gyro net, as it should). The VO chain ends at 63.47 deg /
  1.611 m on this take — the VO drift, not a reference — and the
  per-step rotation disagreement with VO is median 1.054 deg,
  which is the known per-pair PnP bias size, not a pre-integration
  error measurement.
- Translation: primary end translation 14.643 m (diverged;
  velocity grows from a per-pair median |dvel| of 0.061 m/s).
  Per-pair median |dpos| 5.4 mm. VO per-pair step median 0.0297 m.
- Zero-velocity-reset chain: end translation 0.121 m, path
  translation 0.309 m; per-step translation disagreement with VO
  median 0.0281 m, about the size of the VO step itself.
- Shift-0 sensitivity: end rotation 12.46 deg (vs 12.61), end
  translation 14.471 m (vs 14.643).
- No ORB-SLAM3 trajectory exists for this take and it is not a
  loop, so there is no absolute reference; the VO chain is used
  only as a consistency check. The VO baseline was run once with
  the shared script for this check.
- Files: `runs/20261026_173250_preint.json`,
  VO poses `runs/173250_vo_poses.json`.

## All six takes done

| take | end rot, primary (deg) | end trans, primary (m) | end trans, zero-reset (m) | reference end rot / trans |
|---|---|---|---|---|
| 174745 (still) | 1.58 | 1.308 | 0.053 | truth 0 / 0 |
| 102631c (still) | 0.65 | 0.299 | 0.016 | truth 0 / 0 |
| 080033 (loop) | 13.82 | 80.425 | 1.270 | ORB 13.00 deg / 0.061 m |
| 075947 (loop) | 10.88 | 3.475 | 0.022 | ORB 8.84 deg / 0.188 m |
| 155419 (loop) | 1.42 | 1.097 | 0.078 | ORB 20.26 deg / 0.135 m |
| 173250 (ordinary) | 12.61 | 14.643 | 0.121 | none (VO 63.47 deg / 1.611 m) |
