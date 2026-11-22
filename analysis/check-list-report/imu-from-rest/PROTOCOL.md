# Item 5 protocol: IMU vs RGB-D translation from a verified rest start

Written on 2026-11-21 before any extracted IMU or dense RGB-D data was read.
Thresholds below are fixed and are not changed after seeing results.

## Data
Six November 19 recordings (073137, 073608, 073711, 074041, 074125, 074200).
Per recording: raw gyro (~200 Hz) and accelerometer (~100 Hz) with header
timestamps, and dense RGB-D odometry between colour frames i and i+3 (0.1 s):
SIFT matches, 3D points from registered depth of frame i, PnP pose of i+3.

IMU-to-camera time shift: the per-recording `shift_ms` from the existing
`gyro_timing.json` (fitted on the first 20% of each video against visual
rotation only; it never saw translation). IMU sample time t_imu maps to
camera time t_imu - shift.

## Step 1. Rest verification (all three must hold)
The rest window starts at the first colour frame and extends over consecutive
0.1 s visual pairs while all of these hold:
- R1 visual: every pair has median image motion < 0.3 px, and the chained
  RGB-D pose stays within 3 mm and 0.2 deg of the first frame.
- R2 gyro: 95th percentile of the gyro norm inside the window < 0.02 rad/s.
- R3 accelerometer: standard deviation of the accelerometer norm inside the
  window < 0.05 m/s^2.
A recording counts as starting at rest only if this window lasts >= 0.5 s.
A quiet IMU alone never counts as rest.

## Step 2. Initialisation from the verified window only
- Gyro bias: mean gyro over the window.
- Gravity plus accelerometer bias: mean specific force f0 over the window,
  expressed in the camera frame at the end of the window. These two cannot be
  separated from one still pose; the combined vector is subtracted.
- Initial velocity: exactly zero, because the rest is verified.
- Integration origin: the last frame of the rest window. Trapezoidal gyro and
  accelerometer integration in the colour-camera frame, IMU lever arm included.
  No visual information is used after the origin.

## Step 3. Scoring against RGB-D
The reference is the chained dense RGB-D odometry from the same origin.
Horizons: 0.1, 0.3, 1.0 and 3.0 s after the origin (3.0 s is descriptive).
A horizon is unscored if the visual chain has a failed pair before it, or if
the camera moved < 1 cm by then (too little motion to judge translation).
The comparator is the constant-velocity prediction from a verified rest,
which is "no motion", so its error equals the visual displacement.

Gates (each must hold on every rest-verified recording with a scored horizon):
- G1: IMU position error < visual displacement at 0.3 s and 1.0 s
  (beats the constant-velocity prediction).
- G2: IMU position error <= 25% of visual displacement at 1.0 s.
- G3: IMU rotation differs from RGB-D rotation by <= 1.0 deg at 1.0 s.
Item 5 passes only if at least 3 recordings are rest-verified and G1-G3 hold
on all of them.

## Controls and descriptive checks (no gates)
- The same integration started from the first 0.5 s on recordings that fail
  rest verification, to show what an unverified zero-velocity start does.
- |f0| against local gravity (Madison, about 9.803 m/s^2): a direct reading of
  accelerometer scale or bias magnitude.
- If a recording also ends still (same R1-R3 rules on the final frames), the
  velocity integrated over the whole video should return to zero; the
  residual is reported, not gated.

RGB-D odometry is the reference, not ground truth.
