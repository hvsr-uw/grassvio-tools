# Gyro-as-rotation-prior for the RGB-D chain: quick test

Date: 2026-11-09. Work area: `analysis-drafts/full-audit-20261109/gyro-prior/`
(CS 566 GrassVIO project). Nothing in this task was uploaded to GitHub.
No message was sent to anyone. Bags were read read-only.

## Short version

The gyro prior fixes the rotation drift but does not fix the chain.
Seeding PnP with the gyro rotation does nothing at all (bit-identical
results on both takes). Replacing the PnP rotation with the gyro rotation
cuts the chained end rotation from 86.3 deg to 0.21 deg on the still take
and from 67.6 deg to 18.9 deg on the loop take, but the chained
translation drift is untouched (0.26 m on the still take, 0.88 m on the
loop take), so the maps stay smeared and misaligned. The rotation result
is bounded by the gyro itself: on the loop take the gyro's own composed
net rotation is 21.1 deg where the truth is about 0. The prior is a
real, partial help for rotation only. It does not beat ORB-SLAM3 RGB-D
on either take, and it does not produce a clean map on its own. Two
takes were run, as allowed after the first test clearly helped rotation.
No further tuning was done.

## Method

Staged variant `gyro_prior_test.py` (a copy of the relevant part of
`dataset-tools/stitch-map/rgbd_odometry.py`; the shared script is not
modified). One change from the shared script: the per-pair SIFT
correspondences are built once and shared across all modes, so every
mode sees identical matches and the comparison is exact. All other
settings match the shared script: SIFT with 10000 features, Lowe 0.75,
stride 5, back-projection with depth plus the depth intrinsics,
`cv2.solvePnPRansac` at 3.0 px, confidence 0.99, 200 iterations, chain
`A[i+1] = A[i] @ inv(T)`, orthomosaic splat at 0.01 m cells, pixel step
2, depth capped at 3.0 m, frames tilted past 35 deg skipped from the
splat (but not from the poses). RANSAC was seeded for reproducibility.

Gyro prior, per stride-5 pair over the colour-frame interval
[t_i, t_j]: trapezoid-integrate the gyro samples in that window, minus
the per-take median gyro vector as a bias estimate, then take the
inverse rotation, `R_prior = Exp(-integral of omega dt)`. Points move
by the inverse gyro rotation: the gyro measures the body rotation and
points move the other way round. The integration window is shifted by
-50 ms against the colour interval (see the sign and sync check below).

Modes, all chained the same way:

- baseline: plain PnP, as in the shared script.
- seed: PnP with `useExtrinsicGuess`, seeded with the gyro prior
  rotation and zero translation.
- veto: plain PnP, but if the PnP rotation disagrees with the prior by
  more than 0.5 deg, the pair rotation is replaced by the prior (the
  PnP translation is kept).
- replace: the PnP translation is kept and the rotation is replaced by
  the prior on every pair.
- gyro-t0: rotation replaced by the prior and translation set to zero
  on every pair. On a still take this is the full still-camera prior.
  On a moving take it is wrong by construction and is reported only
  to show that.
- replace0: like replace, but with the prior integrated with no
  time-shift correction. Numbers only, no map. This is the sync
  sensitivity check.

End rotation and end translation are the last absolute pose against
the first (identity). Path rotation is the sum of per-step rotation
magnitudes along the chain. Per-pair rotation error is the angle
between a mode's pair rotation and the gyro prior rotation.

## Sign and sync check (verified, not assumed)

The sign and the -50 ms shift were checked against the plain PnP
rotations on both takes before trusting the prior:

- On 20261007_080033, which has real rotation, the inverse sign is
  clearly right: median PnP-vs-gyro angle 0.62 deg with the inverse
  sign at -50 ms, against 2.44 deg with the raw sign. The -50 ms
  shift also beats zero shift (0.62 deg against 0.75 deg median).
- On 20261106_085611 the gyro signal is near zero, so this take
  cannot establish the sign or the shift on its own: inverse 1.120
  deg against raw 1.125 deg median, and 0 ms against -50 ms differ
  by 0.002 deg. The 1.12 deg median on this take is the PnP bias
  itself, not a sync effect. The calibration used here is the one
  measured on 080033, matching the earlier IMU-HMM calibration.

## Take 1: 20261106_085611 (still camera, 386 frames)

Bag md5 `0b7de10d7030202ada40f076ea4eac26` (two downloads of this take
are byte-identical). 78 frames chosen at stride 5, 77 of 77 pairs
solved by PnP. The gyro over the take totals 0.58 deg of per-pair
rotation summed, 0.21 deg net, median 0.007 deg per pair. The
per-take median gyro bias magnitude is 0.00467 rad/s.

What the baseline does wrong, measured: the per-pair PnP rotation has
median magnitude 1.123 deg (mean 1.125, p90 1.174, max 1.248), and the
median error against the gyro prior is 1.120 deg. The bias is
systematic, not random: the per-pair rotation axes line up with the
mean axis at median cosine 0.9981 (minimum 0.9774), and the magnitudes
vary by only 0.041 deg standard deviation. That is a near-constant
1.12 deg rotation about a fixed axis on every pair, compounding to
86.6 deg of path rotation. The translation has its own systematic
error: median per-pair translation magnitude 3.62 mm, mean step
3.09 mm in one direction, cosine to the mean direction 0.899 median.

Results:

| mode | end rot (deg) | end trans (m) | path rot (deg) | path trans (m) | map extent (m) | frames splatted |
| --- | --- | --- | --- | --- | --- | --- |
| baseline | 86.33 | 0.238 | 86.60 | 0.278 | 3.95 x 2.65 | 32 / 78 |
| seed | 86.33 | 0.238 | 86.60 | 0.278 | 3.95 x 2.65 | 32 / 78 |
| veto (0.5 deg) | 0.21 | 0.263 | 0.58 | 0.278 | 3.48 x 2.30 | 78 / 78 |
| replace | 0.21 | 0.263 | 0.58 | 0.278 | 3.48 x 2.30 | 78 / 78 |
| gyro-t0 | 0.21 | 0.000 | 0.58 | 0.000 | 3.48 x 2.12 | 78 / 78 |
| replace0 (no shift) | 0.21 | 0.263 | 0.55 | 0.278 | — | — |

Notes on the table:

- Seed is identical to baseline to the last digit: same end rotation,
  same end translation, same median inlier count (2385), and the
  orthomosaic files are byte-identical. Seeding the RANSAC PnP with
  the gyro rotation changes nothing. The solver re-samples and lands
  on the same minimum.
- Veto fired on all 77 pairs, so veto and replace are the same run on
  this take. The rotation drift is gone: end rotation 0.21 deg is the
  gyro's own net rotation over the take, confirmed independently by
  composing the gyro prior over all stride-5 pairs (0.21 deg).
- The translation drift is untouched by the rotation prior: end
  translation 0.263 m with the prior against 0.238 m without it, and
  the path translation is identical (0.278 m) because the PnP
  translations are kept. The rotation fix does not fix the map by
  itself.
- Map extent barely moves (3.95 x 2.65 m to 3.48 x 2.12 m) because
  extent is dominated by one frame's footprint, not by drift: a
  single frame splatted alone covers 3.42 x 2.11 m at the 3.0 m depth
  cap. Extent is not a useful drift metric here. The telling map
  number is frames splatted: the baseline drops 46 of 78 frames
  because the drifted poses tilt past the 35 deg splat gate, while
  the prior modes splat all 78.
- The sync correction is irrelevant on this take, as expected:
  replace0 (no shift) also gives 0.21 deg.

Map judgement by eye (all three orthomosaics opened and compared):

- Baseline: a sparse smeared wedge with large black gaps where
  frames were dropped. Bad, as expected from the earlier VO result.
- Veto / replace: denser and complete, but the grass texture is
  visibly streaked vertically. That is the 0.26 m translation drift
  smearing the map. Better than baseline, not clean.
- Gyro-t0: a coherent rectangular grass patch, dense coverage,
  recognizable grass texture, one black hole where depth is missing.
  This is the clean map for a still camera, but it assumes the
  camera did not move at all, which is only true on this take.

## Take 2: 20261007_080033 (moving loop take, 441 frames)

Bag md5 `6110426113a8ec93fa9d676397dbc104`. 89 frames chosen at stride
5, 86 of 88 pairs solved by PnP (pairs 71 and 75 held). The gyro over
the take totals 159.4 deg of per-pair rotation summed, median 1.174
deg per pair. The take is a loop that returns to its start, so the
true end rotation and end translation are both about zero. The
reference values are the earlier VO (67.6 deg / 0.836 m, reproduced
exactly below) and ORB-SLAM3 RGB-D-only (13.0 deg / 0.061 m).

Results:

| mode | end rot (deg) | end trans (m) | path rot (deg) | path trans (m) | map extent (m) | frames splatted |
| --- | --- | --- | --- | --- | --- | --- |
| baseline | 67.62 | 0.836 | 165.89 | 0.988 | 6.73 x 3.24 | 61 / 89 |
| seed | 67.62 | 0.836 | 165.89 | 0.988 | 6.73 x 3.24 | 61 / 89 |
| veto (0.5 deg) | 30.66 | 0.882 | 151.70 | 0.988 | 6.48 x 4.10 | 89 / 89 |
| replace | 18.92 | 0.882 | 150.46 | 0.988 | 6.24 x 4.12 | 83 / 89 |
| gyro-t0 | 18.92 | 0.000 | 150.46 | 0.000 | 6.21 x 3.95 | 83 / 89 |
| replace0 (no shift) | 19.50 | 0.883 | 149.56 | 0.988 | — | — |

Notes on the table:

- Baseline reproduces the earlier VO numbers exactly (67.62 deg,
  0.836 m, 165.89 deg path). Per-pair PnP error against the gyro
  prior is median 0.615 deg (mean 0.627, p90 0.930, max 1.496),
  smaller per pair than on the still take, but the end error is
  still 67.6 deg: the per-pair errors are mostly systematic in
  direction.
- Seed is again bit-identical to baseline (orthomosaic files
  byte-identical, median inliers 2022.5 in both).
- Veto fired on 51 of 86 pairs and gives 30.66 deg, worse than full
  replace at 18.92 deg: the 35 pairs that pass the 0.5 deg veto still
  carry their own errors, which accumulate. A threshold veto is
  strictly worse than replacing the rotation outright on both takes.
- Replace at 18.92 deg is the gyro's own result, not better than the
  gyro: composing the gyro prior over all stride-5 pairs gives
  21.05 deg of net rotation, and the two held pairs account for the
  gap (pair 71 carries 1.02 deg of gyro rotation, pair 75 carries
  7.87 deg; the chain holds the last pose on those pairs). The gyro
  itself drifts about 21 deg net over this 14.7 s take, so no
  gyro-rotation prior can close this loop to zero. ORB-SLAM3 RGB-D
  at 13.0 deg is still better.
- Translation is again untouched: 0.882 m with the prior against
  0.836 m without it, against 0.061 m for ORB-SLAM3. The end
  translation is dominated by vertical drift: the end translation
  vector is (0.103, 0.850, 0.212) m, so most of the 0.88 m is the
  chained poses thinking the camera rose 0.85 m.
- The sync correction is worth about 0.6 deg here: replace0 (no
  shift) gives 19.50 deg against 18.92 deg with the -50 ms shift.

Map judgement by eye (baseline, replace, and gyro-t0 orthomosaics
opened and compared):

- Baseline: a smeared fan with washed-out grass texture, sparse
  edges, and a notch cut out at the bottom right. The loop does not
  close visually.
- Replace: denser and more complete, grass texture more visible, but
  still a fan shape with a misaligned block at the bottom right. The
  loop still does not close; the 0.88 m translation error shows
  directly. Better than baseline, not a clean map.
- Gyro-t0: a smeared fan, worse than replace. Zeroing the translation
  on a take where the camera really moved about 0.7 m folds the
  whole path onto one spot. This mode is only valid on a still take.

## Verdict: partial help for rotation only

- Helps, clearly and on both takes: replacing the PnP rotation with
  the gyro prior removes the rotation drift (86.3 to 0.21 deg still,
  67.6 to 18.9 deg loop). The mechanism is confirmed: the per-pair
  PnP rotation bias is near-constant in magnitude and axis on the
  still take (median 1.12 deg per pair, axis cosine 0.9981), and the
  gyro prior removes exactly that.
- Null: seeding PnP with the prior. Identical numbers and
  byte-identical maps on both takes. Do not pursue seeding.
- Not fixed: translation. The chained translation drift (0.26 m
  still, 0.88 m loop, mostly vertical) survives the rotation prior
  unchanged, and the maps stay smeared and misaligned. A rotation
  prior alone does not produce a clean map, and no gyro-prior mode
  beats ORB-SLAM3 RGB-D on either take (13.0 deg / 0.061 m on
  080033, effectively zero on 085611).
- Ceiling: the prior cannot beat the gyro. On the loop take the
  gyro's own composed net rotation is 21.1 deg on a loop that should
  close near zero, so the best a pure gyro-rotation prior can do
  there is about 19 deg end error. A veto threshold is worse than
  full replacement and adds a tuning parameter for no benefit.

## Stopping point

The first test clearly helped rotation, so the second take (080033)
was run as allowed. Stopping after two takes: the mechanism is
understood, the ceiling is the gyro itself, and the remaining failure
is translation drift, which a rotation prior cannot touch. No
threshold sweeps, no third take, no translation-prior tuning were
attempted. The natural next step, if anyone picks this up, is a
translation-side fix (the per-pair PnP translation has its own
systematic bias, about 3 mm per pair mostly in one direction on the
still take), not more rotation-prior work.

## Files

All under `analysis-drafts/full-audit-20261109/gyro-prior/`:

- `SUMMARY.md` — this file.
- `gyro_prior_test.py` — the staged variant (all six modes, shared
  correspondences, sign/sync check, per-mode maps and trajectory
  plots).
- `run_085611.log`, `run_080033.log` — full run logs.
- `runs/20261106_085611/` — `results.json` (all numbers),
  per-mode `*_poses.json`, `*_trajectory.npy`,
  `*_orthomosaic.png`, `*_trajectory.png`.
- `runs/20261007_080033/` — same set for the second take.
