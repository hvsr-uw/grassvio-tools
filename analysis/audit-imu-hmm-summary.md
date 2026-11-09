# IMU-agreement channel for the reliability model: implemented and tested

Date: 2026-11-09. This implements the IMU channel that the reliability-models
summary designed but did not run, and tests it on 11 takes against the
residual-only version on the same matches. Short verdict first: the IMU
version does NOT separate reliable from unreliable takes better than the
residual-only version. It makes the non-rigid-signal take look more
unreliable (fraction above 0.8 falls from 0.472 to 0.219), but it also
makes a clean control take look unreliable (0.978 to 0.162) and it
discounts clean rotating takes by 8 to 25 points. The wider gap between
the signal take and the clean takes is the channel failing on clean
takes, not better discrimination. The two parts of the channel fail in
different ways, and each failure is measured below.

## What was run

Staged variant `reliability_hmm_imu.py`, a copy of the prototype with the
IMU channel added. The prototype file itself is untouched. Every track is
scored twice from the same SIFT matches, once with the residual-only
filter and once with the IMU filter, so the comparison is exact: the
residual-only numbers reproduce the original three takes to three
decimals (174745: 0.978 above 0.8; 173701: 0.472; 080033: 0.975, identical
track counts). Takes: the original three (174745, 173701, 080033) plus all
eight Nov-06 takes (084542, 084741, 084852, 085224, 085301, 085422,
085453, 085611). Bags read read-only, resolved by exact filename stamp.
085611 scores reliable despite its brightness oscillation, as expected:
residual-only 0.989 above 0.8 and IMU 0.989, identical. The light change
is global and rigid, and neither version is confused by it.

## The method, exact formulas and parameters

The base model is unchanged from the reliability-models summary: hidden
state reliable/unreliable, prior P(reliable) = 0.5, transition
P(R|R) = 0.95, P(U|R) = 0.05, P(R|U) = 0.30, P(U|U) = 0.70, half-normal
likelihoods on the residual r with sigma_R = 0.5 px and sigma_U = 3.0 px,
forward filter per track. SIFT (10000) + Lowe 0.75 on consecutive
stride-5 frame pairs, homography by RANSAC at 3.0 px. Colour intrinsics
are read from each bag's own camera_info topic, not assumed.

(a) Pair-level gyro-vs-vision agreement. For each pair, the gyro is
integrated over the pair interval to a rotation vector, and points move
by the inverse rotation: R_imu = Exp(-integral of omega dt). The visual
rotation is R_vis = orthonormalized K^-1 H K from the fitted homography.
The agreement angle is a = angle(R_imu^T R_vis). The pair weight is
w = exp(-a^2 / (2 sigma_a^2)) with sigma_a = 0.03 rad, and the pair's
likelihoods are raised to the power w (likelihood tempering). w = 1
recovers the prototype filter; w = 0 makes the pair contribute nothing.

(b) Still takes only: the rotation-only predicted pixel motion. The
gyro-predicted position of a feature is K R_imu K^-1 applied to its
position in the first frame, and r_imu is the distance from that
prediction to the observed position in the second frame. On still takes
r_imu REPLACES r_vis as the single observation, with the same sigmas
(0.5 and 3.0 px), so any change in separation is the reference motion,
not doubled evidence or retuned likelihoods. The gyro bias is removed
first as the per-take median gyro vector, on still takes only. On moving
takes r_imu is recorded for diagnosis and NOT used: translation makes
the rotation-only prediction wrong by pixels (pooled median 2.8 px on
080033, 13.5 px on 085301, both measured before this run).

Still is decided from the gyro alone, with the feature table's own
thresholds: gyro mean below 0.10 rad/s and under 5% of samples above
0.25 rad/s. The take-level wind label is NOT used anywhere: it is the
held-out label, so using it would be circular.

## Three calibration findings, measured before the full run

1. Points move by the inverse gyro rotation. On 080033 the visual
rotation axis dots -0.986 with the raw gyro axis and +0.986 with the
negated axis. The gyro measures the body rotation; points move the
other way round. Using the raw sign gives a median rotation-only
residual of 16.6 px on 080033; the inverse gives 2.8 px.
2. The gyro timestamps lag the colour timestamps by about 50 ms. Scanning
the gyro window shift from -80 to 0 ms on four takes, the median
gyro-vision relative angle has a sharp minimum at -50 ms on 080033
(0.0015 rad against 0.0069 at zero shift), a shallow minimum at -50 ms
on 174745 (0.0026 against 0.0044), and is nearly flat on 173701 (0.0070
against 0.0081) and 085301 (0.0245 against 0.0248). The channel is
therefore run twice: uncorrected (zero shift, kept in shift0/) and
corrected (-50 ms, the primary result below). The offset is a real
pipeline sync error on some takes, and it is NOT the explanation for
the disagreement on 173701 or 085301.
3. The median-gyro bias estimate is harmless but nearly useless. On the
still takes the median gyro vector has magnitude 0.0045-0.0057 rad/s,
and removing it changes the pooled rotation-only residual almost not at
all (174745: 2.09 px raw vs 2.05 px debiased; 173701: 4.52 vs 4.50;
085611: 0.37 vs 0.12, the one take where it matters). The large
rotation-only residuals on the October still takes are NOT a bias
problem.

## Per-take comparison table (corrected IMU version, -50 ms sync fix)

Median reliability is the median over tracks of final P(reliable).
Median residual is the median over tracks of the track median residual
(r_vis for the residual-only version; r_imu for the IMU version on still
takes, r_vis on moving takes). The last column is the uncorrected IMU
fraction above 0.8, so the effect of the sync fix can be read off.

| take | motion / label | tracks | med track len | resid med rel | resid > 0.8 | resid < 0.2 | resid med resid | IMU med rel | IMU > 0.8 | IMU < 0.2 | IMU med resid | IMU > 0.8, no sync fix |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 20261030_174745 | still, no wind signal | 32,030 | 1 | 0.909 | 0.978 | 0.002 | 0.204 px | 0.035 | 0.162 | 0.613 | 1.660 px | 0.063 |
| 20261026_173701 | still, non-rigid signal (2nd half) | 35,456 | 1 | 0.759 | 0.472 | 0.322 | 0.791 px | 0.000 | 0.219 | 0.639 | 2.097 px | 0.136 |
| 20261007_080033 | rotation, no wind signal | 85,903 | 1 | 0.909 | 0.975 | 0.002 | 0.209 px | 0.908 | 0.975 | 0.002 | 0.209 px | 0.972 |
| 20261106_084542 | still, no wind signal (micro-shake) | 65,385 | 2 | 0.976 | 0.978 | 0.004 | 0.153 px | 0.924 | 0.905 | 0.024 | 0.285 px | 0.863 |
| 20261106_084741 | still, no wind signal (moving shadow) | 55,860 | 2 | 0.978 | 0.984 | 0.003 | 0.132 px | 0.977 | 0.979 | 0.003 | 0.158 px | 0.962 |
| 20261106_084852 | still, no wind signal (shadow event) | 67,127 | 2 | 0.979 | 0.986 | 0.003 | 0.115 px | 0.979 | 0.985 | 0.003 | 0.125 px | 0.985 |
| 20261106_085611 | still, no wind signal (illumination oscillation) | 68,473 | 2 | 0.979 | 0.989 | 0.002 | 0.112 px | 0.979 | 0.989 | 0.002 | 0.126 px | 0.989 |
| 20261106_085224 | rotation high, no wind signal | 105,270 | 1 | 0.904 | 0.833 | 0.035 | 0.344 px | 0.848 | 0.586 | 0.008 | 0.344 px | 0.581 |
| 20261106_085301 | rotation high, no wind signal | 102,495 | 1 | 0.904 | 0.799 | 0.056 | 0.372 px | 0.872 | 0.666 | 0.026 | 0.372 px | 0.657 |
| 20261106_085422 | rotation moderate, no wind signal | 92,116 | 1 | 0.908 | 0.944 | 0.006 | 0.251 px | 0.902 | 0.864 | 0.003 | 0.251 px | 0.854 |
| 20261106_085453 | rotation high, no wind signal | 78,808 | 1 | 0.906 | 0.910 | 0.011 | 0.292 px | 0.872 | 0.685 | 0.004 | 0.292 px | 0.672 |

Separation, measured as clean-take mean fraction above 0.8 minus the
173701 fraction: residual-only 0.465 (clean mean 0.938, 173701 0.472);
IMU corrected 0.561 (clean mean 0.780, 173701 0.219); IMU uncorrected
0.624 (clean mean 0.760, 173701 0.136). The IMU gap is wider, but the clean-take mean falls from 0.938 to 0.780 and the
clean-take minimum falls from 0.799 (085301) to 0.162 (174745). A wider
gap bought by scoring a clean control take as unreliable is not better
separation. The plain threshold rule (fraction of tracks with median
residual under 0.5 px) shows the same pattern, so this is not an HMM
artifact: on 174745 the threshold gives 0.926 residual-only against
0.063 IMU; on 173701 it gives 0.344 against 0.123.

## Geometry diagnostics per take (corrected run)

Median relative angle is the median over pairs of a, the gyro-vision
agreement angle. Median pair weight is the median of w. Median r_imu
per pair is the median over pairs of the pair median r_imu; it is the
cleanest single number for how well the gyro predicts the pixels.

| take | still | median rel angle | median pair weight | median r_imu per pair | median r_vis per pair | median theta_vis per pair |
|---|---|---|---|---|---|---|
| 20261030_174745 | yes | 0.0027 rad | 0.996 | 1.654 px | 0.188 px | 0.00643 rad |
| 20261026_173701 | yes | 0.0069 rad | 0.974 | 4.054 px | 1.105 px | 0.01002 rad |
| 20261007_080033 | no | 0.0015 rad | 0.999 | 1.302 px | 0.198 px | 0.02121 rad |
| 20261106_084542 | yes | 0.0005 rad | 1.000 | 0.293 px | 0.141 px | 0.00049 rad |
| 20261106_084741 | yes | 0.0001 rad | 1.000 | 0.131 px | 0.118 px | 0.00008 rad |
| 20261106_084852 | yes | 0.0001 rad | 1.000 | 0.110 px | 0.100 px | 0.00003 rad |
| 20261106_085611 | yes | 0.0001 rad | 1.000 | 0.121 px | 0.099 px | 0.00004 rad |
| 20261106_085224 | no | 0.0378 rad | 0.452 | 19.608 px | 0.344 px | — |
| 20261106_085301 | no | 0.0244 rad | 0.717 | 16.704 px | 0.316 px | 0.05231 rad |
| 20261106_085422 | no | 0.0154 rad | 0.877 | 9.145 px | 0.251 px | — |
| 20261106_085453 | no | 0.0279 rad | 0.648 | 16.482 px | 0.292 px | — |

## Channel (a): the pair-level agreement weight hurts rotating takes

On 080033 the sync fix makes tempering a non-event: median weight 0.999,
IMU fraction above 0.8 is 0.975, identical to residual-only, and the two
belief curves lie on top of each other. On the four Nov-06 rotating
takes the median relative angle is 0.015-0.038 rad, so the median weight
is 0.45-0.88 and the fraction above 0.8 falls: 085224 by 0.247 (0.833 to
0.586), 085301 by 0.133 (0.799 to 0.666), 085422 by 0.080 (0.944 to
0.864), 085453 by 0.225 (0.910 to 0.685). The histograms show the
mechanism: tempering smears the reliability mass down from the 0.95-1.0
pile toward 0.5-0.9 instead of moving a few bad pairs. The belief curves
keep the same shape over time, just lower. The disagreement is a property
of fast rotation, not of individual bad pairs: it does not vary with the
time shift (085301 is flat at 0.024-0.026 rad for every shift from -80 to
0 ms), and it grows with rotation size. The likely cause is the
homography rotation extraction under large rotation plus translation, not
the gyro: the fitted homography is a compromise over 3D grass, and its
rotation part is not the camera rotation. As implemented, tempering
penalizes takes for rotating fast. It would need a disagreement measure
that is relative within a take (a pair that disagrees more than its own
take usually does), not an absolute angle, to act as a veto on bad pairs
instead of a discount on fast takes.

## Channel (b): the rotation-only residual works only where the camera is really still

On the four Nov-06 still takes the gyro predicts the pixels almost as
well as the fitted homography does: median r_imu per pair 0.110-0.293 px
against median r_vis 0.099-0.141 px. The IMU version matches the
residual-only version on 085611 (0.989 vs 0.989 above 0.8), 084741
(0.979 vs 0.984) and 084852 (0.985 vs 0.986), with flat belief curves
near 0.97 on all three. On 084542 it drops to 0.905, and the drop is
real camera motion, not grass: the IMU belief curve dips sharply to 0.37
at pair 7 while the residual-only curve stays flat at 0.96, and the
pair-median r_imu spikes to 1.78 px at pair 7 and 1.03 px at pair 8
against a 0.29 px take median. Pairs 7-8 are frames 35-45, the first
camera micro-shake episode measured in the Nov-06 batch summary; the
fitted homography absorbs the shake, the gyro prediction does not. The
second documented shake episode (frames 175-215, pairs 35-43) does NOT
show in r_imu (pair medians 0.13-0.40 px), so the channel catches the
first episode and misses the second; the shake is intermittent and the
second episode is smaller in the gyro-predicted residual.

On the two October still takes the channel fails outright. 174745 has
median r_imu 1.654 px per pair against r_vis 0.188 px, so the IMU
version scores the control take at median reliability 0.035 with 61.3%
of tracks below 0.2. 173701 has median r_imu 4.054 px against r_vis
1.105 px, so it scores at median reliability 0.000 with 63.9% below 0.2.
The cause is visible in the geometry: these cameras are not really
still. Their median visual rotation per pair is 0.0064 rad (174745) and
0.0100 rad (173701), against 0.00003-0.00049 rad on the Nov-06 still
takes, and the gyro-predicted rotation disagrees with the visual
rotation by a median 0.0027 and 0.0069 rad even after the sync fix.
That disagreement alone is 1.7-4.1 px of pixel error, several times
sigma_R = 0.5 px, so every feature looks unreliable whether or not
anything is blowing. The gyro-mean "still" criterion (below 0.10 rad/s)
is too loose to gate this channel: 174745 passes it at 0.063 rad/s while
drifting. A usable gate is the visual rotation itself: median theta_vis
below about 0.001 rad per pair separates the Nov-06 still takes
(0.00003-0.00049) from the October takes (0.0064-0.0100) cleanly.

On 173701 the IMU belief curve keeps the right timing and exaggerates
the depth: it starts at 0.861 (residual-only 0.886), falls from pair 4,
reaches 0.003 at pair 29 (residual-only minimum 0.223), and ends at
0.112 (residual-only 0.503). The early pairs agree with the residual
version (r_imu pair medians 0.37-1.54 px on pairs 0-7), the late pairs
do not (3.5-10.8 px). Part of that rise is the non-rigid signal and part
is the growing gyro-vision disagreement as the take drifts; the two
cannot be separated from this run, so the deeper fall on 173701 is not
evidence the channel detects wind better.

## What the plots show (all opened and judged by eye)

- Comparison bar chart (comparison_frac_gt_0.8.png): the residual-only
bars are near 1.0 on every clean take; the IMU bars collapse on 174745
and 173701 and drop on all four Nov-06 rotating takes, and match on the
Nov-06 still takes and 080033.
- Tempering-cost scatter (tempering_cost_scatter.png): the IMU minus
residual-only fraction above 0.8 falls as the median relative angle
grows for the rotating takes, with 174745 far below the trend at -0.816
(a still take hurt by channel (b), not by tempering) and 080033,
084741, 084852, 085611 clustered at (0, 0).
- Per-take histograms (one per take, residual-only vs IMU side by
side): 085611, 084741, 084852 and 080033 are near-identical piles at
0.95-1.0; 085224, 085301, 085422, 085453 show the tempering smear from
the 0.95-1.0 pile down to 0.5-0.9; 084542 shows a small peak at 0 on the
IMU side; 174745 flips from a pile at 0.9-1.0 to a pile at 0.0-0.05;
173701 goes from bimodal (peaks at 0 and 0.95) to a dominant peak at 0.
- Per-take belief curves: 085611, 084741, 084852 flat near 0.97 on both
versions; 080033 the two lines overlap exactly; 085224, 085301, 085422,
085453 track the same shape with the IMU line below; 084542 flat on
residual-only with the sharp IMU dip to 0.37 at pair 7; 174745 flat at
0.94 on residual-only with the IMU line near 0 for pairs 0-10 then
oscillating 0.0-0.8; 173701 falling on both versions, deeper and without
recovery on the IMU version.

## Honest limits

- Only one non-rigid-signal take exists in the test set (173701), so
every separation claim rests on one take against ten clean ones.
- Median track length is 1 on the October takes and the rotating takes,
2 on the Nov-06 still takes, so the Markov temporal part still
contributes little and the separation comes almost entirely from the
single-residual likelihood, same as the prototype.
- Wind vs parallax is still not separated: the channel flags features
that do not follow the gyro-predicted rotation, which on a drifting
camera includes the gyro-vision disagreement itself.
- The -50 ms sync offset is a single global value fitted on four takes
by rotation agreement. It is sharp on 080033 and shallow or flat on the
others, so it may be per-session rather than universal; it changes the
verdict on no take except 080033 (0.972 to 0.975 above 0.8) and 174745
(0.063 to 0.162).
- The homography rotation extraction is the weakest link on moving
takes: R_vis from K^-1 H K is approximate on a non-planar scene with
translation, and the 0.015-0.038 rad disagreement on fast rotating
takes is most plausibly that extraction error, not gyro error. An
essential-matrix rotation would be the next thing to try before trusting
any agreement weight on moving takes.

## Files

All under analysis-drafts/full-audit-20261109/imu-hmm/:

- SUMMARY.md — this file.
- reliability_hmm_imu.py — the staged variant (copy of the prototype
plus the IMU channel; the prototype is untouched).
- *_tracks.json — per-take JSONs for all 11 takes (per-track reliability
under both filters, pair diagnostics, both belief curves). The
"imu" block is the corrected (-50 ms) version; the "residual_only"
block reproduces the prototype.
- shift0/*_shift0_tracks.json — the same 11 takes without the sync fix.
- *_hist.png, *_curve.png — per-take reliability histograms and belief
curves, both versions, all judged by eye.
- comparison_frac_gt_0.8.png, tempering_cost_scatter.png — summary
figures.
- explore_imu_geometry.py, *_explore.json — the geometry calibration
(sign flip, bias, r_imu distributions) on 085611, 080033, 173701, 085301.
- diagnose_174745.py, *_diag.json, diagnose_174745.log — the 174745
failure diagnosis (bias ruled out, raw vs debiased).
- test_time_offset.py, test_time_offset.log, test_offset_fine.log — the
sync-offset scan (-80 to 0 ms) on four takes.
- aggregate.py, aggregate_out.txt — the comparison table aggregation.
- make_summary_figs.py — the two summary figures.
- run_all.sh, run_all.log, run_all_shift50.sh, run_all_shift50.log —
the two full runs (uncorrected, then corrected).
