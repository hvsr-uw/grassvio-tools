# Recording feature table — all 36 takes

Date: 2026-10-30. Built for Jinming's Oct 28 ask: comb through all the recordings
and tabulate what features each one has (wind, loop, translation, rotation,
translation + rotation, etc.). Loop and wind checks run on all 35 usable takes
on 2026-10-30; every cell below is measured except where marked.

Coverage: 36 .db3 takes from the three Box download sessions
(sess-197710181, sess-1191256009, sess-1861756418) — 21 new takes recorded
2026-10-26 (Jinming's loop/wind set) plus 15 older takes from 2026-09-26 and
2026-10-07. Every bag was resolved by its exact filename. All reads were
read-only; no bag was modified, moved, or renamed.

## Methods

Per-take stream stats were computed with one uniform script over all 36 bags:
color/depth message counts, color span (duration), gyro magnitude stats (mean,
median, p95, fraction of samples above 0.25 rad/s) from header timestamps, and
accel mean magnitude. The 21 new takes were independently inspected on Oct 27;
the recomputed numbers here reproduce those values.

Rotation level thresholds (gyro magnitude): still = mean below 0.10 rad/s and
under 5% of samples above 0.25 rad/s; gentle = mean 0.10-0.18 or 5-20% above
0.25; moderate = mean 0.18-0.28 or 20-50% above 0.25; high = mean above 0.28 or
over 50% above 0.25.

Translation proxy: per color-frame interval, the accelerometer demeaned by the
take mean, double-integrated from zero velocity — the same method as
dataset-tools/imu-integration-filtering/imu_fusion.py step 3. Honesty
constraints: it is a per-interval displacement proxy, not a trajectory, and
when the camera rotates the take-mean demeaning does not fully remove gravity,
so the proxy is contaminated upward on high-rotation takes. Cross-check: on
the four September takes imu_fusion.py itself supports, its per-interval noise
estimates are 0.098-0.336 mm and its displacement numbers agree with this
proxy to the same order of magnitude (both at the noise floor for still
takes). Translation is labeled "present" when the proxy mean is at or
above 0.4 mm (about 3x the still-take noise floor of ~0.1-0.15 mm) and the
signal clearly exceeds the gravity leakage plausible at that rotation level;
on high-rotation takes the leakage can be large, so those stay "uncertain"
regardless of the number. 0.2-0.4 mm is "uncertain"; below 0.2 mm is
"negligible". The 0.2 mm line is
arbitrary: 172859 (0.197 mm, negligible) and 173017 (0.200 mm, uncertain) are
effectively the same measurement on opposite sides of it.

Loop check (all 35 usable takes): SIFT-match the last 5 stride-5 chosen frames
against the first 5 (25 pairs; Lowe 0.75, RANSAC 3.0 px — the stitch script's
own settings). Verdict YES if any pair reaches 20+ inliers (the stitch
script's MIN_INLIERS), else NO. This tests end-vs-start closure, the practical
meaning of a "loop take" for loop-closure testing. On takes where the camera
never moves, the end trivially overlaps the start; those are marked trivially
closed, not genuine loops.

Wind check (all 35 usable takes): rigid-fit residual analysis per consecutive
frame pair, the same method as the Oct 27 wind test on 173701. Pooled
median/p90 residual per take, compared against the wind-free control 102631c
(recomputed here as pooled median 0.185 px / p90 0.458 px, confirming the
pipeline). Bands: "no wind signal" at control level (median <= 0.25 and
p90 <= 0.60); "non-rigid signal, not separated from parallax" when clearly
above (median > 0.30 or p90 > 0.80); "uncertain" between the bands or when too
few pairs fit. Never "wind yes" — the test cannot separate wind from parallax
over 3D grass, and no take is labeled as wind.

## Evidence rules

- Loop measured on all 35 usable takes (end-vs-start, 25 pairs each).
  Genuine loops, camera moved and end overlaps start: 080033 (all 25 pairs at
  1444-1882 inliers), 075947 (all 25 pairs at 133-195 inliers), 155419 (max
  728 inliers on a 1.8 s take, rotation in place). Weak but consistent,
  above the 20-inlier bar across the matrix: 172532 (max 45), 172828
  (max 37), 172859 (max 51). All still takes close trivially — the camera
  never left, so the end overlaps the start with hundreds to thousands of
  inliers (e.g. 102737a: 2129). 173807 is confirmed NO again (max 0, all 25
  pairs zero), on top of the earlier 4,400-pair tree evidence. Every other
  moving take: NO.
- Wind measured on all 35 usable takes against the 102631c control. No wind
  signal: 102608b, 102631c, 102706d, 102737a, 153136, 153610, 153825, 155411,
  075947, 080033, 172532, 172644, 172739, 173045, 173403. Uncertain: 155419,
  155430, 155507, 155555, 155614, 172828, 173507. Non-rigid signal, not
  separated from parallax: 172859, 172912, 172955, 173017, 173124, 173154,
  173220, 173250, 173540, 173603, 173628, 173701, 173807.
- 173701 keeps its detailed note: the pooled numbers hide a real second-half
  rise (per-pair median 0.5-0.8 px, p90 to 2.45 px vs a flat control), which
  is why it sits in the signal band rather than "no wind signal".
- 172642: failed recording (0.17 s, 6 frames) — excluded from both checks.
- The four September takes 20260926_102608b, 20260926_102631c,
  20260926_102706d, 20260926_102737a: no wind per Jinming's own Oct 9
  confirmation, now also measured at control level (pooled medians
  0.153-0.185 px).
- Jinming described the Oct 26 set as "recordings with loops and wind", but
  the files carry no per-take labels. Stillness is never treated as evidence
  of wind, and no label is inferred.

### New takes (2026-10-26, Jinming's loop/wind set)

| take | date | duration (s) | color frames | motion class | loop evidence | wind evidence | confidence (loop / wind) | notes |
|---|---|---|---|---|---|---|---|---|
| 20261026_172532 | 2026-10-26 | 35.3 | 1058 | rotation (high); translation uncertain | yes, weak (max 45) | no wind signal (measured) | measured / measured | End-vs-start weak but consistent across all 25 pairs (max 45). |
| 20261026_172642 | 2026-10-26 | 0.2 | 6 | n/a (failed recording) | n/a (failed recording) | n/a (failed recording) | n/a / n/a | Failed recording: 0.17 s, 6 color frames. Exclude from all tests. |
| 20261026_172644 | 2026-10-26 | 23.3 | 698 | rotation (high); translation uncertain | no | no wind signal (measured) | measured / measured |  |
| 20261026_172739 | 2026-10-26 | 21.6 | 649 | rotation (high); translation uncertain | no | no wind signal (measured) | measured / measured |  |
| 20261026_172828 | 2026-10-26 | 21.4 | 642 | rotation (high); translation uncertain | yes, weak (max 37) | uncertain (measured) | measured / measured | End-vs-start weak (max 37; last two frames below the bar). |
| 20261026_172859 | 2026-10-26 | 5.4 | 163 | rotation-only (gentle) | yes, weak (max 51) | non-rigid signal, not separated from parallax (measured) | measured / measured | End-vs-start weak but consistent (max 51). |
| 20261026_172912 | 2026-10-26 | 13.0 | 390 | rotation (gentle); translation uncertain | no | non-rigid signal, not separated from parallax (measured) | measured / measured |  |
| 20261026_172955 | 2026-10-26 | 11.5 | 347 | rotation (gentle); translation uncertain | no | non-rigid signal, not separated from parallax (measured) | measured / measured | 347 color vs 348 depth frames (1-frame mismatch, trivial). |
| 20261026_173017 | 2026-10-26 | 9.2 | 276 | rotation (gentle); translation uncertain | no | non-rigid signal, not separated from parallax (measured) | measured / measured |  |
| 20261026_173045 | 2026-10-26 | 26.5 | 795 | rotation (high); translation uncertain | no | no wind signal (measured) | measured / measured |  |
| 20261026_173124 | 2026-10-26 | 19.7 | 590 | translation+rotation (gentle) | no | non-rigid signal, not separated from parallax (measured) | measured / measured |  |
| 20261026_173154 | 2026-10-26 | 13.9 | 419 | translation+rotation (moderate) | no | non-rigid signal, not separated from parallax (measured) | measured / measured |  |
| 20261026_173220 | 2026-10-26 | 18.7 | 562 | translation+rotation (gentle) | no | non-rigid signal, not separated from parallax (measured) | measured / measured |  |
| 20261026_173250 | 2026-10-26 | 9.5 | 287 | rotation (gentle); translation uncertain | no | non-rigid signal, not separated from parallax (measured) | measured / measured |  |
| 20261026_173403 | 2026-10-26 | 6.4 | 194 | rotation (moderate, borderline); translation uncertain | no | no wind signal (measured) | measured / measured |  |
| 20261026_173507 | 2026-10-26 | 10.1 | 305 | rotation (gentle); translation uncertain | no | uncertain (measured) | measured / measured |  |
| 20261026_173540 | 2026-10-26 | 16.1 | 482 | translation+rotation (gentle) | no | non-rigid signal, not separated from parallax (measured) | measured / measured |  |
| 20261026_173603 | 2026-10-26 | 14.3 | 428 | translation+rotation (gentle) | no | non-rigid signal, not separated from parallax (measured) | measured / measured |  |
| 20261026_173628 | 2026-10-26 | 11.6 | 348 | translation+rotation (gentle) | no | non-rigid signal, not separated from parallax (measured) | measured / measured |  |
| 20261026_173701 | 2026-10-26 | 8.4 | 254 | still | no | inconclusive: real second-half non-rigid signal (median residual 0.5-0.8 px, p90 to 2.45 px vs flat control), not separated from parallax | measured / measured (inconclusive) | Wind test take: pooled median residual 0.348 px vs 0.185 px wind-free control; first ~30-90 pairs match control, then residuals rise and stay up. |
| 20261026_173807 | 2026-10-26 | 32.6 | 979 | translation+rotation (gentle) | no: last-5 vs first-5 frames 0 inliers in all 25 pairs; 4400-pair tree data shows 4 weak long-range edges, none with gap >= 50 | non-rigid signal, not separated from parallax (med 0.392, p90 1.079; measured) | measured / measured | Chain stitch diverged: canvas 57,504 x 56,538 px (0 skipped pairs, median 732 inliers). Tree placement unfinished at 4,400/19,110 pairs; checkpoint resumable. |

### Older takes (2026-09-26 and 2026-10-07)

| take | date | duration (s) | color frames | motion class | loop evidence | wind evidence | confidence (loop / wind) | notes |
|---|---|---|---|---|---|---|---|---|
| 20260926_102608b | 2026-09-26 | 5.7 | 173 | still | yes (trivial — camera still) | no wind (Jinming Oct 9; measured at control level) | measured / reported (Jinming Oct 9) + measured |  |
| 20260926_102631c | 2026-09-26 | 4.1 | 124 | still | yes (trivial — camera still) | no wind (Jinming Oct 9; measured at control level) | measured / reported (Jinming Oct 9) + measured |  |
| 20260926_102706d | 2026-09-26 | 4.6 | 138 | still | yes (trivial — camera still) | no wind (Jinming Oct 9; measured at control level) | measured / reported (Jinming Oct 9) + measured |  |
| 20260926_102737a | 2026-09-26 | 3.9 | 117 | still; translation uncertain | yes (trivial — camera still) | no wind (Jinming Oct 9; measured at control level) | measured / reported (Jinming Oct 9) + measured | Noisier IMU than the other Sept takes (imu_fusion.py per-interval noise 0.336 mm); proxy 0.219 mm sits in the uncertain band. |
| 20260926_153136 | 2026-09-26 | 3.2 | 96 | still | yes (trivial — camera still) | no wind signal (measured) | measured / measured |  |
| 20260926_153610 | 2026-09-26 | 4.2 | 128 | still | yes (trivial — camera still) | no wind signal (measured) | measured / measured |  |
| 20260926_153825 | 2026-09-26 | 6.5 | 196 | still | yes (trivial — camera still) | no wind signal (measured) | measured / measured |  |
| 20260926_155411 | 2026-09-26 | 5.8 | 176 | still | yes (trivial — camera still) | no wind signal (measured) | measured / measured |  |
| 20260926_155419 | 2026-09-26 | 1.8 | 55 | rotation (moderate); translation uncertain | yes (max 728; 1.8 s take) | uncertain (measured) | measured / measured | Very short take: 55 color frames, 1.8 s. End-vs-start max 728 inliers. |
| 20260926_155430 | 2026-09-26 | 5.0 | 150 | still | yes (trivial — camera still) | uncertain (measured) | measured / measured |  |
| 20260926_155507 | 2026-09-26 | 7.0 | 210 | still | yes (trivial — camera still) | uncertain (measured) | measured / measured |  |
| 20260926_155555 | 2026-09-26 | 10.1 | 305 | still | yes (trivial — camera still) | uncertain (measured) | measured / measured |  |
| 20260926_155614 | 2026-09-26 | 5.3 | 160 | still | yes (trivial — camera still) | uncertain (measured) | measured / measured |  |
| 20261007_075947 | 2026-10-07 | 7.8 | 235 | translation+rotation (gentle) | yes (max 195) | no wind signal (measured) | measured / measured | End-vs-start: all 25 pairs at 133-195 inliers — loop. |
| 20261007_080033 | 2026-10-07 | 14.7 | 441 | translation+rotation (moderate) | yes (max 1882) | no wind signal (measured) | measured / measured | Strongest translation signal of the older takes (1.08 mm/interval mean). Genuine loop: all 25 end-vs-start pairs match at 1444-1882 inliers. |

## Appendix: measured per-take IMU and stream numbers

All values measured read-only from the .db3 files with one uniform script. Gyro stats use header timestamps. Translation proxy: per color-frame interval, accelerometer demeaned by the take mean, double-integrated from zero velocity (same method as imu_fusion.py step 3); reported in mm per interval.

| take | color n | depth n | gyro n | gyro mean (rad/s) | gyro median (rad/s) | gyro p95 (rad/s) | frac > 0.25 rad/s | accel mean (m/s^2) | transl. proxy mean (mm) | transl. proxy median (mm) | transl. proxy max (mm) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 20261026_172532 | 1058 | 1058 | 7020 | 0.24719 | 0.2557 | 0.48831 | 0.512 | 9.7614 | 0.461 | 0.392 | 2.063 |
| 20261026_172642 | 6 | 6 | 39 | 0.07212 | 0.06209 | 0.12516 | 0.0 | 9.8908 | 0.134 | 0.129 | 0.228 |
| 20261026_172644 | 698 | 698 | 4670 | 0.33976 | 0.3375 | 0.55399 | 0.7647 | 9.7747 | 0.329 | 0.27 | 1.535 |
| 20261026_172739 | 649 | 649 | 4336 | 0.31723 | 0.30986 | 0.51304 | 0.6797 | 9.7813 | 0.339 | 0.281 | 1.379 |
| 20261026_172828 | 642 | 642 | 4291 | 0.38252 | 0.37489 | 0.6547 | 0.7833 | 9.8011 | 0.446 | 0.373 | 1.666 |
| 20261026_172859 | 163 | 163 | 1094 | 0.11987 | 0.10846 | 0.24608 | 0.043 | 9.7327 | 0.197 | 0.17 | 0.662 |
| 20261026_172912 | 390 | 390 | 2611 | 0.15684 | 0.14408 | 0.29983 | 0.1306 | 9.7591 | 0.232 | 0.196 | 0.732 |
| 20261026_172955 | 347 | 348 | 2323 | 0.14602 | 0.11787 | 0.39435 | 0.1386 | 9.764 | 0.233 | 0.191 | 0.929 |
| 20261026_173017 | 276 | 276 | 1836 | 0.12963 | 0.11975 | 0.23691 | 0.0349 | 9.7703 | 0.2 | 0.169 | 0.694 |
| 20261026_173045 | 795 | 795 | 5313 | 0.3233 | 0.32204 | 0.54848 | 0.7237 | 9.7821 | 0.389 | 0.328 | 1.534 |
| 20261026_173124 | 590 | 590 | 3945 | 0.16815 | 0.14019 | 0.35793 | 0.1947 | 9.7625 | 0.545 | 0.435 | 2.045 |
| 20261026_173154 | 419 | 419 | 2802 | 0.2059 | 0.18057 | 0.45693 | 0.3055 | 9.7393 | 0.565 | 0.48 | 1.891 |
| 20261026_173220 | 562 | 562 | 3758 | 0.17651 | 0.14721 | 0.4553 | 0.1772 | 9.7465 | 0.674 | 0.559 | 2.254 |
| 20261026_173250 | 287 | 287 | 1920 | 0.11825 | 0.09676 | 0.28542 | 0.0823 | 9.7549 | 0.253 | 0.206 | 0.794 |
| 20261026_173403 | 194 | 194 | 1298 | 0.12844 | 0.08422 | 0.32473 | 0.2042 | 9.7359 | 0.206 | 0.178 | 0.707 |
| 20261026_173507 | 305 | 305 | 2038 | 0.13175 | 0.11615 | 0.27003 | 0.0903 | 9.7538 | 0.299 | 0.24 | 1.082 |
| 20261026_173540 | 482 | 482 | 3221 | 0.1299 | 0.12794 | 0.23855 | 0.0444 | 9.6986 | 0.522 | 0.421 | 1.838 |
| 20261026_173603 | 428 | 428 | 2862 | 0.11577 | 0.10273 | 0.2551 | 0.0563 | 9.6914 | 0.43 | 0.369 | 1.232 |
| 20261026_173628 | 348 | 348 | 2331 | 0.1119 | 0.10568 | 0.24164 | 0.0455 | 9.7 | 0.46 | 0.36 | 2.097 |
| 20261026_173701 | 254 | 254 | 1701 | 0.06317 | 0.05491 | 0.12515 | 0.0 | 9.7094 | 0.132 | 0.109 | 0.579 |
| 20261026_173807 | 979 | 979 | 6549 | 0.1076 | 0.09393 | 0.23594 | 0.0351 | 9.7044 | 0.57 | 0.497 | 1.739 |
| 20260926_102608b | 173 | 173 | 1157 | 0.03407 | 0.0306 | 0.06866 | 0.0 | 9.7401 | 0.02 | 0.007 | 0.284 |
| 20260926_102631c | 124 | 124 | 828 | 0.05026 | 0.04891 | 0.08934 | 0.0 | 9.7454 | 0.084 | 0.074 | 0.277 |
| 20260926_102706d | 138 | 138 | 919 | 0.07227 | 0.05409 | 0.18742 | 0.0239 | 9.7365 | 0.096 | 0.08 | 0.305 |
| 20260926_102737a | 117 | 118 | 777 | 0.0727 | 0.04787 | 0.20422 | 0.0373 | 9.7345 | 0.219 | 0.179 | 1.007 |
| 20260926_153136 | 96 | 96 | 634 | 0.03902 | 0.02996 | 0.10877 | 0.0 | 9.7472 | 0.067 | 0.051 | 0.328 |
| 20260926_153610 | 128 | 128 | 843 | 0.03308 | 0.03058 | 0.06633 | 0.0 | 9.7429 | 0.053 | 0.043 | 0.17 |
| 20260926_153825 | 196 | 196 | 1309 | 0.05826 | 0.04891 | 0.13316 | 0.0015 | 9.744 | 0.126 | 0.106 | 0.408 |
| 20260926_155411 | 176 | 177 | 1179 | 0.05096 | 0.04528 | 0.10381 | 0.0 | 9.7499 | 0.076 | 0.059 | 0.258 |
| 20260926_155419 | 55 | 54 | 363 | 0.22381 | 0.21863 | 0.32375 | 0.2066 | 9.7824 | 0.228 | 0.208 | 0.521 |
| 20260926_155430 | 150 | 150 | 1003 | 0.06125 | 0.04829 | 0.1399 | 0.009 | 9.7421 | 0.068 | 0.05 | 0.387 |
| 20260926_155507 | 210 | 210 | 1403 | 0.03592 | 0.03337 | 0.06778 | 0.0 | 9.7546 | 0.069 | 0.057 | 0.226 |
| 20260926_155555 | 305 | 305 | 2036 | 0.03971 | 0.03535 | 0.08199 | 0.0 | 9.7422 | 0.068 | 0.057 | 0.339 |
| 20260926_155614 | 160 | 160 | 1069 | 0.03851 | 0.03624 | 0.0724 | 0.0 | 9.7411 | 0.076 | 0.067 | 0.282 |
| 20261007_075947 | 235 | 235 | 1562 | 0.13056 | 0.10742 | 0.30572 | 0.0832 | 9.7327 | 0.418 | 0.359 | 1.406 |
| 20261007_080033 | 441 | 440 | 2945 | 0.22608 | 0.15489 | 0.64511 | 0.2706 | 9.7507 | 1.079 | 0.89 | 3.05 |

## Appendix: per-take wind residuals (rigid-fit, consecutive pairs)

Pooled over all good matches. Control 102631c recomputed here as median 0.185 px / p90 0.458 px, confirming the pipeline.

| take | pooled median (px) | pooled p90 (px) | verdict |
|---|---|---|---|
| 20260926_102608b | 0.153 | 0.398 | no wind signal |
| 20260926_102631c | 0.185 | 0.458 | no wind signal |
| 20260926_102706d | 0.175 | 0.446 | no wind signal |
| 20260926_102737a | 0.165 | 0.419 | no wind signal |
| 20260926_153136 | 0.188 | 0.514 | no wind signal |
| 20260926_153610 | 0.177 | 0.473 | no wind signal |
| 20260926_153825 | 0.178 | 0.479 | no wind signal |
| 20260926_155411 | 0.194 | 0.52 | no wind signal |
| 20260926_155419 | 0.267 | 0.67 | uncertain |
| 20260926_155430 | 0.231 | 0.697 | uncertain |
| 20260926_155507 | 0.258 | 0.733 | uncertain |
| 20260926_155555 | 0.223 | 0.636 | uncertain |
| 20260926_155614 | 0.213 | 0.608 | uncertain |
| 20261007_075947 | 0.167 | 0.422 | no wind signal |
| 20261007_080033 | 0.182 | 0.447 | no wind signal |
| 20261026_172532 | 0.206 | 0.478 | no wind signal |
| 20261026_172644 | 0.215 | 0.491 | no wind signal |
| 20261026_172739 | 0.249 | 0.584 | no wind signal |
| 20261026_172828 | 0.269 | 0.627 | uncertain |
| 20261026_172859 | 0.277 | 0.897 | non-rigid signal, not separated from parallax |
| 20261026_172912 | 0.415 | 1.105 | non-rigid signal, not separated from parallax |
| 20261026_172955 | 0.344 | 0.983 | non-rigid signal, not separated from parallax |
| 20261026_173017 | 0.47 | 1.173 | non-rigid signal, not separated from parallax |
| 20261026_173045 | 0.233 | 0.528 | no wind signal |
| 20261026_173124 | 0.321 | 0.977 | non-rigid signal, not separated from parallax |
| 20261026_173154 | 0.376 | 1.187 | non-rigid signal, not separated from parallax |
| 20261026_173220 | 0.407 | 1.254 | non-rigid signal, not separated from parallax |
| 20261026_173250 | 0.412 | 1.076 | non-rigid signal, not separated from parallax |
| 20261026_173403 | 0.189 | 0.497 | no wind signal |
| 20261026_173507 | 0.261 | 0.742 | uncertain |
| 20261026_173540 | 0.467 | 1.327 | non-rigid signal, not separated from parallax |
| 20261026_173603 | 0.524 | 1.603 | non-rigid signal, not separated from parallax |
| 20261026_173628 | 0.386 | 1.188 | non-rigid signal, not separated from parallax |
| 20261026_173701 | 0.348 | 0.97 | non-rigid signal, not separated from parallax |
| 20261026_173807 | 0.392 | 1.079 | non-rigid signal, not separated from parallax |

## What this table does not claim

- Loop labels come from the end-vs-start check only: whether the last frames
  still see the first frames' view. A "yes" on a still take is trivial, and a
  "yes, weak" (max 37-51 inliers) is a small overlap, not a confident closure.
  The full stitch remains the stronger loop test.
- Wind labels are signal-vs-control comparisons, never wind proof. A
  "non-rigid signal" on a moving take can be parallax over 3D grass; only on
  a still take (173701) does camera motion drop out as an explanation, and
  even there parallax from the earlier part of the take cannot be ruled out.
- Motion classes come from the IMU alone. "Uncertain" means the measurement
  cannot separate the cases, not that the feature is absent.
