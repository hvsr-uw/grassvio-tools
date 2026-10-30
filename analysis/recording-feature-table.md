# Recording feature table — all 36 takes

Date: 2026-10-30. Built for Jinming's Oct 28 ask: comb through all the recordings
and tabulate what features each one has (wind, loop, translation, rotation,
translation + rotation, etc.).

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

## Evidence rules

Only the following are established. Everything else is marked untested.

- 173807: loop tested NO — last-5 vs first-5 frames give 0 inliers in all 25
  pairs, and 4,400 computed tree pairs show only 4 weak long-range edges, none
  with gap >= 50.
- 173701: wind tested INCONCLUSIVE — a real second-half non-rigid signal
  (per-pair median residual rises to 0.5-0.8 px, p90 to 2.45 px, against a flat
  wind-free control), not separated from parallax. Never labeled wind yes.
  Residuals figure: wind_residuals_comparison.png (same folder); the per-take
  numbers are in the 173701 row below.
- 172642: failed recording (0.17 s, 6 frames) — unusable for any test.
- The four September takes 20260926_102608b, 20260926_102631c,
  20260926_102706d, 20260926_102737a: no wind, per Jinming's own Oct 9
  confirmation.
- Jinming described the Oct 26 set as "recordings with loops and wind", but
  the files carry no per-take labels. Stillness is never treated as evidence
  of wind, and no label is inferred.

### New takes (2026-10-26, Jinming's loop/wind set)

| take | date | duration (s) | color frames | motion class | loop evidence | wind evidence | confidence (loop / wind) | notes |
|---|---|---|---|---|---|---|---|---|
| 20261026_172532 | 2026-10-26 | 35.3 | 1058 | rotation (high); translation uncertain | untested | untested | untested / untested |  |
| 20261026_172642 | 2026-10-26 | 0.2 | 6 | n/a (failed recording) | n/a (failed recording) | n/a (failed recording) | n/a / n/a | Failed recording: 0.17 s, 6 color frames. Exclude from all tests. |
| 20261026_172644 | 2026-10-26 | 23.3 | 698 | rotation (high); translation uncertain | untested | untested | untested / untested |  |
| 20261026_172739 | 2026-10-26 | 21.6 | 649 | rotation (high); translation uncertain | untested | untested | untested / untested |  |
| 20261026_172828 | 2026-10-26 | 21.4 | 642 | rotation (high); translation uncertain | untested | untested | untested / untested |  |
| 20261026_172859 | 2026-10-26 | 5.4 | 163 | rotation-only (gentle) | untested | untested | untested / untested |  |
| 20261026_172912 | 2026-10-26 | 13.0 | 390 | rotation (gentle); translation uncertain | untested | untested | untested / untested |  |
| 20261026_172955 | 2026-10-26 | 11.5 | 347 | rotation (gentle); translation uncertain | untested | untested | untested / untested | 347 color vs 348 depth frames (1-frame mismatch, trivial). |
| 20261026_173017 | 2026-10-26 | 9.2 | 276 | rotation (gentle); translation uncertain | untested | untested | untested / untested |  |
| 20261026_173045 | 2026-10-26 | 26.5 | 795 | rotation (high); translation uncertain | untested | untested | untested / untested |  |
| 20261026_173124 | 2026-10-26 | 19.7 | 590 | translation+rotation (gentle) | untested | untested | untested / untested |  |
| 20261026_173154 | 2026-10-26 | 13.9 | 419 | translation+rotation (moderate) | untested | untested | untested / untested |  |
| 20261026_173220 | 2026-10-26 | 18.7 | 562 | translation+rotation (gentle) | untested | untested | untested / untested |  |
| 20261026_173250 | 2026-10-26 | 9.5 | 287 | rotation (gentle); translation uncertain | untested | untested | untested / untested |  |
| 20261026_173403 | 2026-10-26 | 6.4 | 194 | rotation (moderate, borderline); translation uncertain | untested | untested | untested / untested |  |
| 20261026_173507 | 2026-10-26 | 10.1 | 305 | rotation (gentle); translation uncertain | untested | untested | untested / untested |  |
| 20261026_173540 | 2026-10-26 | 16.1 | 482 | translation+rotation (gentle) | untested | untested | untested / untested |  |
| 20261026_173603 | 2026-10-26 | 14.3 | 428 | translation+rotation (gentle) | untested | untested | untested / untested |  |
| 20261026_173628 | 2026-10-26 | 11.6 | 348 | translation+rotation (gentle) | untested | untested | untested / untested |  |
| 20261026_173701 | 2026-10-26 | 8.4 | 254 | still | untested | inconclusive: real second-half non-rigid signal (median residual 0.5-0.8 px, p90 to 2.45 px vs flat control), not separated from parallax | untested / measured (inconclusive) | Wind test take: pooled median residual 0.348 px vs 0.185 px wind-free control; first ~30-90 pairs match control, then residuals rise and stay up. |
| 20261026_173807 | 2026-10-26 | 32.6 | 979 | translation+rotation (gentle) | no: last-5 vs first-5 frames 0 inliers in all 25 pairs; 4400-pair tree data shows 4 weak long-range edges, none with gap >= 50 | untested | measured / untested | Chain stitch diverged: canvas 57,504 x 56,538 px (0 skipped pairs, median 732 inliers). Tree placement unfinished at 4,400/19,110 pairs; checkpoint resumable. |

### Older takes (2026-09-26 and 2026-10-07)

| take | date | duration (s) | color frames | motion class | loop evidence | wind evidence | confidence (loop / wind) | notes |
|---|---|---|---|---|---|---|---|---|
| 20260926_102608b | 2026-09-26 | 5.7 | 173 | still | untested | no wind (Jinming, Oct 9) | untested / reported (Jinming Oct 9) |  |
| 20260926_102631c | 2026-09-26 | 4.1 | 124 | still | untested | no wind (Jinming, Oct 9) | untested / reported (Jinming Oct 9) |  |
| 20260926_102706d | 2026-09-26 | 4.6 | 138 | still | untested | no wind (Jinming, Oct 9) | untested / reported (Jinming Oct 9) |  |
| 20260926_102737a | 2026-09-26 | 3.9 | 117 | still; translation uncertain | untested | no wind (Jinming, Oct 9) | untested / reported (Jinming Oct 9) | Noisier IMU than the other Sept takes (imu_fusion.py per-interval noise 0.336 mm); proxy 0.219 mm sits in the uncertain band. |
| 20260926_153136 | 2026-09-26 | 3.2 | 96 | still | untested | untested | untested / untested |  |
| 20260926_153610 | 2026-09-26 | 4.2 | 128 | still | untested | untested | untested / untested |  |
| 20260926_153825 | 2026-09-26 | 6.5 | 196 | still | untested | untested | untested / untested |  |
| 20260926_155411 | 2026-09-26 | 5.8 | 176 | still | untested | untested | untested / untested |  |
| 20260926_155419 | 2026-09-26 | 1.8 | 55 | rotation (moderate); translation uncertain | untested | untested | untested / untested | Very short take: 55 color frames, 1.8 s. |
| 20260926_155430 | 2026-09-26 | 5.0 | 150 | still | untested | untested | untested / untested |  |
| 20260926_155507 | 2026-09-26 | 7.0 | 210 | still | untested | untested | untested / untested |  |
| 20260926_155555 | 2026-09-26 | 10.1 | 305 | still | untested | untested | untested / untested |  |
| 20260926_155614 | 2026-09-26 | 5.3 | 160 | still | untested | untested | untested / untested |  |
| 20261007_075947 | 2026-10-07 | 7.8 | 235 | translation+rotation (gentle) | untested | untested | untested / untested |  |
| 20261007_080033 | 2026-10-07 | 14.7 | 441 | translation+rotation (moderate) | untested | untested | untested / untested | Strongest translation signal of the older takes (1.08 mm/interval mean). |

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

## What this table does not claim

- Loop status is established for one take only (173807: no). Whether any other
  take loops is untested; the stitch test itself is what can confirm a loop.
- Wind status is established for five takes only (173701: inconclusive with a
  signal; four September takes: no wind per Jinming). Every other take is
  untested, including takes whose camera barely moves.
- Motion classes come from the IMU alone. "Uncertain" means the measurement
  cannot separate the cases, not that the feature is absent.
