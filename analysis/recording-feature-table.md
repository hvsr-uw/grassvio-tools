# Recording feature table — all 49 takes

Date: 2026-10-30, extended 2026-11-01 and 2026-11-06. Built for Jinming's Oct 28 ask: comb
through all the recordings and tabulate what features each one has (wind,
loop, translation, rotation, translation + rotation, etc.). Loop and wind
checks run on all 35 usable takes on 2026-10-30, on the 5 new takes on
2026-11-01, and on the 8 new Nov-06 takes on 2026-11-06 (see
analysis-drafts/new-takes-20261106/SUMMARY.md for the Nov-06 batch,
including its new illumination check); every cell below is measured except where marked.

Coverage: 49 .db3 takes from five Box download sessions
(sess-197710181, sess-1191256009, sess-1861756418, sess-rotation-nov01,
sess-2653029502) —
21 takes recorded 2026-10-26 (Jinming's loop/wind set), 5 takes recorded
2026-10-30 (Jinming's "more rotation" set), 8 takes recorded 2026-11-06
(Jinming's "wind blowing and illumination change" set), plus 15 older takes from
2026-09-26 and 2026-10-07. Every bag was resolved by its exact filename.
All reads were read-only; no bag was modified, moved, or renamed.

## Correction (2026-11-09): translation-proxy column

The translation-proxy mean, median, and max in the IMU appendix were
wrong for the 36 takes recorded 2026-09-26, 2026-10-07, and 2026-10-26.
They were computed with an earlier integration variant (explicit Euler
over the accel samples inside each frame interval, each with its full
gap-to-next-sample dt), which reproduces the published values to within
a few percent on 35 of the 36 rows, and came out about half the values the stated
method gives. The stated method is semi-implicit Euler with
interval-clipped dt, the same as imu_fusion.py step 3; the script was
changed to that method for the 2026-11-01 extension and the old rows
were never recomputed. Recomputing all 49 takes with the stated method
reproduces the 13 newer rows (2026-10-30 and 2026-11-06) exactly and
gives the corrected values now in the appendix. Two headline rows:
173701 mean 0.132 mm becomes 0.236 mm (median 0.109 becomes 0.217,
max 0.579 becomes 0.729); the control 102631c mean 0.084 mm becomes
0.142 mm (median 0.074 becomes 0.128, max 0.277 becomes 0.326). The
corrected values agree exactly with imu_fusion.py's own displacement
numbers on the four September takes it supports (0.121, 0.142, 0.169,
0.397 mm). One row, 102608b (published 0.020/0.007/0.284 mm), matches
neither the old variant nor the stated method; its corrected values
(0.121/0.110/0.336 mm) are the recomputed ones and the original error
there is unexplained.

Wind and loop verdicts are unaffected: the wind verdicts rest on
rigid-fit residuals against the control, and the loop verdicts rest on
SIFT end-vs-start inliers; neither uses the proxy column. The
still-take noise floor in the Methods text is corrected from about
0.1-0.15 mm to about 0.1-0.25 mm per interval.

What does change: 12 takes cross a translation band under the
corrected means. Motion-class labels were left as published and need
a re-check against the corrected bands: 172859 (0.353 mm, now
uncertain, labeled rotation-only), 172912, 172955, 173250, 173507
(0.429-0.543 mm, now above the 0.4 mm present bar, labeled
uncertain), and 155419 (0.419 mm, labeled uncertain). The three
high-rotation takes that cross 0.4 mm (172644, 172739, 173045) stay
uncertain under the gravity-leakage rule. 153825 and 173701 cross
from negligible to uncertain but carry no translation verdict in
their motion class.

## Methods

Per-take stream stats were computed with one uniform script over all 41 bags:
color/depth message counts, color span (duration), gyro magnitude stats (mean,
median, p95, fraction of samples above 0.25 rad/s) from header timestamps, and
accel mean magnitude. The 21 Oct-26 takes were independently inspected on
Oct 27; the recomputed numbers here reproduce those values. The 5 Oct-30
takes were measured with the same procedure on Nov 1.

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
above 0.4 mm (about 2-4x the still-take noise floor of ~0.1-0.25 mm) and the
signal clearly exceeds the gravity leakage plausible at that rotation level;
on high-rotation takes the leakage can be large, so those stay "uncertain"
regardless of the number. 0.2-0.4 mm is "uncertain"; below 0.2 mm is
"negligible". The 0.2 mm line is
arbitrary: under the corrected values 172859 (0.353 mm) and 173017 (0.366 mm)
sit together in the uncertain band, 0.013 mm apart.

Loop check (all 40 usable takes): SIFT-match the last 5 stride-5 chosen frames
against the first 5 (25 pairs; Lowe 0.75, RANSAC 3.0 px — the stitch script's
own settings). Verdict YES if any pair reaches 20+ inliers (the stitch
script's MIN_INLIERS), else NO. This tests end-vs-start closure, the practical
meaning of a "loop take" for loop-closure testing. On takes where the camera
never moves, the end trivially overlaps the start; those are marked trivially
closed, not genuine loops.

Wind check (all 40 usable takes): rigid-fit residual analysis per consecutive
frame pair, the same method as the Oct 27 wind test on 173701. Pooled
median/p90 residual per take, compared against the wind-free control 102631c
(recomputed here as pooled median 0.185 px / p90 0.458 px, confirming the
pipeline). Bands: "no wind signal" at control level (median <= 0.25 and
p90 <= 0.60); "non-rigid signal, not separated from parallax" when clearly
above (median > 0.30 or p90 > 0.80); "uncertain" between the bands or when too
few pairs fit. Never "wind yes" — the test cannot separate wind from parallax
over 3D grass, and no take is labeled as wind.

## Evidence rules

- Loop measured on all 48 usable takes (end-vs-start, 25 pairs each).
  Genuine loops, camera moved and end overlaps start: 080033 (all 25 pairs at
  1444-1882 inliers), 075947 (all 25 pairs at 133-195 inliers), 155419 (max
  728 inliers on a 1.8 s take, rotation in place). Weak but consistent,
  above the 20-inlier bar across the matrix: 172532 (max 45), 172828
  (max 37), 172859 (max 51). All still takes close trivially — the camera
  never left, so the end overlaps the start with hundreds to thousands of
  inliers (e.g. 102737a: 2129, 174745: 1486, 085611: 3731). 173807 is confirmed NO again
  (max 0, all 25 pairs zero), on top of the earlier 4,400-pair tree
  evidence. The four moving Oct-30 takes are all NO (174702, 174818,
  174847: max 0; 174918: max 12). The four moving Nov-06 takes are all NO
  (085224, 085301, 085422, 085453: max 0, all 25 pairs zero). Every other moving take: NO.
- Wind measured on all 48 usable takes against the 102631c control. No wind
  signal: 102608b, 102631c, 102706d, 102737a, 153136, 153610, 153825, 155411,
  075947, 080033, 172532, 172644, 172739, 173045, 173403, 174702, 174745,
  174818, 174847, 174918 (the Oct-30 takes all sit at control level:
  pooled medians 0.180-0.225 px), 084542, 084741, 084852, 085224, 085301,
  085422, 085453, 085611 (the Nov-06 takes all sit at or below control
  level: pooled medians 0.093-0.181 px, the four still takes the cleanest
  in the dataset so far at 0.093-0.108 px). Uncertain: 155419,
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
| 20260926_102737a | 2026-09-26 | 3.9 | 117 | still; translation uncertain | yes (trivial — camera still) | no wind (Jinming Oct 9; measured at control level) | measured / reported (Jinming Oct 9) + measured | Noisier IMU than the other Sept takes (imu_fusion.py per-interval noise 0.336 mm); proxy 0.397 mm sits in the uncertain band. |
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
| 20261007_080033 | 2026-10-07 | 14.7 | 441 | translation+rotation (moderate) | yes (max 1882) | no wind signal (measured) | measured / measured | Strongest translation signal of the older takes (2.01 mm/interval mean). Genuine loop: all 25 end-vs-start pairs match at 1444-1882 inliers. |

### New takes (2026-11-06, Jinming's wind-blowing and illumination-change set)

| take | date | duration (s) | color frames | motion class | loop evidence | wind evidence | confidence (loop / wind) | notes |
|---|---|---|---|---|---|---|---|---|
| 20261106_084542 | 2026-11-06 | 11.8 | 356 | still | yes (trivial — camera still) | no wind signal (measured) | measured / measured | Still take, grass with fallen leaves. Brightness range 8.54, max jump 1.01 (mild slow drift). Frame differences show intermittent camera micro-shake (uniform ~1 px translation field, two episodes), not grass motion. |
| 20261106_084741 | 2026-11-06 | 13.8 | 415 | still | yes (trivial — camera still) | no wind signal (measured) | measured / measured | Still take. Brightness range 8.59, max jump 1.72. A moving shadow sweeps across the lawn mid-take (localized pixel change, no feature motion). |
| 20261106_084852 | 2026-11-06 | 17.1 | 513 | still | yes (trivial — camera still) | no wind signal (measured) | measured / measured | Still take, leaf-covered lawn. Brightness range 8.05, max jump 0.97. One illumination event near pair 310 (shadow; SIFT feature count halves during it, no feature motion). |
| 20261106_085224 | 2026-11-06 | 17.5 | 524 | rotation (high); translation uncertain | no (max 0) | no wind signal (measured) | measured / measured | Brightness range 33.89 (smooth rise 115 to 140, mostly the view changing as the camera rotates over different ground). |
| 20261106_085301 | 2026-11-06 | 14.4 | 432 | rotation (high); translation uncertain | no (max 0) | no wind signal (measured) | measured / measured | Largest brightness swing in the batch (range 66.21): flat near 118 for 6 s, then a smooth wave to 167 by the end as the camera rotates from shaded leaf litter into a sunlit patch. |
| 20261106_085422 | 2026-11-06 | 13.7 | 411 | rotation (moderate); translation uncertain | no (max 0) | no wind signal (measured) | measured / measured | Brightness range 20.45, wobbly with a moving shadow band (visible as a diagonal darker band in its panorama). |
| 20261106_085453 | 2026-11-06 | 13.1 | 393 | rotation (high); translation uncertain | no (max 0) | no wind signal (measured) | measured / measured | Brightness range 39.30; the one sharp 10.14 jump is a white sign entering the view as the camera rotates past it, not a light change. People's shadows visible in frames. |
| 20261106_085611 | 2026-11-06 | 12.8 | 386 | still | yes (trivial — camera still) | no wind signal (measured) | measured / measured | Still take. Cleanest genuine illumination change in the batch: brightness oscillates repeatedly between about 90 and 114 (range 23.31) on a fixed view of grass and leaves. No pixel-motion event anywhere in the take. |

### New takes (2026-10-30, Jinming's more-rotation set)

| take | date | duration (s) | color frames | motion class | loop evidence | wind evidence | confidence (loop / wind) | notes |
|---|---|---|---|---|---|---|---|---|
| 20261030_174702 | 2026-10-30 | 32.0 | 960 | rotation (high); translation uncertain | no (max 0) | no wind signal (measured) | measured / measured | Longest take in the set (32 s). High rotation throughout; translation proxy 0.588 mm sits above the present bar but stays uncertain under the gravity-leakage rule. |
| 20261030_174745 | 2026-10-30 | 6.6 | 200 | still; translation uncertain | yes (trivial — camera still) | no wind signal (measured) | measured / measured | Still take (manhole cover on lawn). End-vs-start max 1486 inliers. Proxy 0.235 mm in the uncertain band. |
| 20261030_174818 | 2026-10-30 | 21.1 | 633 | rotation (high); translation uncertain | no (max 0) | no wind signal (measured) | measured / measured | Strongest rotation of the set (gyro mean 0.44 rad/s, 83% of samples above 0.25). |
| 20261030_174847 | 2026-10-30 | 16.7 | 501 | rotation (high); translation uncertain | no (max 0) | no wind signal (measured) | measured / measured | Highest gyro mean of the set (0.68 rad/s, 96% above 0.25). |
| 20261030_174918 | 2026-10-30 | 7.3 | 220 | rotation (high); translation uncertain | no (max 12) | no wind signal (measured) | measured / measured | Short high-rotation take. End-vs-start max 12 inliers, below the bar. |

## Appendix: measured per-take IMU and stream numbers

All values measured read-only from the .db3 files with one uniform script. Gyro stats use header timestamps. Translation proxy: per color-frame interval, accelerometer demeaned by the take mean, double-integrated from zero velocity (same method as imu_fusion.py step 3); reported in mm per interval.

| take | color n | depth n | gyro n | gyro mean (rad/s) | gyro median (rad/s) | gyro p95 (rad/s) | frac > 0.25 rad/s | accel mean (m/s^2) | transl. proxy mean (mm) | transl. proxy median (mm) | transl. proxy max (mm) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 20261026_172532 | 1058 | 1058 | 7020 | 0.24719 | 0.2557 | 0.48831 | 0.512 | 9.7614 | 0.84 | 0.807 | 2.612 |
| 20261026_172642 | 6 | 6 | 39 | 0.07212 | 0.06209 | 0.12516 | 0.0 | 9.8908 | 0.219 | 0.159 | 0.364 |
| 20261026_172644 | 698 | 698 | 4670 | 0.33976 | 0.3375 | 0.55399 | 0.7647 | 9.7747 | 0.588 | 0.555 | 2.016 |
| 20261026_172739 | 649 | 649 | 4336 | 0.31723 | 0.30986 | 0.51304 | 0.6797 | 9.7813 | 0.606 | 0.573 | 1.704 |
| 20261026_172828 | 642 | 642 | 4291 | 0.38252 | 0.37489 | 0.6547 | 0.7833 | 9.8011 | 0.813 | 0.781 | 2.855 |
| 20261026_172859 | 163 | 163 | 1094 | 0.11987 | 0.10846 | 0.24608 | 0.043 | 9.7327 | 0.353 | 0.317 | 1.549 |
| 20261026_172912 | 390 | 390 | 2611 | 0.15684 | 0.14408 | 0.29983 | 0.1306 | 9.7591 | 0.429 | 0.421 | 0.949 |
| 20261026_172955 | 347 | 348 | 2323 | 0.14602 | 0.11787 | 0.39435 | 0.1386 | 9.764 | 0.43 | 0.405 | 1.934 |
| 20261026_173017 | 276 | 276 | 1836 | 0.12963 | 0.11975 | 0.23691 | 0.0349 | 9.7703 | 0.366 | 0.326 | 1.363 |
| 20261026_173045 | 795 | 795 | 5313 | 0.3233 | 0.32204 | 0.54848 | 0.7237 | 9.7821 | 0.712 | 0.674 | 1.887 |
| 20261026_173124 | 590 | 590 | 3945 | 0.16815 | 0.14019 | 0.35793 | 0.1947 | 9.7625 | 1.002 | 0.851 | 2.474 |
| 20261026_173154 | 419 | 419 | 2802 | 0.2059 | 0.18057 | 0.45693 | 0.3055 | 9.7393 | 1.039 | 1.029 | 2.273 |
| 20261026_173220 | 562 | 562 | 3758 | 0.17651 | 0.14721 | 0.4553 | 0.1772 | 9.7465 | 1.251 | 1.315 | 2.625 |
| 20261026_173250 | 287 | 287 | 1920 | 0.11825 | 0.09676 | 0.28542 | 0.0823 | 9.7549 | 0.46 | 0.443 | 1.011 |
| 20261026_173403 | 194 | 194 | 1298 | 0.12844 | 0.08422 | 0.32473 | 0.2042 | 9.7359 | 0.377 | 0.376 | 0.852 |
| 20261026_173507 | 305 | 305 | 2038 | 0.13175 | 0.11615 | 0.27003 | 0.0903 | 9.7538 | 0.543 | 0.496 | 1.536 |
| 20261026_173540 | 482 | 482 | 3221 | 0.1299 | 0.12794 | 0.23855 | 0.0444 | 9.6986 | 0.96 | 0.866 | 2.385 |
| 20261026_173603 | 428 | 428 | 2862 | 0.11577 | 0.10273 | 0.2551 | 0.0563 | 9.6914 | 0.785 | 0.777 | 1.681 |
| 20261026_173628 | 348 | 348 | 2331 | 0.1119 | 0.10568 | 0.24164 | 0.0455 | 9.7 | 0.848 | 0.737 | 2.315 |
| 20261026_173701 | 254 | 254 | 1701 | 0.06317 | 0.05491 | 0.12515 | 0.0 | 9.7094 | 0.236 | 0.217 | 0.729 |
| 20261026_173807 | 979 | 979 | 6549 | 0.1076 | 0.09393 | 0.23594 | 0.0351 | 9.7044 | 1.055 | 1.066 | 2.156 |
| 20260926_102608b | 173 | 173 | 1157 | 0.03407 | 0.0306 | 0.06866 | 0.0 | 9.7401 | 0.121 | 0.11 | 0.336 |
| 20260926_102631c | 124 | 124 | 828 | 0.05026 | 0.04891 | 0.08934 | 0.0 | 9.7454 | 0.142 | 0.128 | 0.326 |
| 20260926_102706d | 138 | 138 | 919 | 0.07227 | 0.05409 | 0.18742 | 0.0239 | 9.7365 | 0.169 | 0.159 | 0.378 |
| 20260926_102737a | 117 | 118 | 777 | 0.0727 | 0.04787 | 0.20422 | 0.0373 | 9.7345 | 0.397 | 0.374 | 1.234 |
| 20260926_153136 | 96 | 96 | 634 | 0.03902 | 0.02996 | 0.10877 | 0.0 | 9.7472 | 0.109 | 0.094 | 0.378 |
| 20260926_153610 | 128 | 128 | 843 | 0.03308 | 0.03058 | 0.06633 | 0.0 | 9.7429 | 0.095 | 0.092 | 0.243 |
| 20260926_153825 | 196 | 196 | 1309 | 0.05826 | 0.04891 | 0.13316 | 0.0015 | 9.744 | 0.228 | 0.199 | 0.57 |
| 20260926_155411 | 176 | 177 | 1179 | 0.05096 | 0.04528 | 0.10381 | 0.0 | 9.7499 | 0.136 | 0.128 | 0.331 |
| 20260926_155419 | 55 | 54 | 363 | 0.22381 | 0.21863 | 0.32375 | 0.2066 | 9.7824 | 0.419 | 0.355 | 0.846 |
| 20260926_155430 | 150 | 150 | 1003 | 0.06125 | 0.04829 | 0.1399 | 0.009 | 9.7421 | 0.119 | 0.104 | 0.46 |
| 20260926_155507 | 210 | 210 | 1403 | 0.03592 | 0.03337 | 0.06778 | 0.0 | 9.7546 | 0.122 | 0.118 | 0.301 |
| 20260926_155555 | 305 | 305 | 2036 | 0.03971 | 0.03535 | 0.08199 | 0.0 | 9.7422 | 0.118 | 0.107 | 0.442 |
| 20260926_155614 | 160 | 160 | 1069 | 0.03851 | 0.03624 | 0.0724 | 0.0 | 9.7411 | 0.134 | 0.131 | 0.335 |
| 20261007_075947 | 235 | 235 | 1562 | 0.13056 | 0.10742 | 0.30572 | 0.0832 | 9.7327 | 0.764 | 0.769 | 1.775 |
| 20261007_080033 | 441 | 440 | 2945 | 0.22608 | 0.15489 | 0.64511 | 0.2706 | 9.7507 | 2.01 | 1.999 | 3.861 |
| 20261030_174702 | 960 | 960 | 6413 | 0.29517 | 0.27534 | 0.60902 | 0.5731 | 9.7358 | 0.588 | 0.526 | 2.255 |
| 20261030_174745 | 200 | 201 | 1340 | 0.0633 | 0.05716 | 0.13394 | 0.0052 | 9.752 | 0.235 | 0.198 | 1.139 |
| 20261030_174818 | 633 | 633 | 4231 | 0.4401 | 0.42173 | 0.81775 | 0.8308 | 9.674 | 1.094 | 1.039 | 3.725 |
| 20261030_174847 | 501 | 500 | 3346 | 0.67886 | 0.65698 | 1.13636 | 0.9611 | 9.7296 | 1.08 | 0.981 | 3.21 |
| 20261030_174918 | 220 | 220 | 1461 | 0.54402 | 0.51722 | 1.0313 | 0.8446 | 9.74 | 1.147 | 1.073 | 3.737 |
| 20261106_084542 | 356 | 355 | 2376 | 0.01469 | 0.01083 | 0.03139 | 0.0029 | 9.7342 | 0.092 | 0.071 | 0.829 |
| 20261106_084741 | 415 | 415 | 2776 | 0.00699 | 0.00614 | 0.01401 | 0.0 | 9.7517 | 0.024 | 0.012 | 0.308 |
| 20261106_084852 | 513 | 513 | 3431 | 0.00593 | 0.00568 | 0.0104 | 0.0 | 9.76 | 0.009 | 0.008 | 0.089 |
| 20261106_085224 | 524 | 524 | 3504 | 0.28184 | 0.23529 | 0.68006 | 0.4795 | 9.7235 | 0.676 | 0.654 | 1.957 |
| 20261106_085301 | 432 | 432 | 2888 | 0.26592 | 0.26363 | 0.51598 | 0.5447 | 9.709 | 0.736 | 0.7 | 1.874 |
| 20261106_085422 | 411 | 411 | 2745 | 0.1878 | 0.17155 | 0.39818 | 0.2321 | 9.728 | 0.46 | 0.415 | 1.368 |
| 20261106_085453 | 393 | 393 | 2629 | 0.28538 | 0.22999 | 0.64046 | 0.4568 | 9.7179 | 0.643 | 0.585 | 1.833 |
| 20261106_085611 | 386 | 387 | 2584 | 0.00592 | 0.00576 | 0.01036 | 0.0 | 9.7412 | 0.007 | 0.005 | 0.115 |

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
| 20261030_174702 | 0.195 | 0.456 | no wind signal |
| 20261030_174745 | 0.18 | 0.443 | no wind signal |
| 20261030_174818 | 0.208 | 0.493 | no wind signal |
| 20261030_174847 | 0.225 | 0.531 | no wind signal |
| 20261030_174918 | 0.209 | 0.505 | no wind signal |
| 20261106_084542 | 0.108 | 0.312 | no wind signal |
| 20261106_084741 | 0.108 | 0.308 | no wind signal |
| 20261106_084852 | 0.097 | 0.279 | no wind signal |
| 20261106_085224 | 0.181 | 0.461 | no wind signal |
| 20261106_085301 | 0.18 | 0.484 | no wind signal |
| 20261106_085422 | 0.171 | 0.423 | no wind signal |
| 20261106_085453 | 0.176 | 0.431 | no wind signal |
| 20261106_085611 | 0.093 | 0.266 | no wind signal |

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
