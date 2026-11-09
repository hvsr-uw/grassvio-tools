# Synthetic abrupt-brightness test — SIFT matching vs injected intensity steps

Date: 2026-11-09. CS 566 GrassVIO project only.

## What this is, plainly

This is a synthetic manipulation of real recorded frames. It is NOT a
full simulator, NOT a recorded cloud event, and NOT a test of the whole
SLAM pipeline. It tests one thing only: how sensitive the SIFT
matching stage is to an abrupt global brightness change between two
frames. The step is a uniform gray-level shift applied in memory to
copies of real frames. The bags on disk were opened read-only and were
never modified.

This is the smallest honest form of the simulation idea raised in the
team chat (synthesizing data to test the algorithm, including abrupt
intensity change from clouds). A real cloud edge would change light
unevenly across the frame (moving shadows, partial coverage). This test
does not do that. It measures the global-shift case only.

## Method

Takes: the wind-free control 20260926_102631c (124 frames, 640x480) and
20261106_085611 (386 frames, 640x480), the take with the cleanest
genuine illumination change in the Nov 6 batch.

For each take, 10 consecutive stride-5 pairs were spread evenly
through the take. For each pair, the first frame was left untouched
and a constant gray-level step was added to the second frame only, so
the step sits between the two frames of the pair, mimicking a cloud
edge passing between frames. Values were clipped to 0-255. The same
10 pairs were measured unmodified (baseline) and under each step size,
so every comparison is paired on identical geometry.

Step sizes (gray levels): +10, +25, +50, +100, -25, -50.

Matching (same settings as the rest of this project): SIFT with
10000 features, Lowe ratio 0.75, homography RANSAC at 3.0 px, 20+
inliers counts as a fit. Recorded per pair: good matches, inliers,
fit flag, median inlier residual.

Ramp control: the same total change (+50 and -25) spread linearly over
31 frames, so each stride-5 pair inside the ramp carries only about
8.3 (for +50) or 4.2 (for -25) gray levels. Two ramp windows per take,
12 pairs each. Each ramp pair was also matched unmodified, so the ramp
comparison is paired against its own baseline (ramp_baseline.json).

Scripts: synth_brightness.py (step test), ramp_baseline.py (paired
ramp baselines), plot_synth.py (plots). Per-pair numbers are in
20260926_102631c.json and 20261106_085611.json.

## Result: SIFT barely notices, even at large steps

No pair failed at any step size, on either take. Fit fraction is 10/10
for every step size on both takes, and 12/12 for every ramp condition.

Median inliers per pair (10 pairs per take):

| step | control 102631c (baseline 1681) | 085611 (baseline 3884.5) |
|---|---|---|
| +10 | 1682 (+0.1%) | 3871.5 (-0.3%) |
| +25 | 1674 (-0.4%) | 3847.5 (-1.0%) |
| +50 | 1646.5 (-2.1%) | 3788.5 (-2.5%) |
| +100 | 1545.5 (-8.1%) | 3114.5 (-19.8%) |
| -25 | 1688.5 (+0.4%) | 3868 (-0.4%) |
| -50 | 1568.5 (-6.7%) | 3423.5 (-11.9%) |

The worst single pair anywhere in the test still had 1479 inliers
(control, +100) and 2041 inliers (085611, +100) — far above the
20-inlier fit threshold. Median inlier residuals barely move: control
0.192 px at baseline vs 0.189-0.213 px across all steps; 085611
0.097 px at baseline vs 0.099-0.124 px. Keypoint counts on the
modified frame barely change except at the extremes.

The +100 and -50 losses are partly a clipping effect, not a pure
shift effect: at +100, 7.7% of pixels saturate on the control take
and 11.3% on 085611; at -50, 5.3% and 7.7% hit black. At +50 only
2.9% / 1.9% of pixels clip. The example frame sheet shows what this
looks like: at +100 the frame is visibly washed out.

![degradation curve](degradation_curve.png)

## Abruptness barely matters; magnitude does (a little)

Ramp vs single step, same total change, change in median inliers vs
each condition's own paired baseline:

| total change | control: step | control: ramp | 085611: step | 085611: ramp |
|---|---|---|---|---|
| +50 | -2.1% | -0.8% | -2.5% | -0.3% |
| -25 | +0.4% | +0.3% | -0.4% | -0.1% |

Spreading the +50 change over 31 frames recovers most of the small
loss, so abruptness does cost something — but the whole effect is
about 2%, which is within the pair-to-pair spread of the baseline
itself (baseline pairs on 085611 range from 2520 to 4199 inliers).
At -25 there is nothing to recover: step and ramp are both flat.

![ramp vs step](ramp_vs_step.png)

![example frames](example_step_frames.png)

## Comparison with the real-data illumination findings

This is where the test earns its keep. On the real Nov 6 batch,
brightness jumps cost far more than the synthetic uniform step does:
on 085611 the correlation between real jump size and per-pair inliers
was -0.52, with median inliers 3435 on high-jump pairs vs 4010 on
low-jump pairs (about a 14% drop), at real jumps of at most 4.39 gray
levels. The synthetic test says a uniform +10 step costs 0.3% and a
uniform +25 step costs 1.0% on the same take. A uniform global shift
cannot explain the real 14% drop.

The likely reason, stated as a reading of the numbers rather than a
proven cause: real illumination change is not a uniform shift. It is
spatially varying — shadows sweeping across the lawn, part of the
frame going dark while the rest stays lit — which changes local
contrast and gradients, the exact things SIFT descriptors are built
from. A global level shift mostly cancels out in gradient-based
descriptors, which is why this test finds almost nothing. The real
cost measured on Nov 6 must come from the spatial structure of the
light change, not its size.

## Verdict

SIFT matching is robust to abrupt global brightness steps up to about
+/-50 gray levels (loss under 3%, no pair ever close to failing), and
still fits every pair at +/-100 / -50 with thousands of inliers.
Abruptness adds only a small extra cost over a gradual change of the
same size. As a test of the matching stage against cloud-like
intensity change, this uniform-step version is a null result — and it
is a useful null, because it rules out "the level changed" as the
explanation for the real 14% match loss on 085611 and points at
spatially varying light (moving shadows) instead. A follow-up that
actually tests the cloud case would need a spatially varying
synthetic change (for example a dark band sweeping across the frame),
not a bigger global step.

## Limits

- Only the matching stage was tested (SIFT + RANSAC on frame pairs).
  Nothing here says anything about tracking, odometry, or mapping
  under brightness change.
- Only two takes, both still-camera, 10 step pairs and 12 ramp pairs
  per take. The numbers are medians over small samples, but the
  effect is so small and the margins so large (thousands of inliers
  vs a 20-inlier threshold) that the verdict does not depend on the
  sample size.
- The step is a uniform global shift with clipping. Real cloud edges
  are spatially varying and were not tested.
- Negative steps darken toward black and positive steps saturate
  toward white; at +/-50 and beyond, part of the measured loss is
  pixels hitting the clip bounds, not the shift itself.

## Files

- SUMMARY.md — this file.
- synth_brightness.py — step test (step sizes, pair selection,
  matching settings).
- ramp_baseline.py — paired unmodified baselines for the ramp windows.
- plot_synth.py — plot generation.
- 20260926_102631c.json, 20261106_085611.json — per-pair numbers and
  summaries for the step test and the ramps.
- ramp_baseline.json — per-pair numbers for the ramp windows,
  unmodified.
- run.log — console log of the runs.
- degradation_curve.png — median inliers vs step size, both takes.
- ramp_vs_step.png — abrupt step vs gradual ramp, same total change.
- example_step_frames.png — unmodified vs +50 vs +100 on the same
  frame, showing the clipping at +100.
