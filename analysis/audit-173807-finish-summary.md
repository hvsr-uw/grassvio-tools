# Finishing take 20261026_173807 — final verdict

Date: 2026-11-09. Take 173807 is the one remaining not-good deliverable
in the project: every earlier attempt at a map of it fails. This task
tried the remaining render variants on the best available poses
(ORB-SLAM3 RGB-D-only) and checked whether loop closure fires. All
numbers below are read from the matching `_v2.json` sidecars or run
logs, and every map listed was opened and judged by eye, by the
coordinator as well as the workers.

Eye test for GOOD: coherent patch, no radial streaks dominating,
holes mostly filled, grass texture readable across the map.

## Verdict up front

**PROVEN UNFIXABLE from this recording.** No variant tried produces
a GOOD map of 173807, and the failure is not in the renderer: the
streaks survive every render change, appear in both halves of the
take independently, and loop closure cannot help because the take
never revisits its start. What would be needed is new data — a
re-recording of the same ground with the camera held mostly downward
and slower, ideally returning to its start so loop closure can
anchor the trajectory. See "What new data would fix it" below.

The least-bad render is the 5 mm average-fill at 50 deg tilt
(`variants/besttilt_5mm_173807_v2_avg_fill.png`): finer detail than
the baseline, but the radial smear still covers the whole map and
black holes remain. It is NOT good. There is no good map to deliver
for this take.

## What was already closed before this task

- Homography chain (direct 57,504 x 56,538 px canvas; normalized
  20,373 x 24,820 px), tree placement (19,110 of 19,110 pairs, tree
  identical to the direct chain to the pixel), homography pose graph
  with loop edge (self-consistent to ~1 px, still a sheared
  diamond): all diverge. The take has no loop closures (all 25
  end-vs-start SIFT pairs at 0 inliers).
- RGB-D visual-odometry orthomosaic: a smeared blotch.
- ORB-SLAM3 RGB-D-only poses: the best so far. Total rotation
  217.0 deg against a gyro total of 201.4 deg; median tilt 26.2
  deg; 730 of 979 frames inside the 35 deg tilt limit. The map
  from those poses is 7.76 x 5.08 m at 56.0% coverage, with real
  texture in the middle, spiky radial streaks at the edges, and
  black holes. Better, not good. That map is the baseline for the
  variants below.

## Variant 1 — wider tilt allowance

Renderer `render_v2.py`, unmodified; edge filter on, 1 cm grid,
px-step 2. Each run writes average, last-wins, and average-fill
blends plus a trajectory overlay.

| Variant | Extent (m) | Coverage | Frames splatted |
|---|---|---|---|
| Baseline, 35 deg | 7.76 x 5.08 | 56.0% | 730/979 |
| Tilt 50 deg | 7.76 x 5.17 | 56.9% | 959/979 |
| Tilt 90 deg | 7.76 x 5.17 | 57.0% | 979/979 |

Eye judgement (coordinator, by eye): the tilt 50 last-wins render
shows the same radial streak burst as the baseline, streaks across
the whole patch, not just the edges; a small detached grey blob
appears below the main patch from the extra tilted frames. The
average blend is smoother but still a radial smear, grass texture
not readable. The fill closes pinholes only; the smear and the black
holes on the right remain. Tilt 90 is indistinguishable from tilt
50 by eye; the extra 20 frames add nothing. NOT good, both.

## Variant 2 — finer grid at the best tilt

Tilt 50 deg was carried forward (a tie with 90 deg, but it splats
fewer stray frames for identical coverage). 5 mm grid, px-step 1,
edge filter on. Note: the renderer reads grid size from the poses
file first, so a copy of the poses file with grid_m 0.005 was made
(`variants/poses_5mm.json`) and passed as `--poses`.

- Extent 7.78 x 5.19 m, coverage 56.5%, 959/979 frames splatted,
  1557 x 1040 cells.
- Eye judgement (coordinator, by eye): finer-grained than the 1 cm
  renders; individual leaf texture is visible in the dense patch at
  the bottom right, and the average-fill is the smoothest blend.
  But the radial streak pattern still covers the whole map, and
  several black holes remain on the right and in the centre. The
  finer grid sharpens detail inside the streaks. It does not change
  the streak geometry or the coverage (56.5% vs 56.9%). NOT good.

## Variant 3 — diagnostic: first half vs second half

`render_v2.py` has no frame-range option, so a copy was made at
`variants/render_half.py` with a `--frame-range START END` option
added (subsets the loaded frames and the pose list to the same
range; everything else identical). Both halves rendered at the
baseline settings (35 deg, edge filter on, 1 cm).

| Half | Extent (m) | Coverage | Frames splatted | Median splatted height |
|---|---|---|---|---|
| First, frames 0-489 | 5.59 x 4.56 | 51.0% | 472/490 | 1.83 m |
| Second, frames 490-978 | 5.86 x 4.87 | 46.5% | 258/489 | 4.01 m |

Eye judgement (coordinator, by eye): neither half is clean. The
first half is streaked the same way as the full map. The second
half is also streaked, fanning from a point on the right side.
The trajectory overlays show the camera path is a nearly straight
line across the middle of the map in both halves, with streaks
fanning out perpendicular to that line in both halves.

Diagnostic answer: the streaks are present through the whole
trajectory, not confined to one stretch. The second half is the
worse-behaved stretch in two measurable ways — only 258 of 489
frames pass the 35 deg tilt gate (vs 472 of 490 in the first
half), and its median splatted height is 4.01 m (vs 1.83 m) —
but it is not the sole source, since the first half streaks too.

## Variant 4 — loop closure on this take

One fresh ORB-SLAM3 RGB-D-only run (same converted data and
settings as the Nov 9 audit run), after a smoke test that
reproduced the 080033 baseline exactly (440 poses). The fresh
173807 run tracked 979/979 poses. Result: loop closure did not
fire. The existing Nov 9 log contains 0 `loop` / 0 `gba` lines;
the fresh log contains 2 `loop` hits that are both the words
"Saving camera trajectory" in the output path, not loop events.
No "Loop detected", "Loop BA", or GBA line appears in either run.
The fresh trajectory matches the existing one (same 979
timestamps, max position difference 0.077 m). This lines up with
the SIFT check: the take never revisits its start, so there is
nothing to close on. The driver prints a line only when a loop
fires, so the evidence is the absence of any loop-event line
across two independent full runs.

## Why this is proven unfixable from this recording

Every placement method tried on this take fails for a measured
reason, and every render variant fails the same eye test:

1. No loop exists to anchor anything. All 25 end-vs-start SIFT
   pairs have 0 inliers; ORB-SLAM3 loop closure fires nowhere
   in two independent runs. A loop-based fix is impossible on
   this recording, not just untried.
2. The render is not the bottleneck. Tilt gating (35 to 90 deg),
   blending (average / last-wins / fill), grid resolution
   (1 cm to 5 mm, px-step 2 to 1), and trajectory stretch (both
   halves independently) were all varied, and the streak pattern
   survives every change with coverage stuck at 56-57%.
3. The remaining suspects are upstream of the renderer: the poses
   themselves (orientation / height drift, visible in the
   second-half numbers: median splatted height 4.01 m vs 1.83 m,
   tilt-gate pass rate 258/489 vs 472/490) or depth quality at
   grazing angles. Both are properties of how the take was
   recorded — a fast, high-rotation sweep in which the camera
   spent much of the take looking sideways — not of how it was
   rendered. No render parameter can put back ground the camera
   never looked at from above, or straighten poses that drifted.

## What new data would fix it

A re-recording of the same ground, with the camera held mostly
downward (low tilt throughout, so most frames pass the tilt gate),
slower motion, and ideally a return to the start so loop closure
can anchor the trajectory. The depth-to-color warp (5.9 cm camera
baseline, depth used as-is) remains an open improvement, but it
would sharpen texture, not remove the streak geometry — it is not
the fix for this take.

## Files

- `SUMMARY.md` — this file.
- `variants/RENDER_NOTES.md` — every render variant with exact
  commands, numbers, and eye judgements.
- `variants/PROGRESS.md` — checkpoint log.
- `variants/tilt50_173807_v2_*`, `tilt90_173807_v2_*`,
  `besttilt_5mm_173807_v2_*`, `half_first_173807_v2_*`,
  `half_second_173807_v2_*` — average / last-wins / average-fill /
  trajectory-overlay renders plus `_v2.json` sidecars.
- `variants/poses_5mm.json`, `variants/render_half.py`,
  `variants/run_step1/2/3.sh` + `.log` — re-runnable scripts and
  logs.
- `loopcheck/LOOP_NOTES.md` — loop-closure evidence with exact
  grep counts.
- `loopcheck/173807_loopcheck.log`,
  `loopcheck/173807_loopcheck_traj.txt` — fresh ORB run (979
  poses).
- `loopcheck/080033_smoke.log`, `loopcheck/080033_smoke_traj.txt`
  — smoke test (440 poses).
