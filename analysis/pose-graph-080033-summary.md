# A good map of take 20261007_080033 — ORB-SLAM3 poses, not homography chaining

Date: 2026-11-07. Take 080033 is the genuine loop take (all 25 end-vs-start
SIFT pairs match, 1444-1882 inliers). The question was how to get a map of
it that actually looks good. The homography panorama is a sheared strip
even with normalization (2170 x 1767 px, far end stretches and smears), and
linear distribution of the loop correction made it worse (6766 x 7829 px).
Harsh's point for this task: optimizing the same homography chain with a
loop edge would just redistribute the same systematic bias, so the primary
strategy had to be different, not the same chaining with a bolt-on. It was:
build the orthomosaic from the ORB-SLAM3 RGB-D-only trajectory
(bundle-adjusted poses, loop-consistent, gyro-matching) instead of from
chained homographies or our VO poses.

## Verdict — best map

**The best map is the unrefined ORB-SLAM3-pose orthomosaic:
`orbslam3-pose/map_orbslam3_080033.png` (6.67 x 4.44 m, 48.8% coverage,
419/441 frames splatted, last-wins).** It is a coherent grass patch with
no stretch or smear, leaves line up across frame boundaries, the far
right end keeps texture with a clean edge, and the take's end lands 6 px
(0.060 m) from its start in the map. The SE(3)-refined version is
indistinguishable from it, so the unrefined map is the deliverable.

## Comparison table (all numbers measured, all maps opened and judged by eye)

| approach | map size / extent | loop end error | visual verdict |
|---|---|---|---|
| Homography chain, normalized | 2170 x 1767 px canvas | 82.9 px loop RMS | sheared strip, far end stretches and smears; not usable |
| Homography chain, linear loop distribution | 6766 x 7829 px canvas | — | worse than the chain; render skipped |
| Homography pose graph + loop edge (control) | 2796 x 2832 px canvas | 1.14 px RMS (all edges) | large sheared diamond, stretch moved into the middle (frame 44: 2.40x to 3.61x); not usable |
| Our RGB-D VO orthomosaic | 6.83 x 3.30 m, 43.1%, 301/441 frames | 0.836 m / 67.6 deg | coherent but vertically smeared, hard seam across the middle, thin sparse right end |
| **ORB-SLAM3-pose orthomosaic** | **6.67 x 4.44 m, 48.8%, 419/441 frames** | **0.061 m / 13.0 deg** | **coherent patch, no stretch or smear, crisp grass, loop closes in the map** |
| ORB-SLAM3-pose + SE(3) loop refinement | 6.64 x 4.43 m, 48.6%, 417/441 frames | 0.058 m / 12.6 deg | indistinguishable from the unrefined map |

## Primary: ORB-SLAM3-pose orthomosaic — works

Adapter script `orbslam3-pose/orbslam3_pose_render.py` converts the 440-pose
TUM trajectory (`analysis-drafts/leftover-three/orbslam3/runs/080033_rgbd_only_traj.txt`)
to a stride-1 poses JSON for `render_v2.py`. No SE(3) alignment was needed:
the ORB trajectory is already in the first-camera frame (first pose
identity), exactly like the VO poses. All 440 poses matched a color frame
exactly by timestamp (441 color vs 440 depth frames; the extra color frame
holds the first pose). `render_v2.py` and `rgbd_odometry.py` untouched.
Render rules as before: color/depth paired by nearest timestamp, splat
depth capped at 3 m, only frames within 35 deg of straight down splatted.

A VO control render through the same machinery reproduces the original VO
orthomosaic closely (6.83 x 3.30 m vs 6.73 x 3.24 m), so the ORB-vs-VO
comparison is fair. The ORB map covers a wider strip (4.44 m vs 3.30 m);
the extra height is genuine footprint, not an artifact — both extremes
come from sustained multi-frame stretches at 22-27 deg tilt, and a crop
of the ORB-only strip shows real grass. Last-wins is the best ORB variant
(a touch crisper than the average). The trajectory overlay shows the ORB
path as a dot with start/end coinciding (6 px apart) while the VO path is
a long line with start/end 83 px apart.

Caveat, recorded not hidden: depth is NOT aligned to color (5.9 cm
baseline between the cameras); the VO renders had the same caveat, so
the comparison is fair. No loop-closure event fired in the ORB run; the
map comes from odometry + local BA alone. No ground-truth map exists;
leaf alignment was judged on grass texture coherence, not landmarks.

## Control: homography pose graph + loop edge — clean null, bias redistributed

Script `homography-graph/pose_graph_080033.py`. Levenberg-Marquardt
Gauss-Newton over all 88 per-frame homographies (pure numpy), minimizing
transfer-error residuals on a 3x3 sample grid for all 88 chain edges plus
the loop edge jointly. This differs from the failed linear distribution:
no correction is prescribed in advance, the loop edge constrains only its
endpoints, and chain edges resist. The loop edge is healthy (best pair
chosen frame 85 into 4, 1882 inliers, cond 1.81e4). The chain reproduces
the earlier result exactly (2170 x 1767 px, 0 skipped, median inliers
2326, chained-T cond median 1.49e5 / max 2.63e5).

Result: 11 iterations, converged, final RMS 1.14 px (chain 1.14, loop
1.39). The optimizer satisfies every edge to about 1 px — yet the canvas
grows to 2796 x 2832 px (29% wider, 60% taller than the plain chain) and
the stretch moves into the middle: frame-centre displacement median 19 px,
max 155 px at frame 54, ends barely move (frame 88: 7 px); frame 44
vertical stretch 2.40x to 3.61x; frame 66: 0.79x to 1.34x; the last frame
is un-pinched (0.55 to 0.92 left-edge) but stretched past true size
vertically (bbox height 1.27x). A weight-10 sensitivity run gives
essentially the same map (loop RMS 0.01 px, canvas 2847 x 2890), so the
result is pinned by the chain edges, not the loop weight.

Judged by eye, the pose-graph map is a large sheared diamond: coherent
grass in the middle, top-left tapering to a thin point, far end fanning
into a broad smeared triangle with radial smear, diagonal stretch
throughout. Still not a usable map. A graph self-consistent to ~1 px
still renders stretched, so the pairwise homographies agree with each
other but are jointly wrong: each carries the same systematic bias from
fitting a planar model to a translating camera over 3D grass, and joint
optimization can only redistribute that coherent bias, not remove it.
Harsh's concern was correct, with numbers.

## Optional: SE(3) pose-graph refinement on the ORB poses — clean null, nothing to fix

Script `se3-refine/se3_refine_080033.py`. The loop edge was measured
directly in SE(3) with depth + PnP (same method as
`rgbd_odometry.pair_pose`): 25/25 end-vs-start pairs succeeded, forward
PnP inliers 1094-1351 per pair; best pair end frame 440 into start frame
2, 1351 inliers. Median implied loop error across pairs: 0.0121 m /
0.63 deg. The measured relative pose itself is 0.0578 m / 12.68 deg,
and the chain implies about 0.061 m / 13.0 deg for the same two nodes:
the PnP measurement confirms the ORB trajectory rather than correcting
it. A reverse-direction check (1526 inliers) puts about 1-3 cm / 1 deg
of directional systematic error on the edge itself (same depth-to-color
caveat), well below the 6 cm / 13 deg end displacement.

Optimization: LM Gauss-Newton over the 440 poses, 439 odometry edges +
1 loop edge, first pose fixed, 10 iterations, cost 2.63e-04 to 5.84e-07.
Loop end error 0.0608 m / 13.00 deg to 0.0578 m / 12.64 deg; loop edge
residual to 0.0000 m / 0.00 deg; node displacement a smooth ramp, median
5.78 mm, max 12.38 mm at node 439. The re-rendered map is
indistinguishable by eye from the unrefined map (blurred average renders
agree to 2.9 gray levels at the best rigid shift; the raw last-wins pixel
difference looks large only because 1 cm pose changes flip which frame
wins each high-frequency grass cell). Verdict: stayed the same, not
crisper, not worse.

Why: most of the 0.061 m / 13.0 deg "loop end error" is genuine camera
displacement — the operator did not return the camera to its exact start
pose — not drift. There was only about 1 cm / 0.6 deg of genuine
inconsistency for the graph to fix, and it fixed all of it. No pose
graph can or should remove genuine displacement. Unlike the homography
graph, nothing was redistributed: the loop edge is satisfied exactly
with a smooth 0-12 mm ramp and a 0.03 mm chain residual. The ORB-SLAM3
trajectory was already right to about a centimeter over the whole take.

## What this settles

- The good map of 080033 comes from bundle-adjusted 3D poses, not from
  any homography chaining, with or without a loop edge. The motion
  model was the problem, and changing it fixed the map.
- The homography pose graph is closed as an approach for this data:
  a self-consistent graph of biased edges stays biased.
- The ORB-SLAM3 RGB-D-only trajectory needs no further refinement on
  this take: its residual loop error is mostly real camera
  displacement, and the map is already coherent.
- Open follow-ups, not done here: warp depth into the color frame
  (5.9 cm baseline) and re-render to see if the map sharpens further;
  check whether loop closure ever fires on a longer take.

## Files

- `SUMMARY.md` — this file.
- `orbslam3-pose/` — `orbslam3_pose_render.py` (adapter),
  `map_orbslam3_080033.png` (best map, last-wins, 668x445),
  `comparison_vs_vo.png` (side-by-side maps + trajectory overlays),
  `PRIMARY_NOTES.md`, `orbslam3_080033_poses.json`, full
  `orbslam3_080033_v2_*` render set, `vo_080033_control_v2_*` control
  renders.
- `homography-graph/` — `pose_graph_080033.py`,
  `map_posegraph_080033.png` (2797x2833), `HOMOGRAPHY_NOTES.md`,
  `Ts_posegraph_w1.npy`, `posegraph_080033.json`, `run.log`.
- `se3-refine/` — `se3_refine_080033.py`,
  `map_orbslam3_refined_080033.png` (665x445), `SE3_NOTES.md`,
  `080033_rgbd_only_traj_refined.txt`,
  `orbslam3_080033_poses_refined.json`, `loop_pairs.json`,
  `refinement_results.json`, `orbslam3_080033_refined_v2_*` render set,
  `run1.log`, `run2.log`.
