# Loop closure: RGB-D pose errors and the stable-homography test

Date: 2026-11-05. Take stamps are exact bag filenames. All numbers measured
on this date; no thresholds invented. Methods: SIFT end-vs-start loop check
from the recording feature table (25 pairs, 20+ inliers = YES); RGB-D VO
poses from rgbd_odometry.py (SIFT + depth back-projection + solvePnPRansac,
stride 5); stable-homography test from dataset-tools/stitch-map/
stable_homography.py (new script; stitch_map.py and rgbd_odometry.py
untouched). Bags read read-only.

Note on stamps: the "155419" take is 20260926_155419 (Sept 26), not Oct 26.

## Task 1 — RGB-D loop-closure error on the three earlier takes

Start pose vs end pose of the chained RGB-D VO trajectory. A take that
closes should return near its start in both translation and rotation.

| take | visual loop (SIFT end vs start) | RGB-D start-end translation | RGB-D start-end rotation | VO pairs ok |
|---|---|---|---|---|
| 20260926_155419 | YES, max 728 inliers | 0.108 m | 19.6 deg | 10/10 |
| 20261007_075947 | YES, 133-195 inliers | 0.580 m | 14.8 deg | 46/47 |
| 20261007_080033 | YES, all 25 pairs 1444-1882 | 0.836 m | 67.6 deg | 86/89 |
| 20261026_173807 | NO, all zero | 4.926 m | 149.6 deg | 195/196 |

Reading: the VO closure error grows with take length and rotation. 155419
(11 poses, 1.8 s rotation in place) closes to 0.11 m. 080033 is a genuine
visual loop — the end frames match the start frames at 1444-1882 inliers —
but the VO chain still ends 0.84 m and 67.6 deg away from its start. The gap
between the visual match and the VO pose is the VO drift on that take
(88 chained pairs, camera tips to about 67 deg mid-take). RGB-D VO alone
does not detect the loop precisely; the visual match does. 173807 does not
close by either measurement.

## Task 2 — RGB-D on the remaining loop takes

| take | visual loop (SIFT) | RGB-D start-end translation | RGB-D start-end rotation | VO pairs ok | path length |
|---|---|---|---|---|---|
| 20260926_155419 | YES (max 728) | 0.108 m | 19.6 deg | 10/10 | 0.17 m |
| 20261026_172532 | weak (max 45) | 6.102 m | 101.0 deg | 211/212 | 14.31 m |
| 20261026_172828 | weak (max 37) | 5.394 m | 67.3 deg | 128/129 | 15.94 m |
| 20261026_172859 | weak (max 51) | 0.838 m | 29.4 deg | 32/33 | 1.01 m |

Reading: the two long weak takes (172532, 172828) do not close by pose
(5-6 m, 67-101 deg) — the small SIFT counts (37-45) are consistent with
repetitive-grass matches, not a loop. 172859 barely moves at all (1.01 m
total path over 33 poses), so its 0.84 m start-end gap and 51 inliers both
say the same thing: a short, nearly still take, not a loop the camera
returned to. None of the weak takes ran into trouble: every pair chain
completed, no take failed for lack of 3D points (pair inlier medians 265,
488, 1089).

## Task 3 — Julius's "174702 and 174818 are loop"

Both takes measured two ways, side by side. Neither measurement is
presented as a verdict on Julius; the two measurements disagree with the
word "loop" only if "loop" means the camera returns to its start.

| take | SIFT end vs start | RGB-D start-end translation | RGB-D start-end rotation | VO pairs ok | path length | max step |
|---|---|---|---|---|---|---|
| 20261030_174702 | max 0 inliers | 5.100 m | 101.3 deg | 191/192 | 16.04 m | 0.123 m |
| 20261030_174818 | max 0 inliers | 4.331 m | 66.2 deg | 126/127 | 15.81 m | 0.176 m |

Both trajectories are smooth (no jumps; per-step motion 0.10-0.12 m
median) and both end 4-5 m from their start with the camera 3.2-3.7 m
along its initial optical axis. Closest the path ever comes to itself:
174702 passes within 0.153 m of an earlier position once (poses 157 and
167, about 1.7 s apart) but never returns at the end; 174818's closest
approach is 0.306 m between poses 0 and 10, i.e. the camera barely moved
in the first seconds. Orthomosaics: 174702 map 3.02 x 1.57 m (only 30/192
frames within 35 deg of down), 174818 map 5.57 x 5.03 m in two separate
patches (camera tipped through the take).

What Julius might mean, as open possibilities: the recording path crossed
itself briefly mid-take (174702 does, once); the take was recorded with
the intent of returning but the path kept going; or "loop" refers to a
rotation the camera made rather than a return to the same spot. Worth
asking him directly which one he means before the meeting, because the
two measurements above cannot tell these apart.

## Task 4 — Q3: panorama with stable homography on 080033 (Harsh's assignment)

Take: 20261007_080033, 89 chosen frames, 88 chained pairs.

(a) Hartley normalization re-verified: normalized chain canvas 2170 x
1767 px, pair inliers median 2326 (min 20). This reproduces the Nov 2
result exactly (2170 x 1767). The panorama PNG was opened and judged:
grass texture is coherent across most of the map, but the far end (right
side) still stretches and smears. Normalization shrinks the drift; it
does not remove the stretch.

(b) Loop-closure anchor: all 25 first-5 x last-5 pairs match at
1444-1882 inliers (best: frame 85 to frame 4, 1882 inliers). The measured
end-to-start homography is healthy (condition number 1.8e4, near-affine)
and places the last frame nearly on top of the first — the visual loop is
real. But the chain end is not: chained transforms are ill-conditioned
all along (median condition number 1.5e5, max 2.6e5 at frame 70), and the
chain squashes the last frame vertically to 0.33x its height — that is
the far-end stretch in numbers. The correction D = H_loop @ inv(T_end)
has Frobenius norm 755.8. Distributing it linearly across the chain made
things worse: corrected canvas 6766 x 7829 px (render skipped, too
large), against 2170 x 1767 uncorrected. The correction is too large and
too projective for linear distribution; it drags the middle frames
hundreds of pixels off instead of fixing the end.

Verdict: the homography chain is NOT stable on 080033, even with
normalization and a loop anchor. The stable panorama to present is the
RGB-D orthomosaic: 6.7 x 3.2 m, no stretch (analysis-drafts/rgbd/
080033_lastwins.png). The loop anchor is still useful evidence — it
proves the visual loop is real and quantifies the chain error — but
fixing the chain needs more than linear error distribution (a proper
pose-graph optimization over the loop edge, or the RGB-D poses as the
placement).

## What is still unanswered

- Don Q1: RGB-D VO alone does not detect loops precisely (080033: 0.84 m
  and 67.6 deg error on a true visual loop). Loop detection needs the
  visual match plus a pose-graph step; VO drift grows with take length.
- Julius Q2 (IMU-only loop without drifting): not addressed here.
- What Julius means by "loop" for 174702 and 174818: needs one question
  to him, not another measurement.
- Whether pose-graph optimization with the loop edge stabilizes the
  homography panorama: not tried; linear distribution failed.

## Files

- 155419_poses.json, 155419_orthomosaic.png, 155419_trajectory.npy
- 172532_poses.json, 172532_orthomosaic.png, 172532_trajectory.npy
- 172828_poses.json, 172828_orthomosaic.png, 172828_trajectory.npy
- 172859_poses.json, 172859_orthomosaic.png, 172859_trajectory.npy
- 174702_poses.json, 174702_orthomosaic.png, 174702_trajectory.npy
- 174818_poses.json, 174818_orthomosaic.png, 174818_trajectory.npy
- 080033_stable_norm_only.png (normalized chain panorama, far-end
  stretch still visible), 080033_stable_stable.json (canvas sizes, loop
  inliers, correction size)
- Script: dataset-tools/stitch-map/stable_homography.py
