# Depth-warp test on the 080033 ORB map — clean null

Date: 2026-11-09. Take 20261007_080033. Work area:
`analysis-drafts/full-audit-20261109/depth-warp/`. Nothing in this task
was uploaded to GitHub. The bag was read read-only. No message was
sent to Jinming, Julius, or Don. ORB-SLAM3 was not re-run. The
existing ORB-SLAM3 poses were reused as-is.

## Verdict — the warp does not sharpen the map

**Warping the depth map into the color frame does not sharpen the
080033 ORB-pose orthomosaic.** The warped map is indistinguishable
from the existing ORB map by eye at full scale and at 2x zoom. The
numbers agree: the multi-frame average render gets blurrier under the
warp, not sharper, and frames disagree with each other more, not
less. The small rise in last-wins gradient is a resampling artifact,
not sharper grass. It is the same size with no baseline correction at
all. The existing unwarped ORB map stays the best map.

## What was warped and how

The depth map is not aligned to the color frame. The color camera
sits 5.9 cm from the depth camera. The existing ORB map pairs each
color pixel with the depth pixel at the same image coordinates. That
attaches each 3D point to the color of a neighboring ray about 20 to
28 pixels away. The median valid depth over the take is 0.86 m. At
that depth the 5.9 cm baseline alone shifts the image by about 26
pixels. This test corrects that shift and re-renders.

Calibration values were read from the bag itself, not copied from
earlier notes. They match the earlier notes except for one
correction: the color intrinsics in `rgbd_odometry.py` do not match
the bag. The bag values below are the ones used.

- Depth camera: fx = fy = 386.009827, ppx = 320.77182,
  ppy = 240.889587, Brown Conrady, all distortion coefficients zero.
- Color camera: fx = 386.040802, fy = 385.300934, ppx = 323.054626,
  ppy = 250.530975, Inverse Brown Conrady, coefficients k1 =
  -0.056599766, k2 = 0.0620031841, p1 = -0.000234535619,
  p2 = 0.000596299826, k3 = -0.0195424054.
- Color tf (`Color_0/tf/ref_0`): rotation near identity (determinant
  1.0, orthonormal to 1e-6), translation t = (-0.059037, 0.000078,
  0.000128) m, baseline |t| = 0.059037 m.
- Color principal point sits 2.3 pixels right and 9.6 pixels below
  the depth principal point.

Warp math per depth pixel, in `warp.py`:

1. Unproject with the depth intrinsics to a point P in the depth
   frame. Only depths from 0.05 m to 3.0 m are used, the same limits
   as the render.
2. Move P into the color frame as P_c = R @ P + t (hypothesis A,
   see the next section).
3. Take the color ray (X_c / Z_c, Y_c / Z_c) and distort it into a
   color pixel with the color intrinsics and the Inverse Brown
   Conrady coefficients. Inverse Brown Conrady means the stored
   coefficients map a distorted point to its undistorted ray by the
   Brown-Conrady polynomial form directly. Undistortion applies
   that form directly. Projection inverts it by fixed-point
   iteration. The inversion code was checked by round trip:
   distort(undistort(p)) returns p to 2e-12 pixels on a grid.
4. Points landing outside the 640x480 color image are dropped.
   Where several depth pixels hit one color pixel, the nearest
   depth wins (Z-buffer).

Two render paths use the same warp, in `render_warped.py`:

- Depth grid: sample depth on the step-2 grid, project each sampled
  depth point into the color image, take the color there, and splat
  the depth-frame point. The set of splatted 3D points is the same
  as the control. Only the color attached to each point changes.
- Color grid: sample color on the step-2 grid, take the warped depth
  there, unproject with the color camera model (undistort the pixel
  to its ray), move the point back into the depth frame with the
  inverse extrinsics, and splat with the ORB pose.

Both paths use the existing ORB poses, the same tilt filter (35
deg), the same depth cap (3 m), the same 0.01 m grid, and the same
blends as `render_v2.py`. `render_v2.py`, `rgbd_odometry.py`, and the
original ORB trajectory were not modified.

## Which way the transform goes

The bag does not label whether its color tf is extrinsics from depth
to color or the pose of the color camera in the depth frame. The two
readings differ by about twice the baseline in image shift, roughly
45 pixels at 1 m. Both were tested.

- Hypothesis A: P_c = R @ P_d + t (tf stored as depth-to-color
  extrinsics).
- Hypothesis B: P_c = R^T @ (P_d - t) (tf stored as the color pose
  in the depth frame).

Three independent checks point to A:

1. Shift search (`shift_search.py`): the offset that best lines raw
   depth edges up with color edges over 9 sample frames is du = -16
   pixels, dv = +12 pixels, score 113.6, against 110.4 at zero
   shift. Hypothesis A predicts about (-20, +10) at 1 m and scores
   112.5. Hypothesis B predicts about (+28, +8) and scores 111.3.
   The vertical signal is the clearer one: at du = -16 the score
   rises steadily with dv, from 104.6 at dv = -16 to 113.6 at
   dv = +12, matching the 9.6 pixel principal-point offset in v.
2. Physical layout: the D455 depth origin is the left IR imager and
   the RGB camera sits to its right. Under A the stored translation
   (-0.059 m in x) is the extrinsics translation for a color camera
   at +0.059 m in the depth frame. Under B the stored translation
   would put the color camera 0.059 m to the left of the left
   imager, outside the device.
3. The maps themselves: A beats B on every sharpness and agreement
   number below, and A drops far fewer points outside the color
   image (about 2.0% of sampled points against about 7.3% for B,
   after the 7.7% invalid-depth share is removed). The D455 depth
   image has a no-data strip on its left edge because the left
   imager is the stereo reference. Hypothesis A shifts depth pixels
   left, so the strip it pushes out of the color image is mostly
   already empty. Hypothesis B shifts them right, pushing valid
   depth out instead.

The per-frame edge test on its own could not separate A from B.
Grass depth edges are weak and color texture is dense everywhere,
so all variants scored within a few percent (raw 110.4, A-inverse
112.7, B-inverse 112.5). The three checks above together settle
the direction as A. The primary warped render uses A.

## Validation that the renderer is faithful

A raw-mode render through the new script reproduces the existing
control map pixel for pixel: mean absolute pixel difference 0.0,
same extent 6.67 x 4.44 m, same coverage 48.8%, same 419 of 441
frames splatted. Any difference in a warped render comes from the
warp alone. The adapter copy in this folder regenerates the poses
JSON with maximum absolute difference 0.0 against the existing
poses file (440 exact timestamp matches, 1 held first frame, worst
nearest gap 33.6 ms, end-to-start translation 0.061 m).

## Numbers — before and after

Control is the existing ORB map (`map_orbslam3_080033.png`,
last-wins). Warped maps are the new renders. Frames splatted is
419 of 441 in every render. The sharpness numbers are computed on
the shared intersection of all five coverage masks (127,311 cells,
42.8% of the raw canvas), after aligning the maps by phase
correlation (shifts 0 to 2 pixels), so extent differences cannot
flatter one map over another.

| map | extent | size | coverage | sampled points dropped |
|---|---|---|---|---|
| raw control | 6.67 x 4.44 m | 668 x 445 | 48.8% | 0 of 32,179,200 |
| warp A, depth grid | 6.67 x 4.40 m | 668 x 441 | 49.0% | 3,118,460 (9.7%) |
| warp B, depth grid | 6.57 x 4.34 m | 658 x 436 | 48.8% | 4,816,294 (15.0%) |
| warp A, color grid | 6.68 x 4.38 m | 669 x 440 | 49.0% | 3,304,172 (10.3%) |
| warp Z, depth grid | 6.63 x 4.38 m | 665 x 440 | 48.9% | 3,533,734 (11.0%) |

Warp Z is the ablation: no baseline (R = identity, t = 0), color
intrinsics and distortion only, same resampling machinery. About
7.7% of sampled depth points are invalid (outside 0.05 to 3.0 m),
so the outside-the-color-image share is about 2.0% for warp A,
about 7.3% for warp B, about 2.6% for warp A on the color grid,
and about 3.3% for warp Z.

| map | last-wins Sobel mean | last-wins Lap var | avg Sobel mean | avg Lap var | avg-vs-lastwins MAD |
|---|---|---|---|---|---|
| raw control | 72.30 | 5023.3 | 46.89 | 761.0 | 12.46 |
| warp A, depth grid | 73.42 (+1.5%) | 5164.2 (+2.8%) | 43.90 (-6.4%) | 579.2 (-23.9%) | 13.00 (+4.4%) |
| warp B, depth grid | 73.90 (+2.2%) | 5209.9 (+3.7%) | 40.97 (-12.6%) | 569.3 (-25.2%) | 13.54 (+8.7%) |
| warp A, color grid | 73.49 (+1.6%) | 5204.1 (+3.6%) | 44.47 (-5.2%) | 602.3 (-20.9%) | 12.99 (+4.2%) |
| warp Z, depth grid | 73.30 (+1.4%) | 5149.9 (+2.5%) | 43.91 (-6.4%) | 622.0 (-18.3%) | 13.19 (+5.9%) |

How to read this table:

- The average render is the multi-frame consensus. A warp that
  puts the right color on each 3D point should make frames agree
  and the average sharper. The average got blurrier instead:
  warp A Sobel -6.4% and Laplacian variance -23.9% against the
  control. Warp B is worse on both.
- Avg-vs-lastwins MAD measures how much frames disagree with each
  other. It rose for every warp: +4.4% for warp A, +8.7% for warp
  B. Frames agree less after warping, not more.
- The last-wins gradient rose slightly for every warp, including
  warp B (+2.2%) and warp Z (+1.4%), which corrects no baseline
  at all. A gain that appears with the wrong direction and with no
  baseline correction cannot be an alignment gain. It is the
  resampling. The warp looks up colors at depth-dependent,
  irregular positions instead of on a regular grid, which adds
  fine speckle inside each frame. That speckle raises single-frame
  gradients and blurs the multi-frame average at the same time.
- Warp A and warp Z are indistinguishable on every number even
  though A corrects about 27 pixels of shift and Z only about 10.
  The measured differences come from the resampling machinery,
  not from the baseline correction. The baseline correction itself
  produces no measurable change in either direction.
- For scale: a 0.5 pixel Gaussian blur on the control average
  render costs 66.5% of its Laplacian variance. The warp costs
  23.9%, so the warp acts like well under half a pixel of extra
  blur. Small, but it is a cost, not a gain.

## Judged by eye

Every map below was opened and looked at, including 2x zoom crops
of the center and the far right end (`comparison_crops.png`).

- At full scale the five maps are indistinguishable: same shape,
  same grass texture, same pinkish patches.
- In the center crop, grass blades look equally sharp in the
  control and in every warp. No leaf edge lines up better under
  warp A. No new doubling appears either.
- In the far-right-end crop, the same small orange speck sits in
  the same place in all five panels. The footprint edge is the
  same shape in all five.
- The warped-depth sample (`sample_warped_depth_frame210.png`)
  looks like a sensible shifted copy of the raw depth: same smooth
  ground gradient, shifted left and down, with the empty strip on
  the correct side. The warp itself ran correctly. It just does
  not show up in the map.

## What this settles

- The depth-to-color warp, done with the correct direction and the
  full distortion model, does not sharpen the 080033 ORB map. The
  existing unwarped ORB map stays the best map.
- The warp direction is A, settled by the shift search, the device
  layout, and the map metrics. B is the wrong direction and is
  worse on every number.
- The reason the test is a null is visible in the ablation: the
  nearest-neighbor resampling in the warp costs about as much as
  the alignment could gain, and the two cannot be separated with
  this machinery. A warp with subpixel color sampling might show
  more, but this test gives no evidence that it would. The grass
  color also changes with viewing angle, which limits how much
  any color alignment can make frames agree.
- ORB-SLAM3 was not re-run. The poses are depth-camera poses and
  the warp only reassigns colors to the same 3D points, so a
  re-run has no reason to change the trajectory. Nothing in this
  null gives a reason to expect a re-run to help.
- Not tested: the "standard" Brown-Conrady reading of the color
  coefficients instead of "inverse". The two differ by a few
  pixels at the image corners against a 27 pixel baseline shift,
  and the edge test preferred them nearly equally (A-standard
  114.8 against A-inverse 112.7, both near raw 110.4). It cannot
  change a null this size.

## Files

- `SUMMARY.md` — this file.
- `calibration.py`, `calibration.json` — calibration read from the
  bag (depth and color intrinsics, color distortion, color tf).
- `warp.py` — warp math, edge-alignment test, shift search
  helpers, sample warped-depth figure.
- `shift_search.py`, `shift_search.log` — the (du, dv) search that
  settled the warp direction.
- `edge_test.log`, `edge_overlay_frame10.png` — per-frame edge
  scores and overlay for frame 10.
- `orbslam3_pose_render.py` — copy of the adapter, with only the
  project-root lookup changed to fit this folder. Regenerates the
  poses exactly (`orbslam3_080033_poses_regen.json`, maximum
  difference 0.0). Original files untouched.
- `080033_rgbd_only_traj.txt` — copy of the original ORB
  trajectory. `orbslam3_080033_poses.json` — the poses as rendered.
- `render_warped.py` — the render script. Raw mode reproduces the
  control map pixel for pixel. Warp modes implement the depth-grid
  and color-grid paths plus hypotheses A, B, and Z.
- `orbslam3_080033_rawval_v2_*` — validation render, pixel
  identical to the control map.
- `orbslam3_080033_warpA_depth_v2_*` — primary warped map
  (hypothesis A, depth grid). Last-wins is the warped map.
- `orbslam3_080033_warpA_color_v2_*` — warp A on the color grid.
- `orbslam3_080033_warpB_depth_v2_*` — warp B, wrong direction,
  kept as the control for the direction question.
- `orbslam3_080033_warpZ_depth_v2_*` — ablation, no baseline.
- `comparison.png` — side-by-side last-wins maps, all five,
  aligned on the raw canvas.
- `comparison_crops.png` — 2x zoom crops, center and far right
  end, all five maps.
- `compare_maps.py`, `compare_numbers.json`, `compare3.log` —
  the numbers behind the tables above.
- `sample_warped_depth_frame210.png` — raw depth against warped
  depth for frame 210.
- `rawval.log`, `warpA_depth.log`, `warpB_depth.log`,
  `warpA_color.log`, `warpZ_depth.log`, `adapter_run.log` — run
  logs.
