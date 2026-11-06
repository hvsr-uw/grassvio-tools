# Orthomosaic render v2 — what changed, per take

Date: 2026-11-06. Script: dataset-tools/stitch-map/render_v2.py (new;
stitch_map.py and rgbd_odometry.py untouched, poses NOT recomputed).
Method: poses from the existing *_poses.json files, SE(3) interpolated
between stride-5 anchors so every frame splats (up to 5x more real
points), 5 cm edge-depth filter (median-blur deviation), per-cell
average and last-frame-wins blends, pinhole fill only within 2 cells of
observed data, trajectory overlay variant. Bags read read-only. Every
PNG below was opened and judged against its v1 version.

## Verdicts (show version named per take)

| take | v1 problem | v2 result | show? |
|---|---|---|---|
| 20261026_173807 | noisy, streaky, holes | v2_avg_fill: solid map, 56.8% coverage, holes gone; streaks remain as diagonal bands | YES, v2_avg_fill, with the streak caveat |
| 20260926_155419 | sparse, patchy, pink strip | v2_lastwins: grass wedge now dense and crisp; pink strip is the brick wall, real scene content | YES, v2_lastwins, as garden bed beside a wall |
| 20261030_174818 | two patches, gap | v2_traj: both patches solid, blue path explains the split (camera looped around between them) | YES, v2_traj |
| 20261026_172532 | two patches, mostly empty | v2_traj: both patches solid, path explains the gap | YES, v2_traj |
| 20261026_172828 | two patches, mostly empty | v2_traj: both patches solid, path explains the gap | YES, v2_traj |
| 20261026_172859 | pink-grey smear | v2_lastwins is sharp now — and it is plainly a brick wall with a downspout pipe and bushes. The camera looked at the house for the whole take and never at the ground. | NO, not as a grass map |
| 20261007_075947 (control) | already good | v2_lastwins looks the same as v1, 1.69 x 2.19 m | pipeline does not regress good takes |

## What actually improved things

- Denser splatting is the whole story for 173807, 174818, 172532, 172828:
  splatting every frame (interpolated poses) fills the holes the
  stride-5 maps left. The 173807 map went from speckled to solid.
- The trajectory overlay is what makes the split takes presentable. The
  gap is real (camera tipped mid-take), so it is not filled; the blue
  path with green start and red end dots shows why the map has two
  parts. On 172532 the path is a clean arc from the top patch to the
  bottom patch.
- Last-frame-wins kept grass texture best on 155419, 174818, 172532,
  172828. Averaging smears it. On 173807 the filled average reads
  better as a map because the last-wins version stays speckly.
- The pinhole fill only closes holes up to 2 cells from observed data.
  It does not touch the big empty regions, which stay empty as they
  should.

## What did NOT work

- Ground-band (z-band) filter: tried on 155419 at 0.5 m above the
  median splatted height. Coverage fell from 31.9% to 10.5% — it cut
  the wall but also most of the grass, because the map frame is tilted
  (defined by the first camera frame), so "height" does not separate
  ground from wall. Dropped; no v2 file uses it.
- 173807 streaks are not a rendering problem. They are VO drift (the
  chain ends 4.9 m / 150 deg off on this take), so denser splatting
  cannot remove them. The filled average hides them best.

## Notes on the pink

- 172859: the pink is a red brick wall, confirmed by opening the source
  frames (wall, white downspout, bushes, gravel at the base, every
  frame). The take is a wall, not a map. No render fixes that.
- 155419: the pink strip is the same brick wall next to the grass bed.
  The grass itself is fine.
- 174818 / 172828: pinkish tones are in the source frames (dry grass
  under evening light). The small purple-pink patches in the maps are
  those tones concentrated where splats overlap, not objects.

## Files here

Per take: *_v2_avg.png, *_v2_lastwins.png, *_v2_avg_fill.png,
*_v2_traj.png, *_v2.json (settings, extent, coverage). 075947_control_*
is the no-regression control only; the good takes (080033, 075947,
174702) keep their v1 files unchanged.
