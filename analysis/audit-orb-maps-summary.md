# ORB-SLAM3-pose orthomosaics for the weak maps

Date: 2026-11-09. Work area: `analysis-drafts/full-audit-20261109/orb-maps/`
(under the CS 566 project). Nothing in this task was uploaded to GitHub.
No message was sent to anyone. Bags were read read-only.

## Short version

ORB-SLAM3 RGB-D-only was run on six takes whose current maps are weak,
and each ORB-pose orthomosaic was compared against the existing RGB-D VO
map by eye and by numbers. The ORB map is better on five of the six.
The exception is 20260926_155419, where the two maps show the same
content and the existing map is kept. The take 20261026_172859 was not
attempted: the camera faced a brick wall for the whole take, so it is
not a grass map at all.

The ORB trajectories are credible on every take: total rotation matches
the gyro to within 1 percent on five of the six takes, and within 8
percent on 173807 (217.0 deg against a gyro total of 201.4 deg), while
the VO poses claim the camera tipped to a median 45 to 107 degrees on
the five long takes.
No loop-closure event fired in any ORB run. The maps still carry
streaks from tilted frames, so "better" here means a single coherent
patch with readable texture, not a clean map.

## Method

The ORB-SLAM3 build and the RGB-D-only driver are the ones from the
earlier baseline work. Each bag was converted to TUM-VI style files
with the same converter, ORB-SLAM3 RGB-D-only was run with the same
settings file, and the trajectory was rendered through the same
adapter into the same render machinery used for the 080033 ORB map.
The render and the odometry code were not modified. Render settings
match the existing VO v2 maps (stride 1, 1 cm grid, 2 px step, 3 m
splat cap, 35 degree tilt limit, edge filter on), so the coverage and
extent numbers compare directly. The 173807 take was also rendered
without the edge filter, as in the 080033 ORB map; the two renders
are nearly identical (56.0 percent vs 57.2 percent coverage).

A note on reading the numbers: end displacement is only an error on
a take that returns to its start. On a take that walks away from its
start, a large end displacement is real travel. The checks that carry
weight on these takes are total rotation against the gyro, tilt
plausibility, path-length agreement with the VO chain, and the map
itself.

## Build incident

The VM had restarted since the baseline work, and the apt-installed
packages were gone: the ORB driver binary failed to link (OpenCV,
Boost serialization, and Eigen headers missing). The full dependency
set would not install this time: the `libzstd-dev` package requires
`libzstd1` version 1.5.5+dfsg2-2build1.1, but the VM carries a patched
1.5.5+dfsg2-2build1.1+hatch1 build, so apt refused the whole set.
The prebuilt binary only needs runtime libraries, so those were
installed instead (OpenCV 4.6.0 runtime libraries, Boost
serialization 1.83, GLEW, X11 and GL libraries, image and video
libraries). Two Pangolin libraries still failed to resolve because
they carry no run path of their own; setting `LD_LIBRARY_PATH` to the
Pangolin install library directory fixed that. A smoke test on the
existing 080033 converted data reproduced the baseline exactly
(440 poses), so the build was trusted for the new takes.

## Per-take table

"ORB end" is end-to-start translation and rotation of the ORB
trajectory. "VO end" is the same pair from the earlier VO chain.
"Gyro" is total rotation from the bag gyro, an independent check
since the gyro never enters the RGB-D-only runs.

| take | existing VO map verdict | ORB end / VO end | ORB total rot vs gyro | ORB map extent / coverage / frames | VO map extent / coverage / frames | better map and why |
|---|---|---|---|---|---|---|
| 20261026_173807 | streaky smear, no readable texture | 5.82 m / 40.0 deg / 4.93 m / 149.6 deg | 217.0 deg vs 201.4 deg | 7.76 x 5.08 m / 56.0% / 730 | 5.55 x 5.59 m / 56.8% / 197 | ORB. Larger footprint with real leaf and grass texture in patches, smooth 7.06 m path, gyro-consistent rotation. Still radially streaked with holes, so better but not good. |
| 20260926_155419 | sparse and patchy, wall at top left, green strip at bottom | 0.14 m / 20.3 deg / 0.11 m / 19.6 deg | 23.2 deg vs 23.0 deg | 5.12 x 2.93 m / 18.4% / 55 | 5.16 x 2.54 m / 31.9% / 55 | Tie, keep the VO map. Same content in both (same wall, same strip). The two trajectories agree to 3 cm and 0.7 deg. The VO map covers more of its bounding box, so it stays. |
| 20261030_174702 | small soft wedge, 3.02 x 1.57 m, 30 of 192 frames | 4.05 m / 91.4 deg / 5.10 m / 101.3 deg | 547.0 deg vs 541.9 deg | 4.95 x 4.34 m / 57.7% / 358 | 3.02 x 1.57 m / not measured the same way / 30 | ORB, clearly. One coherent grass patch with visible texture instead of a wedge. The path crosses itself once mid-take, as the earlier loop work found. |
| 20261030_174818 | two separate patches | 2.43 m / 83.8 deg / 4.33 m / 66.2 deg | 531.6 deg vs 532.2 deg | 5.76 x 4.93 m / 57.9% / 294 | 5.62 x 5.11 m / 46.7% / 259 | ORB, clearly. One coherent patch instead of two, with a round manhole-cover feature rendered sharp. A bright strip on the right comes from tilted frames. |
| 20261026_172532 | two separate patches | 0.81 m / 9.3 deg / 6.10 m / 101.0 deg | 501.6 deg vs 497.8 deg | 6.42 x 3.02 m / 53.8% / 569 | 8.48 x 7.35 m / 23.0% / 230 | ORB, clearly. One continuous grass patch instead of two. The trajectory is a big loop that returns to within 0.81 m of its start, which is why one patch is correct and two patches are drift. |
| 20261026_172828 | two separate patches | 0.83 m / 59.0 deg / 5.39 m / 67.3 deg | 474.0 deg vs 469.3 deg | 5.85 x 4.23 m / 43.2% / 197 | 7.61 x 7.15 m / 28.0% / 179 | ORB, clearly. One coherent patch instead of two, with the same round manhole-cover feature sharp, as in 174818. The trajectory is a big loop that returns to within 0.83 m of its start. |

Skipped: 20261026_172859. The camera faced a brick wall for the
whole take, so there is no grass map to build. No run was attempted.

## Per-take notes

### 20261026_173807

979 color frames over 32.6 s, all 979 paired with depth, none
dropped. ORB-SLAM3 produced 979 poses in 136 ms per frame on
average, with no loop event and no lost tracking. The ORB map
covers 7.76 x 5.08 m with 730 frames splatted; the VO map covers
5.55 x 5.59 m with 197 frames. The VO poses claim a median tilt of
107.4 degrees, meaning the camera spent most of the take looking
sideways, which is not plausible for a grass sweep. The ORB poses
claim a median tilt of 26.2 degrees, with 730 of 979 frames inside
the 35 degree limit. The VO map is a compact blob with uniform
diagonal smear. The ORB map is wider, shows real leaf litter and
grass texture in patches, and follows a smooth straight path, but
it is still radially streaked with holes. Verdict: the ORB map is
the best available map for this take, not a clean map.

### 20260926_155419

55 color frames over 1.8 s, 54 paired with depth, 1 dropped.
ORB-SLAM3 produced 54 poses in 77 ms per frame on average. This
take is a rotation in place: path length 0.23 m, ORB end 0.14 m and
20.3 deg, VO end 0.11 m and 19.6 deg. Total rotation matches the
gyro almost exactly (23.2 deg vs 23.0 deg). Both maps show the same
brick wall at top left and the same green grass strip. The ORB map
covers 18.4 percent of a slightly taller bounding box; the VO map
covers 31.9 percent. Verdict: a tie on content, keep the existing
VO map. The ORB run confirms the VO poses on this take rather than
replacing them.

### 20261030_174702

960 color frames over 32.0 s, 959 paired with depth, 1 dropped.
ORB-SLAM3 produced 959 poses in 82 ms per frame on average. Path
length 16.21 m (VO 16.04 m), total rotation 547.0 deg against a
gyro total of 541.9 deg. Median tilt 44.7 deg, so only 358 of 960
frames are inside the 35 degree limit and the rest of the path
runs outside the mapped patch. The trajectory overlay shows the
path crossing itself once mid-take, matching the earlier finding
that this take passes within 0.153 m of an earlier position. The
ORB map is a single coherent grass patch, 4.95 x 4.34 m at 57.7
percent coverage, against the VO wedge of 3.02 x 1.57 m from 30
frames. Verdict: the ORB map is clearly better.

### 20261030_174818

633 color frames over 21.1 s, all paired with depth, none dropped.
ORB-SLAM3 produced 633 poses in 29 ms per frame on average. Path
length 15.75 m (VO 15.81 m), total rotation 531.6 deg against a
gyro total of 532.2 deg. Median tilt 39.6 deg, with 294 of 633
frames inside the limit. The ORB map is one coherent patch,
5.76 x 4.93 m at 57.9 percent coverage, against the VO map in two
patches at 46.7 percent. A round manhole-cover feature in the
left half of the map is rendered sharp, which is a good sign for
pose consistency: a drifting chain would smear a round feature
into an arc. The bright washed strip on the right of the map comes
from tilted frames. Verdict: the ORB map is clearly better.

### 20261026_172532

1058 color frames over 35.3 s, all paired with depth, none
dropped. ORB-SLAM3 produced 1058 poses in 25 ms per frame on
average. Path length 13.84 m (VO 14.31 m), total rotation 501.6
deg against a gyro total of 497.8 deg. The ORB trajectory ends
0.81 m and 9.3 deg from its start, and the overlay shows a big
loop that returns to its start, while the VO chain ends 6.10 m
and 101.0 deg away. The ORB map is one continuous grass patch,
6.42 x 3.02 m at 53.8 percent coverage, against the VO map in two
patches over 8.48 x 7.35 m at 23.0 percent. One caution: the
end-vs-start SIFT check on this take was weak (45 inliers at
most), so the visual loop was not confirmed by SIFT. The map
itself supports the ORB geometry: a falsely closed loop would
paint the same ground twice and double the texture, and the ORB
map shows no doubling. Verdict: the ORB map is clearly better.

### 20261026_172828

642 color frames over 21.4 s, all paired with depth, none
dropped. ORB-SLAM3 produced 642 poses in 28 ms per frame on
average. Path length 15.86 m (VO 15.94 m), total rotation 474.0
deg against a gyro total of 469.3 deg. The ORB trajectory ends
0.83 m and 59.0 deg from its start, and the overlay shows a big
loop, while the VO chain ends 5.39 m and 67.3 deg away. The ORB
map is one coherent patch, 5.85 x 4.23 m at 43.2 percent coverage,
against the VO map in two patches over 7.61 x 7.15 m at 28.0
percent. The same round manhole-cover feature as in 174818
appears sharp in the middle of the map; the two takes cover the
same ground. The same SIFT caution as 172532 applies (weak
end-vs-start matches, 37 inliers at most), and the same answer:
the map shows no doubling. Verdict: the ORB map is clearly
better.

## What the numbers say overall

ORB-SLAM3 RGB-D-only tracked every frame on all six takes (979,
54, 959, 633, 1058, and 642 poses out, matching the paired frame
counts). Its total rotation matches the gyro within 1 percent on
five of the six takes, and within 8 percent on 173807, and its path
lengths agree with the VO chains to
within 0.5 m on the five long takes. The VO chains drift in tilt:
their median tilts run 45 to 107 degrees on the long takes, while
the ORB medians run 26 to 58 degrees. That tilt drift is what
split the VO maps into two patches on 172532, 172828, and 174818,
and what left 174702 as a small wedge. The ORB maps are single
patches on all five long takes. The remaining weakness is shared:
frames held at 25 to 45 degrees of tilt paint long wedges, so the
ORB maps are still streaky, and the bright washed areas on 174818
and 172828 come from tilted frames. The depth-to-color alignment
caveat from the earlier work still applies (5.9 cm camera
baseline, depth used as-is), equally to both sets of maps.

## Files

- `SUMMARY.md` — this file.
- `PROGRESS.md` — dated work log, including the build incident.
- `maps/best/` — the best map per take: average-fill, last-wins,
  and trajectory-overlay PNGs. For 155419 the best map is the
  existing VO v2 map, copied here so the set is complete.
- `maps/` — full render sets per take (average, average-fill,
  last-wins, trajectory overlay, render JSON). The 173807 set
  includes the no-edge-filter variant as well.
- `runs/` — ORB trajectories (`*_rgbd_only_traj.txt`), run logs,
  adapter pose JSONs, and the two small analysis scripts
  (`traj_stats.py`, `tilt_stats.py`).
- `data/` — converted sequences per take (`rgb/`, `depth/`,
  `rgb.txt`, `depth.txt`, `imu.txt`, `info.json`).
- `convert_bag.py`, `orbslam3_pose_render.py`, `D455F.yaml` —
  the converter, the adapter, and the settings file used.
