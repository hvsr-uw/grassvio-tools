# Stitch detector comparison: SIFT vs ORB

Same script, same inputs, same strides for both detectors. The September take is the bag 20260926_102631c (124 frames, stride 5, 25 frames used). The October take is the bag 20261007_080033 (441 frames, stride 3, 147 frames used). The phone photos are Jinming's 7 pictures from October 22, in EXIF order, stride 1.

The SIFT runs were done twice. The re-runs reproduced the original numbers exactly on both takes, so the SIFT side of this table is verified, not a one-off.

## Verified numbers

| Input | Detector | Frames used | Pairs skipped | Median inliers | Canvas (px) |
|---|---|---|---|---|---|
| September take | SIFT | 25 | 0 of 24 | 1692 | 672 x 507 |
| September take | ORB | 25 | 0 of 24 | 1382 | 672 x 505 |
| October take | SIFT | 147 | 1 of 146 | 2251 | 2204 x 1891 |
| October take | ORB | 147 | 0 of 146 | 1164 | 2309 x 1987 |
| Phone photos | SIFT | 7 | 2 of 6 | 67 | 7123 x 2654 |
| Phone photos | ORB | 7 | 5 of 6 | 29 | 1282 x 1975 |

Pairs skipped means the pair found too few good matches, so the script placed that frame with the previous frame's transform instead of its own.

## What the maps look like

September take: both maps look clean and nearly identical in size. SIFT and ORB both work here.

October take: both maps are grass panoramas, but both are stretched toward the far end. The ORB map is stretched more, which matches its lower inlier count. The extra canvas size on the ORB map (2309 x 1987 against 2204 x 1891) is drift, not extra coverage.

Phone photos: the SIFT map has a recognizable top strip (driveway, parked car, tree shadows) and a smeared lower half. The ORB map is basically one photo: with 5 of 6 pairs skipped, the frames landed on top of each other instead of spreading into a panorama.

## Reading of the results

On normal walking takes from the camera, both detectors finish the job, and SIFT finds more inliers (1692 vs 1382 on the September take, 2251 vs 1164 on the October take). On the longer take, ORB skips nothing but its noisier matches cost more drift at the far end.

On the handheld phone photos, SIFT partially works and ORB breaks. The phone photos are the harder input for both: big rotation between shots, a 3D lawn scene that one homography cannot explain, and weak overlap between some neighbours.

## What this means for the project

For the panorama part, SIFT is the safer default. ORB is fine on easy takes and it is much cheaper to compute, but it should not be trusted on the hard inputs. The bigger problem is not the detector: the script only matches neighbouring frames, so nothing corrects the chained drift, and a loop walk that revisits an earlier frame never gets used. That loop check is the next fix, before any detector tuning.
