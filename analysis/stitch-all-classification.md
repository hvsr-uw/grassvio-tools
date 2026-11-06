# Stitch-all classification — chain SIFT on every usable take

Date: 2026-11-01. Run: batch_stitch.py, chain mode, SIFT (10000 features),
stride 5, Lowe 0.75, RANSAC 3.0 px, min 20 inliers — the same settings as
the stitch baselines. 40 takes (20261026_172642 excluded: failed recording).
Panoramas and per-take stats in panoramas/.

Classes:
- worked: the panorama reads as a map. Frames line up, no wild stretching,
  the take's motion is visible as a sensible layout.
- sort-of-ok: a map comes out but with visible problems — stretched regions,
  heavy blur from drift, or large parts of the take not placing well.
- blow-up: no usable map. The chained canvas diverged (guard: 8000 px),
  or the run failed.

Each line below is one take with the evidence behind the call. Canvas sizes
and inlier counts come from the run stats; the visual judgment comes from
opening each panorama.

Tally over the 40 usable takes: 17 worked, 6 sort-of-ok, 17 blow-ups.
Every September take worked. Both October 7 takes produced maps (075947
clean, 080033 stretched at the far end). Of the 20 usable October 26
takes, 12 blew up — the chain diverges on most of the takes with real
camera motion, even when no pair is skipped and inlier counts look
healthy (e.g. 173124: 0 skipped, median 1421 inliers, canvas 387k x 456k).
The 7 that stitched split into 2 clean (172859, 173403) and 5 with
stretch or heavy blur. Of the 5 new October 30 takes, the 4 with high
rotation all blew up and only the still take (174745) stitched clean.

## Per-take verdicts

| take | verdict | frames used / skipped | median inliers | canvas (px) | evidence |
|---|---|---|---|---|---|
| 20260926_102608b | worked | 35 / 0 | 2029 | 654 x 494 | Still take. Frames line up into one coherent lawn map. |
| 20260926_102631c | worked | 25 / 0 | 1692 | 672 x 507 | Still take. Clean coherent map, matches the earlier baseline exactly. |
| 20260926_102706d | worked | 28 / 0 | 1795 | 770 x 542 | Still take. Coherent map with the path edge visible. |
| 20260926_102737a | worked | 24 / 0 | 2661 | 699 x 574 | Still take. Coherent map, noisier IMU but the stitch is clean. |
| 20260926_153136 | worked | 20 / 0 | 978 | 664 x 498 | Still take. Backyard scene lines up, grill and bins in place. |
| 20260926_153610 | worked | 26 / 0 | 980 | 667 x 520 | Still take. Same backyard, coherent map. |
| 20260926_153825 | worked | 40 / 0 | 1000 | 713 x 529 | Still take. Street scene lines up, houses and trees in place. |
| 20260926_155411 | worked | 36 / 0 | 1183 | 749 x 575 | Still take. Bush close-up is 3D but the map stays coherent. |
| 20260926_155419 | worked | 11 / 0 | 843 | 1004 x 736 | 1.8 s rotation in place. Map fans out coherently, no tear. |
| 20260926_155430 | worked | 30 / 0 | 920 | 693 x 516 | Still take. Bush and car line up. |
| 20260926_155507 | worked | 42 / 0 | 817 | 818 x 647 | Still take. Coherent map. |
| 20260926_155555 | worked | 61 / 0 | 908 | 694 x 508 | Still take. Porch scene lines up, door and steps in place. |
| 20260926_155614 | worked | 32 / 0 | 1017 | 680 x 509 | Still take. Same porch scene, coherent. |
| 20261007_075947 | worked | 47 / 0 | 3195 | 1138 x 786 | Genuine loop take. Rectangular lawn map, no stretch. |
| 20261007_080033 | sort-of-ok | 89 / 1 | 2352 | 2351 x 2101 | Genuine loop take. Map comes out and the loop reads, but the far end stretches and shears visibly. |
| 20261026_172532 | blow-up | 212 / 0 | 551 | 142611 x 131316 | High-rotation take. Pairs match fine (0 skipped) but the chained canvas diverges to 142k x 131k px. No warp attempted. |
| 20261026_172644 | blow-up | 140 / 0 | 460 | 184808 x 14974 | High-rotation take. 0 skipped pairs, median 460 inliers, but the chained canvas diverges to 184808x14974 px. No warp attempted. |
| 20261026_172739 | blow-up | 130 / 0 | 518 | 1193188 x 97166 | High-rotation take. 0 skipped pairs, median 518 inliers, but the chained canvas diverges to 1193188x97166 px. No warp attempted. |
| 20261026_172828 | blow-up | 129 / 0 | 313 | 43247 x 9879 | High-rotation take. 0 skipped pairs, median 313 inliers, but the chained canvas diverges to 43247x9879 px. No warp attempted. |
| 20261026_172859 | worked | 33 / 0 | 1584 | 1182 x 705 | Gentle rotation. Brick building and bushes line up into a sensible map, some smear in the middle. |
| 20261026_172912 | blow-up | 78 / 0 | 313 | 38411 x 7165 | Gentle-rotation take. 0 skipped pairs, median 313 inliers, but the chained canvas diverges to 38411x7165 px. No warp attempted. |
| 20261026_172955 | blow-up | 70 / 0 | 354 | 184547 x 5137 | Gentle-rotation take. 0 skipped pairs, median 354 inliers, but the chained canvas diverges to 184547x5137 px. No warp attempted. |
| 20261026_173017 | blow-up | 56 / 0 | 208 | 139034 x 24018 | Gentle-rotation take. 0 skipped pairs, median 208 inliers, but the chained canvas diverges to 139034x24018 px. No warp attempted. |
| 20261026_173045 | blow-up | 159 / 0 | 250 | 120809 x 17934 | High-rotation take. 0 skipped pairs, median 250 inliers, but the chained canvas diverges to 120809x17934 px. No warp attempted. |
| 20261026_173124 | blow-up | 118 / 0 | 1421 | 387172 x 455680 | Translation+rotation take. 0 skipped pairs, median 1421 inliers, but the chained canvas diverges to 387172x455680 px. No warp attempted. |
| 20261026_173154 | blow-up | 84 / 0 | 647 | 28939 x 14099 | Translation+rotation take. 0 skipped pairs, median 647 inliers, but the chained canvas diverges to 28939x14099 px. No warp attempted. |
| 20261026_173220 | blow-up | 113 / 0 | 244 | 29880 x 65467 | Translation+rotation take. 0 skipped pairs, median 244 inliers, but the chained canvas diverges to 29880x65467 px. No warp attempted. |
| 20261026_173250 | sort-of-ok | 58 / 0 | 220 | 2396 x 789 | Gentle rotation. A map comes out but the building smears into a thin diagonal streak; low inlier count. |
| 20261026_173403 | worked | 39 / 0 | 2522 | 1138 x 784 | Moderate rotation. Garden scene lines up: brick wall, bushes, grasses all in place. |
| 20261026_173507 | blow-up | 61 / 0 | 1319 | 156535 x 120330 | Gentle-rotation take. 0 skipped pairs, median 1319 inliers, but the chained canvas diverges to 156535x120330 px. No warp attempted. |
| 20261026_173540 | sort-of-ok | 97 / 0 | 142 | 835 x 594 | Translation+rotation take. Map comes out but very blurry throughout; low inlier count. |
| 20261026_173603 | sort-of-ok | 86 / 0 | 281 | 825 x 611 | Translation+rotation take. Map comes out but smeared diagonally; low inlier count. |
| 20261026_173628 | sort-of-ok | 70 / 0 | 421 | 731 x 885 | Translation+rotation take. Bush scene mostly lines up but blurry, with a stray strip at the bottom right. |
| 20261026_173701 | sort-of-ok | 51 / 0 | 1165 | 811 x 554 | Still take. Map comes out but very blurry from averaging 51 near-identical frames. |
| 20261026_173807 | blow-up | 196 / 0 | 732 | 57505 x 56539 | Translation+rotation take. 0 skipped pairs, median 732 inliers, but the chained canvas diverges to 57k x 56k px. No warp attempted. |
| 20261030_174702 | blow-up | 192 / 0 | 1116 | 26000 x 8437 | High-rotation take (longest in the set, 32 s). 0 skipped, median 1116 inliers, but the chained canvas diverges. No warp attempted. |
| 20261030_174745 | worked | 40 / 0 | 2095 | 679 x 490 | Still take. Clean lawn map with a manhole cover, no stretch. |
| 20261030_174818 | blow-up | 127 / 0 | 896 | 43189 x 30390 | High-rotation take. 0 skipped, median 896 inliers, but the chained canvas diverges. No warp attempted. |
| 20261030_174847 | blow-up | 101 / 0 | 620 | 64872 x 5776 | High-rotation take (highest gyro mean of the set). Chained canvas diverges. No warp attempted. |
| 20261030_174918 | blow-up | 44 / 0 | 737 | 14574 x 1470 | High-rotation take. Chained canvas diverges. No warp attempted. |

## Addendum — Nov 6 takes (2026-11-06)

Same method, run 2026-11-06 on the 8 new takes (Jinming's wind
blowing and illumination change set). All 8 stitched, 0 skipped
pairs, no blow-ups — the best-behaved batch yet. Median inliers are
1900-3777, far above the Oct 26 takes, likely because the fallen
leaves all over the lawn are distinctive features everywhere.

| take | verdict | frames used / skipped | median inliers | canvas (px) | evidence |
|---|---|---|---|---|---|
| 20261106_084542 | worked | 72 / 0 | 3160 | 644 x 492 | Still take. Clean sharp single-view map. |
| 20261106_084741 | worked | 83 / 0 | 2629 | 643 x 483 | Still take. Clean single-view map. |
| 20261106_084852 | worked | 103 / 0 | 3119 | 642 x 482 | Still take. Clean single-view map. |
| 20261106_085224 | worked | 105 / 0 | 2095 | 1052 x 914 | Rotation. Coherent fan, leaves line up; thin far-end streak, small. |
| 20261106_085301 | worked | 87 / 0 | 2777 | 1394 x 957 | Rotation. Coherent fan; mild blur in the densest leaf litter. |
| 20261106_085422 | worked | 83 / 0 | 2518 | 848 x 694 | Rotation. Coherent map; the moving shadow shows as a diagonal darker band. |
| 20261106_085453 | sort-of-ok | 79 / 0 | 1900 | 1054 x 860 | Rotation. Coherent fan, but the far end stretches into a thin diagonal streak and there is a brightness seam across the middle. The white sign the camera sweeps past does not appear in the map. |
| 20261106_085611 | worked | 78 / 0 | 3777 | 642 x 482 | Still take. Clean sharp single-view map. |

New tally for the batch: 7 worked, 1 sort-of-ok, 0 blow-ups. Overall
across all 48 usable takes: 24 worked, 7 sort-of-ok, 17 blow-ups.
