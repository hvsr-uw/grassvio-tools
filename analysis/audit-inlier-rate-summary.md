# RANSAC inlier rate

Nov 10, 2026. 14 videos, 505 pairs, stride 5. All numbers below are read from `results/<stamp>.json` in the `audit-inlier-rate` folder.

## Result

The inlier rate here means inliers over the good matches after the Lowe check. Every video clears half. Overall middle value 0.987, 10th percentile 0.703, only 1 of 505 pairs below half. The lowest video is 173250 at 0.698. The clean videos sit at 0.999 to 1.000.

Two things to know:

1. The normalized fit cannot answer this check. Its rate is 1.000 on every video, because the 3.0 threshold goes into `findHomography` on normalized points, where 3.0 units equals 427 pixels. The normalized inlier count is just the good match count. The direct fit rate is the real number, and that is the one quoted above.
2. Counted the other way, inliers over all matches including the ones the Lowe check rejected, the answer is no. Middle value 0.315, 416 of 505 pairs below half. But that number mostly measures the Lowe check rejecting matches on grass, not the fit. Both ways are in the table.

## How I measured it

`dataset-tools/stitch-map/stitch_map.py`, current default: SIFT nfeatures=10000, BFMatcher L2, knnMatch k=2, Lowe ratio 0.75, `findHomography` RANSAC threshold 3.0 px, minimum 20 good matches or the pair is rejected before RANSAC (0 such pairs in this sample). Frames are stride-5 consecutive pairs, up to 40 pairs per video, sampled evenly when a video has more. Script `inlier_rate.py` in this folder. Driver `run_all.sh` skips videos that are already done.

For each pair I computed both ways of counting, and both fits:
- `rate_good` = inliers / good matches after Lowe 0.75
- `rate_all` = inliers / all knn matches with 2 neighbours
- `norm` = Hartley-normalized fit (pipeline default), `direct` = direct fit (threshold in real pixels)

## Per video

Middle value and 10th percentile of the per-pair rate. `n_good` and `n_inl` are per-pair middle values (direct fit). `failed` = pairs with fewer than 20 good matches (0 everywhere).

| Video | Pairs | Failed | rate_good direct med / p10 | rate_good norm med / p10 | rate_all direct med / p10 | n_good | n_inl |
|---|---|---|---|---|---|---|---|
| 20261007_080033 (loop) | 40 | 0 | 1.000 / 0.975 | 1.000 / 1.000 | 0.453 / 0.031 | 2356 | 2356 |
| 20261007_075947 (loop) | 40 | 0 | 1.000 / 0.999 | 1.000 / 1.000 | 0.506 / 0.440 | 3180 | 3180 |
| 20260926_155419 (loop) | 10 | 0 | 0.962 / 0.853 | 1.000 / 0.999 | 0.334 / 0.170 | 875 | 842 |
| 20261026_172532 | 40 | 0 | 0.992 / 0.986 | 1.000 / 0.998 | 0.204 / 0.142 | 549 | 544 |
| 20261026_172828 | 40 | 0 | 0.930 / 0.779 | 1.000 / 0.997 | 0.099 / 0.069 | 345 | 326 |
| 20261026_172859 | 32 | 0 | 0.949 / 0.721 | 1.000 / 0.999 | 0.350 / 0.108 | 1661 | 1584 |
| 20260926_102631c (control) | 24 | 0 | 0.999 / 0.997 | 1.000 / 0.999 | 0.464 / 0.450 | 1694 | 1692 |
| 20261030_174745 (still) | 39 | 0 | 0.999 / 0.999 | 1.000 / 1.000 | 0.453 / 0.430 | 2095 | 2095 |
| 20261106_085611 (still) | 40 | 0 | 0.999 / 0.998 | 1.000 / 1.000 | 0.686 / 0.469 | 3776 | 3772 |
| 20261026_173701 (wind signal) | 40 | 0 | 0.851 / 0.654 | 1.000 / 0.998 | 0.176 / 0.059 | 1430 | 1164 |
| 20261026_173807 | 40 | 0 | 0.843 / 0.670 | 1.000 / 0.997 | 0.109 / 0.057 | 891 | 718 |
| 20261026_172912 | 40 | 0 | 0.842 / 0.655 | 1.000 / 0.997 | 0.085 / 0.031 | 376 | 303 |
| 20261026_173250 | 40 | 0 | 0.698 / 0.562 | 1.000 / 0.996 | 0.077 / 0.032 | 320 | 216 |
| 20261026_173507 | 40 | 0 | 0.958 / 0.812 | 1.000 / 0.998 | 0.293 / 0.181 | 1394 | 1294 |
| **Overall, 505 pairs pooled** | **505** | **0** | **0.987 / 0.703** | **1.000 / 0.998** | **0.315 / 0.066** | — | — |

The control video's middle inlier count is 1692, the same as the earlier baseline number for that video. The earlier counts were direct fit, and this measurement gives the same numbers.

The clean videos sit at 0.999 to 1.000. The noisier Oct 26 videos are lower, 173250 at 0.698, 172912 at 0.842, 173807 at 0.843, 173701 at 0.851, but every one of them still clears half on the middle value, and only 1 of 505 pairs falls below half. The normalized rate tells nothing apart: it is 1.000 on every video, clean or noisy.

## The threshold is in the wrong units

`fit_homography` in `stitch_map.py` Hartley-normalizes both point sets and then calls `cv2.findHomography(..., cv2.RANSAC, 3.0)` on the normalized points, without rescaling the 3.0 threshold. Checked on one control pair (102631c frames 0-5, 1743 good matches): the normalized point cloud spans 4.40 x 3.30 units, the scale is 0.00702, so a 3.0-unit threshold equals 427 pixels in the original image, most of the 640x480 frame. On 080033 the normalized rate is 1.000 on 33 of 40 pairs, lowest over all 505 pairs is 0.968. The normalized inlier count is the good match count, not a fit quality measure.

What this means:
- The homographies themselves are fine. Normalization helps the fit, and the maps built from them are not affected. Only the inlier count and mask from a normalized fit mean nothing.
- Anything that reads an inlier count from a normalized fit is really reading a good match count: the 20 inlier pair gate, the 50 inlier loop re-anchor, and the tree edge weights.
- The counts quoted before (102631c middle 1692, 080033 loop pairs 1444 to 1882, tree middles) were direct fit and are not affected.
- The fix is to scale the threshold by the normalization (3.0 px times the Hartley scale, about 0.021 units here) or to count inliers by reprojecting with the denormalized H in pixel units.

## Fix (Nov 10)

I fixed it. `fit_homography` in `stitch_map.py` now passes `RANSAC_THRESH_PX * T_dst[0, 0]` as the threshold on normalized points. I checked it after the fix by re-running `inlier_rate.py` on the control video and the noisiest video. The normalized middle rate now equals the direct middle rate (102631c 0.999 = 0.999; 173250 0.698 = 0.698). I also re-ran the 102631c stitch. The normalized map is now 672x507 (it was 676x502 with the broken threshold) and agrees with the direct map to within 1 gray level on 0.01% of pixels. The direct path is not touched by the fix.

## To run it again

```
./run_all.sh   # all 14 videos; skips videos already done
# single video (from the project venv):
python inlier_rate.py --bag <video>.db3 --out results/<stamp>.json --max-pairs 40
```

## Files

- `inlier_rate.py` — measurement script (mirrors `stitch_map.py`).
- `run_all.sh` — driver for the 14-video sample.
- `results/<stamp>.json` — per-pair counts and rates for each video (14 files, 505 pairs total).
