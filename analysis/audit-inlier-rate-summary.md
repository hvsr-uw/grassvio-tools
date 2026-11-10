# RANSAC inlier rate — Jinming's check "RANSAC inlier rate (>50%)?"

Date: 2026-11-10. Measured fresh on 14 takes, 505 stride-5 pairs.
All numbers below are read from `results/<stamp>.json` in this folder.

## Verdict up front

Under the standard meaning of RANSAC inlier rate — inliers over the
good matches fed to RANSAC (after Lowe 0.75) — **YES, every take
clears the 50% bar by a wide margin.** Overall median 0.987, 10th
percentile 0.703, only 1 of 505 pairs below 0.50. The lowest
per-take median is 0.698 (173250); the clean takes sit at 0.999-1.000.

Two qualifications, both measured, not assumed:

1. **The pipeline's current default (Hartley-normalized fitting)
   cannot answer this check.** Its inlier rate is 1.000 on every
   take, because the 3.0 px RANSAC threshold is passed to
   `findHomography` on Hartley-normalized points, where 3.0 units
   equals 427 pixels (verified below). The normalized "inlier count"
   is just the good-match count. The direct-fit rate is the real
   measurement, and it is the one quoted in the verdict.
2. **Under the other possible reading — inliers over ALL knn
   matches, including ones Lowe's test rejected — the answer is NO:**
   overall median 0.315, 416 of 505 pairs below 0.50. But that
   number mostly measures Lowe rejection on repetitive grass, not
   RANSAC quality, so it is the wrong reading of the check. Both
   definitions are reported in the table so Jinming can see which
   one he meant.

## Method (copied from the pipeline, nothing invented)

`dataset-tools/stitch-map/stitch_map.py`, current default:
SIFT nfeatures=10000, BFMatcher L2, knnMatch k=2, Lowe ratio 0.75,
`findHomography` RANSAC threshold 3.0 px, minimum 20 good matches
or the pair is rejected before RANSAC (0 such pairs in this sample).
Frames are stride-5 consecutive pairs, up to 40 pairs per take,
sampled evenly when a take has more. Script: `inlier_rate.py` in
this folder; driver: `run_all.sh` (skips takes whose result file
already exists, so it resumes after a restart).

Per pair, both definitions and both fitting variants were computed:
- `rate_good` = inliers / good matches after Lowe 0.75 (headline)
- `rate_all` = inliers / all knn matches with 2 neighbours
- `norm` = Hartley-normalized fit (pipeline default), `direct` =
  direct fit (threshold in real pixels)

## Per-take table

Median and 10th percentile of the per-pair rate. `n_good` and
`n_inl` are per-pair medians (direct fit). `failed` = pairs with
fewer than 20 good matches (0 everywhere).

| Take | Pairs | Failed | rate_good direct med / p10 | rate_good norm med / p10 | rate_all direct med / p10 | n_good | n_inl |
|---|---|---|---|---|---|---|---|
| 20261007_080033 (loop) | 40 | 0 | 1.000 / 0.975 | 1.000 / 1.000 | 0.453 / 0.031 | 2356 | 2356 |
| 20261007_075947 (loop) | 40 | 0 | 1.000 / 0.999 | 1.000 / 1.000 | 0.506 / 0.440 | 3180 | 3180 |
| 20260926_155419 (loop) | 10 | 0 | 0.962 / 0.853 | 1.000 / 0.999 | 0.334 / 0.170 | 875 | 842 |
| 20261026_172532 (weak loop) | 40 | 0 | 0.992 / 0.986 | 1.000 / 0.998 | 0.204 / 0.142 | 549 | 544 |
| 20261026_172828 (weak loop) | 40 | 0 | 0.930 / 0.779 | 1.000 / 0.997 | 0.099 / 0.069 | 345 | 326 |
| 20261026_172859 (weak loop) | 32 | 0 | 0.949 / 0.721 | 1.000 / 0.999 | 0.350 / 0.108 | 1661 | 1584 |
| 20260926_102631c (control) | 24 | 0 | 0.999 / 0.997 | 1.000 / 0.999 | 0.464 / 0.450 | 1694 | 1692 |
| 20261030_174745 (still) | 39 | 0 | 0.999 / 0.999 | 1.000 / 1.000 | 0.453 / 0.430 | 2095 | 2095 |
| 20261106_085611 (still) | 40 | 0 | 0.999 / 0.998 | 1.000 / 1.000 | 0.686 / 0.469 | 3776 | 3772 |
| 20261026_173701 (wind signal) | 40 | 0 | 0.851 / 0.654 | 1.000 / 0.998 | 0.176 / 0.059 | 1430 | 1164 |
| 20261026_173807 (unfixable) | 40 | 0 | 0.843 / 0.670 | 1.000 / 0.997 | 0.109 / 0.057 | 891 | 718 |
| 20261026_172912 (ordinary) | 40 | 0 | 0.842 / 0.655 | 1.000 / 0.997 | 0.085 / 0.031 | 376 | 303 |
| 20261026_173250 (ordinary) | 40 | 0 | 0.698 / 0.562 | 1.000 / 0.996 | 0.077 / 0.032 | 320 | 216 |
| 20261026_173507 (ordinary) | 40 | 0 | 0.958 / 0.812 | 1.000 / 0.998 | 0.293 / 0.181 | 1394 | 1294 |
| **Overall, 505 pairs pooled** | **505** | **0** | **0.987 / 0.703** | **1.000 / 0.998** | **0.315 / 0.066** | — | — |

Cross-check: the control take's median direct inlier count is 1692,
exactly reproducing the previously reported 102631c baseline median
(1692). The earlier counts were direct-fit, and this measurement
reproduces them.

What the direct rate separates: the clean takes (both loop takes,
control, both still takes) sit at 0.999-1.000 with p10 at 0.975 or
above. The noisier Oct-26 takes are lower — 173250 at 0.698,
172912 at 0.842, 173807 at 0.843, 173701 at 0.851 — but every one
of them still clears 50% on the median, and only 1 of 505 pairs
falls below 0.50 on the direct rate. The normalized rate separates
nothing: it is 1.000 (or 0.999) on every take, clean or noisy.

## Finding: the normalized RANSAC threshold is in the wrong units

`fit_homography` in `stitch_map.py` Hartley-normalizes both point
sets and then calls `cv2.findHomography(..., cv2.RANSAC, 3.0)` on
the normalized points, without rescaling the 3.0 threshold. Verified
on one control pair (102631c frames 0-5, 1743 good matches): the
normalized point cloud spans 4.40 x 3.30 units, the scale is
0.00702, so a 3.0-unit threshold equals 427 pixels in the original
image — most of the 640x480 frame. On 080033 the normalized rate is
exactly 1.000 on 33 of 40 pairs (minimum 0.968 over all 505 pairs);
the normalized inlier count is the good-match count, not a fit
quality measure.

Consequences, stated carefully:
- The homographies themselves are fine — normalization helps the
  conditioning of the fit, and the maps built from them are
  unaffected. Only the inlier COUNT and mask from a normalized fit
  are meaningless.
- Any pipeline decision that reads an inlier count from a normalized
  fit is really reading a good-match count: the 20-inlier pair gate,
  the 50-inlier loop re-anchor, and the tree edge weights. The
  historical counts quoted in earlier summaries (102631c median
  1692, 080033 loop pairs 1444-1882, tree medians) were direct-fit
  and are unaffected.
- The fix, if the count is wanted from normalized fits, is to scale
  the threshold by the normalization (3.0 px times the Hartley
  scale, about 0.021 units here) or to count inliers by reprojecting
  with the denormalized H in pixel units. Not applied here — this
  task only measures; the fix is a one-line change for the owner
  of `stitch_map.py` to approve.

## Fix applied (Nov 10, after this measurement)

Harsh approved the one-line fix. `fit_homography` in
`stitch_map.py` now passes `RANSAC_THRESH_PX * T_dst[0, 0]` as the
threshold on normalized points. Verified after the fix by re-running
`inlier_rate.py` on the control take and the noisiest take:
normalized median rate now equals the direct median exactly
(102631c 0.999 = 0.999; 173250 0.698 = 0.698). Regression check on
the 102631c stitch: the normalized map is now 672x507 (it was
676x502 with the broken threshold) and agrees with the direct map
to within 1 gray level on 0.01% of pixels. The direct path is
untouched by the fix.

## Re-running

```
./run_all.sh   # all 14 takes; skips takes already done
# single take (from the project venv):
python inlier_rate.py --bag <take>.db3 --out results/<stamp>.json --max-pairs 40
```

## Files

- `SUMMARY.md` — this file.
- `inlier_rate.py` — measurement script (mirrors `stitch_map.py`).
- `run_all.sh` — driver for the 14-take sample.
- `run_all.log` — per-take run output.
- `results/<stamp>.json` — per-pair counts and rates for each take
  (14 files, 505 pairs total).
