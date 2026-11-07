# Hartley normalization in stitch_map.py

## What the Nov 2 method was

Source: `analysis-drafts/normalization-test/test_normalization.py` and
`test_chain_norm.py` (Nov 2). The same formulas already appear in
`dataset-tools/stitch-map/stable_homography.py`, which was built on the
Nov 2 result. The method is explicit Hartley normalization around the
`cv2.findHomography` call:

- For each point set, build the normalization matrix `T` that moves the
  centroid to the origin and scales so the mean distance from the origin
  is sqrt(2): `s = sqrt(2) / mean_distance`, `T = [[s, 0, -s*mx],
  [0, s, -s*my], [0, 0, 1]]`.
- Fit the homography on the normalized points, then denormalize:
  `H = T_dst^-1 @ findHomography(T_src src, T_dst dst) @ T_src`.
- RANSAC threshold stays 3.0 px, Lowe ratio 0.75, SIFT with 10000
  features — all unchanged.

Re-running the original `test_normalization.py` on Nov 6 confirms the
Nov 2 finding: explicit normalization changes H slightly, so OpenCV does
not already normalize internally. Frobenius norm of
`H_direct - H_normalized` (after scaling so H[2,2] = 1) is max 1.47,
mean 0.43 on the first 11 pairs of 173807, and max 0.36, mean 0.09 on
the first 11 pairs of 080033. Inlier counts are not identical between
the two modes: medians 2772 direct vs 2782 normalized on 173807, and
3079 vs 3081 on 080033 (first 12 frames each).

## What changed in stitch_map.py

File: `dataset-tools/stitch-map/stitch_map.py`. **Update Nov 7 (after
review): normalized fitting is now the default** — the flag default
was flipped, and `--direct` was added to restore direct fitting.
Verified on the control take 102631c: no flag gives 676x502 with
`"normalize": true`, and `--direct` gives 672x507 with
`"normalize": false`, the original direct baseline exactly. Nothing
else about existing behaviour changes.

- `hartley_transform(pts)` and `apply_transform(T, pts)` — the Nov 2
  formulas, ported exactly.
- `fit_homography(src, dst, normalize=False)` — one place where the
  RANSAC homography is fitted. With `normalize=True` it normalizes
  both point sets, fits, and denormalizes as above. Direct fitting is
  the unchanged `cv2.findHomography` call.
- `pair_homography(..., normalize=False)` passes the flag through.
- `stitch(..., normalize=False)` passes it through to every pair
  fit: neighbour pairs in chain mode, loop-check pairs, and all pairs
  in tree mode. The run stats JSON now records `"normalize": true/false`.
- CLI: `--normalize` flag. Usage: `stitch_map.py --bag <take>.db3
  --out map.png --normalize`.
- Tree-mode checkpoint safety: the pairs-checkpoint metadata now
  includes the normalize setting, so a checkpoint written by a direct
  run cannot be reused by a normalized run (or the reverse) — the
  cached pair homographies differ between modes.
- The loop-check and tree code paths take the flag but were not
  exercised in verification; only chain mode was run.

No other behaviour changed. No upload was made to GitHub.

## Verification numbers

All runs use stride 5, SIFT, chain placement, default Lowe ratio 0.75.
Bags were resolved by exact filename stamp. The 102631c and 080033
numbers come from full end-to-end `stitch_map.py` runs. The 173807
numbers come from chaining through the wired-in `pair_homography`
(see the note below the table).

| Take | Mode | Canvas (px) | Pairs skipped | Median inliers | Nov 2 expected canvas |
|---|---|---|---|---|---|
| 20260926_102631c | direct | 672 x 507 | 0 | 1692 | 672 x 507, 0 skipped, median 1692 |
| 20260926_102631c | normalized | 676 x 502 | 0 | 1694 | — (not tested Nov 2) |
| 20261007_080033 | direct | 2351 x 2101 | 1 | 2352 | 2307 x 2047 |
| 20261007_080033 | normalized | 2171 x 1769 | 0 | 2326 | 2170 x 1767 |
| 20261026_173807 | direct | 57505 x 56539 | 0 | 732 | 57504 x 56538 |
| 20261026_173807 | normalized | 20373 x 24820 | 0 | 896 | 20372 x 24819 |

Direct mode reproduces the 102631c baseline exactly (672x507, 0
skipped, median 1692). Normalized mode reproduces the Nov 2 numbers
to within 1–2 px on 173807 in both modes and on 080033 in normalized
mode — that is RANSAC draw noise, not a port difference.

### Why 080033 direct differs (2351x2101 vs 2307x2047)

One borderline pair explains it. Pair 71 (chosen frames 70→71) has
only 20 good matches. Direct RANSAC finds 0 inliers on it, so
`stitch_map.py` skips the pair (identity step, counted in
`pairs_skipped`). Normalized RANSAC finds 20 inliers on the same pair
and chains it. The Nov 2 chain test had no inlier floor: it chained
whatever H came back as long as there were at least 20 good matches.
Replaying the Nov 2 chaining rule on the same pairs with the wired-in
code gives 2309x2048 — the Nov 2 number within 2x1 px. So the port is
faithful; the gap is a pre-existing difference in weak-pair handling
between `stitch_map.py` (skip) and the Nov 2 test script (chain
everything), not a normalization bug. Note the irony: on this take the
skipped pair makes the direct canvas larger, not smaller, because
skipping leaves the drift uncorrected while chaining the weak H
happened to fold the chain back.

### Why 173807 was verified without a full render

This VM has 7 GB of RAM. The renderer accumulates float64, so a
full-res render of the 173807 direct canvas (57k x 56k px) or the
normalized canvas (20k x 25k px) does not fit in memory. The canvas
numbers above were computed by chaining the pair homographies through
`stitch_map.pair_homography` itself (script
`verify_173807_chain.py` in this folder), with the same frame
selection, features, and matcher as the full script, and the visual
check used downscaled preview renders of those same chains (long side
capped at 2000 px). The 080033 and 102631c numbers are full renders.

## Visual judgement

Every panorama below was opened and looked at, not just measured.

- **102631c, direct and normalized**: identical clean grass frames,
  as expected — the camera barely moves. Normalization changes nothing
  visible here.
- **080033, direct** (`verify_080033_direct.png`, 2351x2101): a long
  diagonal sheared strip of grass with a notch near the top-left edge
  and the right end tapering to a point. Not a usable map.
- **080033, normalized** (`verify_080033_norm.png`, 2171x1769): the
  same diagonal-strip failure, but visibly fuller and wider, and the
  right end is less pinched. Less stretched than direct, but the same
  failure mode — still not a usable map.
- **173807, direct preview** (`verify_173807_direct_preview.png`):
  total divergence. The chain explodes into two hair-thin diagonal
  streaks meeting at a point on a black void. Nothing map-like remains.
- **173807, normalized preview** (`verify_173807_norm_preview.png`):
  still totally diverged — a broad pink radial fan of streaks
  radiating from a point, with no recognizable grass texture. It fills
  more of its canvas than the direct version (matching the much
  smaller canvas), but it is still not a map.

Verdict: normalization strictly helps — smaller canvas on both moving
takes, and it rescues the borderline pair that direct RANSAC fails on
— but it does not cure chaining divergence. The Nov 2 verdict stands.

## Recommendation for the default: normalized

Recommend making `--normalize` the default (or equivalently keeping
the flag and switching the default to on), for these reasons:

1. It never hurts in the measurements: smaller canvas on both moving
   takes (080033: 2171x1769 vs 2351x2101; 173807: 20373x24820 vs
   57505x56539, about 2.8x smaller in each dimension), and the
   still-camera baseline is undisturbed (676x502 vs 672x507, median
   inliers 1694 vs 1692).
2. It rescues borderline pairs: pair 71 on 080033 goes from 0 to 20
   inliers, and median inliers on 173807 rise from 732 to 896.
   Fewer skipped pairs means fewer identity steps in the chain.
3. It costs nothing: one extra small matrix computation per pair, same
   RANSAC call, same runtime to within measurement noise.
4. It is the textbook-correct way to fit a homography (Hartley
   normalization), so a reader of the code expects it.

Against: the change is small and the benefit, while consistent, does
not fix the real problem — chaining divergence on long takes still
needs loop closure or tree placement, which is separate work. If the
team prefers to keep the established direct baseline byte-identical
for past comparisons, keep direct as the default and use
`--normalize` explicitly; the flag is there either way. The parent
decides.

## Files in this folder

- `SUMMARY.md` — this file.
- `PROGRESS.md` — run checklist.
- `verify_102631c_direct.png/.json`, `verify_102631c_norm.png/.json` —
  full renders and stats, still-camera baseline.
- `verify_080033_direct.png/.json`, `verify_080033_norm.png/.json` —
  full renders and stats, stride 5.
- `verify_173807_chain.py` — chain-only canvas check plus downscaled
  preview renders for 173807 (full render does not fit in memory).
- `verify_173807_direct_preview.png`,
  `verify_173807_norm_preview.png` — downscaled previews, long side
  2000 px.
- `diag_080033.py` — per-pair good-match/inlier table and the
  three-way canvas comparison that pins the 080033 direct gap on
  pair 71.

Note: the staging scripts in this folder use home-relative paths for
the bags; only `stitch_map.py` itself is meant for upload, and it
contains no local paths, no attributed quotes, and no bookkeeping
labels (checked with grep).
