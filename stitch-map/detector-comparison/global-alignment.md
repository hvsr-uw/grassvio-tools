# Global alignment for stitch maps

Tree placement still positions each frame along one path, so error can
pile up along that path. Global alignment instead positions every frame
by all of its matches at once. It starts from the tree placement, then
repeats this step: for each frame, every pair it belongs to predicts
where its corners should land, those predictions are averaged, and a
new homography is fit from the frame's own corners to the averaged
corners. The script is `global_align.py`. It reads a pairs file from a
tree run, so no matching is redone, and it reports the mean corner
distance over all pair constraints before and after, in pixels.

## Verified results

Phone photos (SIFT, stride 1, ratio 0.85): only 6 pairs pass the inlier
threshold, and the tree already uses all 6, so the starting residual is
essentially 0 pixels and global alignment changes nothing. Canvas
identical at 3523 by 2771 pixels. There are no redundant constraints
here, so there is nothing to average. The phone map's remaining
roughness is limited by the data, not by the placement method.

September take (SIFT, stride 5): 300 pairs used, residual goes from
0.065 to 0.035 pixels, canvas identical at 672 by 507 pixels. The tree
was already nearly consistent here.

October take (SIFT, stride 3): 10345 pairs used, residual goes from
5.97 to 4.60 pixels, canvas essentially identical at 2325 by 2050
pixels against the tree's 2326 by 2038, and viewed the map shows the
same far end stretch. The residual floor of about 4.6 pixels across
10345 constraints means the homographies themselves disagree: no
arrangement of flat-plane transforms can satisfy them all at once,
because the take is down-facing close-up 3D grass where the flat plane
model does not hold. Placement iteration stops here. Fixing the stretch
needs a different model, such as depth-aware alignment using the depth
stream, or full 3D SLAM, which is beyond the stitch script.

Usage:

    python3 global_align.py --bag <take>.db3 --pairs <pairs.json> \
        --stride 3 --out map.png
