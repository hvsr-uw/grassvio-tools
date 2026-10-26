# Loop closure in the stitch script

The stitch script now has a `--loop-check` flag. Without it, the script
only matches neighbouring frames, so when the camera walks in a loop and
comes back to the same place, the map cannot use that overlap to fix the
drift. With the flag on, each frame is also matched against earlier frames
at least 5 chosen frames back. A strong loop match (at least 50 inliers)
re-anchors the frame: its transform comes from the loop match instead of
the drifted chain, and later frames chain from there. The 5 frame gap
matters because closer frames still overlap from normal forward motion,
and matching them is not a loop closure. Neighbour pair stats keep their
meaning; loop matches are reported separately in the stats JSON as
`loop_matches`, with the frame, the matched frame, the inlier count, and
whether the frame was re-anchored.

## Verified results

No regression: without the flag, the refactored script reproduces the
September take baseline exactly (25 frames used, 0 skipped, median 1692
inliers, canvas 672 by 507 pixels).

Phone photos (SIFT, stride 1, loop check on): exactly one loop match is
found, the last photo against the first one with 96 inliers, and it is
re-anchored. The canvas goes from 7123 by 2654 pixels down to 3766 by
3148 pixels, because the last photo is no longer dragged along the
drifted chain. The top of the map stays coherent and the runaway
horizontal smear is gone. The map is still not a clean panorama, because
the phone photos are handheld with big rotations over a 3D lawn, but the
loop closure is a real improvement on the worst part.

September take (SIFT, stride 5, loop check on): 20 distant matches are
found, all with 1490 to 1875 inliers, and the canvas comes out identical
at 672 by 507 pixels. The walk stays in a small area, so nearly every
frame overlaps frames from much earlier, and the chain was already
consistent there. Loop closure changes nothing on this take, which is
the expected result when there is no drift to correct.

October take (SIFT, stride 3, loop check on): 142 loop matches are found
with a median of 2167 inliers, and 141 frames are re-anchored. The
neighbour stats are identical to the baseline (147 frames used, 1
skipped, median 2251 inliers), but the canvas comes out slightly larger
at 2288 by 1991 pixels instead of 2204 by 1891, and the far end stretch
is still visible in the map. Same story as the September take: the chain
was already consistent, so loop closure is neutral here. The phone
photos remain the one case where it genuinely helps.

Usage:

    python3 stitch_map.py --frames <folder> --stride 1 --detector sift \
        --loop-check --out map.png
