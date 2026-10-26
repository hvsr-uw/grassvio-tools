# Tree placement in the stitch script

The chain version of the script places each frame from its neighbour
only. When a neighbour pair fails, that frame is placed with the previous
frame's transform, which is the worst option: it is not a measurement at
all. The phone photos showed this clearly. Photos 2 and 4 have no usable
match to any earlier photo (best earlier matches of 4 and 11 inliers),
so no backwards-looking fix, including loop re-anchoring, can place
them. But photo 2 matches photo 3 with 102 inliers, so a frame that
fails against its predecessor can still be placed from its successor.

The `--place tree` option does exactly that. It matches every frame pair,
then builds the strongest spanning tree over all pairs, rooted at the
first frame, always taking the strongest edge that reaches a new frame.
Each frame is placed along its strongest path back to the first frame.
`--lowe-ratio` tunes the matcher (default 0.75). Tree mode saves every
pair result to `<out>_pairs.json` as it goes, and resumes from that file
if the run is interrupted, instead of starting over. That checkpointing
was added after runtime restarts killed the October tree run three
times; a resumed phone run took 14 seconds against 61 seconds fresh,
with identical results.

## Verified results

Phone photos (SIFT, stride 1): tree at ratio 0.85 places all 7 photos
with real matches, 0 skipped, along the path 0 to 6 to 5 to 4 to 3 to 2
to 1, canvas 3523 by 2771 pixels. Viewed: better than the loop version,
the photos tile more sensibly, but still not fully clean at the bottom,
because two of the links holding it together are weak (23 and 24
inliers) and the lawn is 3D with big rotations between shots. Tree at
ratio 0.75 leaves photos 2 and 3 unplaced (2 skipped). Tree at ratio
0.9 is worse: only 2 edges survive and 4 photos go unplaced, because
the looser ratio floods RANSAC with false matches on repetitive grass.
0.85 is the sweet spot for this set.

September take (SIFT, stride 5, tree): 0 skipped, canvas identical to
the chain map at 672 by 507 pixels, viewed clean. Tree placement is
safe on a good take.

October take (SIFT, stride 3, tree): 147 frames used, 0 skipped (the
chain skipped 1), tree edges have a median of 2640 inliers against the
chain median of 2251, but the canvas comes out larger at 2326 by 2038
pixels instead of 2204 by 1891, and viewed the map shows the same far
end stretch. Tree placement fixes the skipped frame but does not fix
the stretch. The stretch is not a placement path problem: it is drift
piling up in the homographies themselves on a down-facing close-up
grass take. The next step for it is global alignment, positioning every
frame by all of its matches at once instead of one path.

Usage:

    python3 stitch_map.py --frames <folder> --stride 1 --detector sift \
        --place tree --lowe-ratio 0.85 --out map.png
