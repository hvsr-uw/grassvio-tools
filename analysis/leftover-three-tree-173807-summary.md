# Tree placement on take 173807 — summary

Take 173807 is a long Oct 26 take (979 frames, 196 chosen frames at
stride 5). The question was whether placing frames along a spanning
tree over all pairwise matches would do better than the neighbour
chain, which blows up to a 57,504 x 56,538 px canvas. The answer is
no. On this take the tree is the chain, exactly.

## Final numbers

- Pairs completed: 19,110 of 19,110. The checkpoint now holds every
  pair (SIFT, Lowe 0.75, RANSAC 3.0 px, 20-inlier minimum).
- Usable edges (20 or more inliers): 2,252. Failed or weak pairs:
  16,858. Most far-apart pairs have no overlap at all.
- Tree edges used: 195. Non-neighbour edges used: 0. Unplaced
  frames: 0.
- Tree edge inliers: median 732, minimum 183, maximum 3,555.
- Tree canvas: 57,504 x 56,538 px (x from -23,390 to 34,114;
  y from -32,968 to 23,569).

## Comparison table

| Placement | Canvas (px) | Median inliers | Skipped / unplaced |
| --- | --- | --- | --- |
| Direct chain | 57,504 x 56,538 | 732 | 0 skipped |
| Normalized chain | 20,372 x 24,819 | 896 | 0 skipped |
| Tree (this run) | 57,504 x 56,538 | 732 | 0 unplaced |

The direct-chain row is recomputed here from the checkpoint
neighbour pairs. It matches the earlier chain diagnostic to the
pixel (57,503.96 x 56,537.68). The normalized-chain row is the
Nov 2 measurement; an independent re-run gave 20,373 x 24,820.

## Visual judgement

I opened all three panoramas and judged them by eye.

- Tree panorama (`panorama_tree.png`, 4000 x 3933 px): a thin
  diagonal streak across a mostly black canvas, with one small
  recognizable patch near the center-left. The frames are sheared
  into a line. Only 7.5 percent of the pixels are non-black. This
  is not a map.
- Direct-chain panorama (`panorama_chain_direct.png`): byte-identical
  to the tree panorama (same md5, max pixel difference 0). There is
  nothing to distinguish them. An independent direct-chain render
  from the normalization work shows the same streak and the same
  composition, which confirms the render is correct and not a bug
  in this run's code.
- Normalized-chain preview (from the normalization work, 2001 x 1642
  px): a smaller canvas, and it fills more of it (74.7 percent
  non-black), but the content is a pink fan that sweeps out from a
  point at the lower-left. It is still a smear, not a map.

The tree is not less stretched or less diverged than the chain
versions. It is identical to the direct chain, and the normalized
chain is smaller but still diverged.

## Why the tree did not help

The tree had no shortcut to use. There are 2,057 usable
non-neighbour edges, but their median is only 66 inliers, against
732 for neighbour edges. For every one of the 195 frames, the
single strongest edge touching it is a neighbour edge. Prim's
algorithm grows the tree by always taking the strongest edge to a
new frame, so it takes the neighbour edge every time. The result
is the chain with a different name.

This is what the loop check predicted. All 25 end-vs-start pairs
have zero inliers, so 173807 has no loop closures. A spanning tree
can only beat a chain when a frame matches some far-apart frame
better than its neighbour does. On a take that walks away and
never returns, that never happens.

## How the resume worked

The driver is `dataset-tools/stitch-map/stitch_map.py --place tree`.
It resumes from a pairs checkpoint, but only when the checkpoint
meta matches exactly. The first relaunch silently discarded the
4,400-pair checkpoint: the Oct 27 meta has four keys, and the
current script builds five (a `--normalize` option was added
since, default false). That run started from scratch and overwrote
the file with 50 pairs. No work was lost. A backup copy taken
before the run held the 4,400 pairs. The checkpoint was restored
from that backup with the truthful added key `"normalize": false`
(the Oct 27 run used direct fitting), and the resume then worked.

The remaining 14,460 pairs were split into two shards of 7,230
(`shard_A.txt`, `shard_B.txt`) and matched by `match_shard.py`,
which imports the same `pair_homography` from `stitch_map.py`, so
the matching is identical to the original run. Each shard file
flushes every 25 pairs and resumes from its own file. The shards
were merged back with `merge_shards.py`, which refuses to write
unless the meta matches, no key overlaps, and the total is exactly
19,110.

Two things slowed the run down. First, the measured matching rate
is 0.65 to 1.0 pairs per second, not the estimated 2-hour pace;
the whole resume took about five hours of wall time
(20:34 UTC Nov 6 to 01:55 UTC Nov 7). Two workers together ran no
faster than one alone (combined 0.93 pairs per second against
1.03 solo), so the shards ran one after the other. Second, the VM
rebooted three times (about 21:25 UTC, 23:26 UTC, and 01:13 UTC).
Each reboot killed the running worker, and each time the worker
resumed from its shard file with at most 25 pairs lost.

The full-resolution panorama was not rendered through
`stitch_map.py`. Its renderer accumulates float64 over the whole
canvas, and 57,504 x 56,538 px does not fit in this machine's
7 GB of RAM. Both panoramas here were rendered by
`render_tree_vs_chain.py` with the same averaging warp, downscaled
to a 4000 px longest side (scale 0.06956), so the tree and the
chain are compared at the same scale.

## Verdict

The tree did not help on 173807. It produced the exact same
placement, the exact same canvas size, and the exact same image as
the direct chain. The take has no loop closures, and without one
there is nothing for a tree to exploit. The normalized chain
shrinks the canvas about threefold but still diverges. None of the
three placements yields a usable map of this take.

## Files in this directory

- `SUMMARY.md` — this file.
- `PROGRESS.md` — run log with the false start, the reboots, and
  the rate measurements.
- `stitch_173807_sift_tree_pairs_COMPLETE.json` — the completed
  19,110-pair checkpoint (copy; the original is at
  `new-takes-20261026/stitch_173807_sift_tree_pairs.json`).
- `checkpoint_backup_4400pairs.json` — the pre-resume checkpoint.
- `panorama_tree.png` — tree panorama, downscaled to 4000 x 3933.
- `panorama_chain_direct.png` — direct-chain panorama at the same
  scale (byte-identical to the tree panorama).
- `tree_vs_chain_stats.json` — canvas sizes, edge counts, inlier
  stats, render scale.
- `match_shard.py`, `merge_shards.py`, `render_tree_vs_chain.py` —
  the scripts used for this run.
- `shard_A.txt`, `shard_B.txt`, `shard_A.json`, `shard_B.json`,
  `worker_A.log`, `worker_B.log`, `resume_run.log` — shard inputs,
  outputs, and logs.
