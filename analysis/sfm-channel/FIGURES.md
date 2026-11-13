# SfM figures — first scored runs

Trajectory overlays and the 080033 map from the SfM channel.

## Results table

`summary.csv` holds the same numbers. Loop is each trajectory's
own start-to-end distance. End is the distance between its end
pose and the reference end pose. All three are compared at the
same common frames.

| take | alignment | verdict | SfM loop (m) | ORB loop (m) | channel 2 loop (m) | SfM end (m) | channel 2 end (m) | SfM rotation (deg) | ORB rotation (deg) | channel 2 rotation (deg) | SfM ATE (m) | SfM valid frames |
| 080033 | prefix | PASS | 0.032 | 0.061 | 0.836 | 0.072 | 0.817 | 11.3 | 13.0 | 67.6 | 0.040 | 76 of 89 |
| 075947 | all-frame fit only | NOT SCORED | 0.161 | 0.188 | 0.580 | 0.004 | 0.633 | 6.7 | 8.9 | 14.8 | 0.015 | 35 of 47 |
| 155419 | prefix | PASS (thin) | 0.076 | 0.115 | 0.108 | 0.044 | 0.080 | 18.4 | 19.1 | 19.6 | 0.024 | 10 of 11 |

075947 has no prefix alignment, so its row is the all-frame fit
only and it is not scored. On 155419 the ORB loop is 0.135 m over
the full ORB trajectory and 0.115 m on the common frames used here.

## Trajectory overlays

One PNG per take. Top-down plot, same common frames, equal
aspect, y-axis inverted to match the orthomosaic image
orientation. SfM blue, ORB reference green, channel-2 VO red.
Green dot = start (all three start at the same first frame).
X in each trajectory's colour = its end. Legend carries loop
separations; the SfM and channel-2 entries also carry endpoint
discrepancy against ORB. Plotted loop separations match the
scored run files.

- `traj-080033.png` — SfM prefix alignment, loop 0.032 m,
  end 0.072 m. ORB loop 0.061 m. Channel 2 loop 0.836 m,
  end 0.817 m. Verdict: PASS.
- `traj-075947.png` — SfM ALL-FRAME fit only, loop 0.161 m,
  end 0.004 m. ORB loop 0.188 m. Channel 2 loop 0.580 m,
  end 0.633 m. Verdict: NOT SCORED. No prefix alignment exists.
- `traj-155419.png` — SfM prefix alignment, loop 0.076 m,
  end 0.044 m. ORB loop 0.115 m on the common frames (0.135 m
  over the full ORB trajectory). Channel 2 loop 0.108 m,
  end 0.080 m. Verdict: PASS (thin evidence).

## Orthomosaic map of 080033

`sfm-080033_poses_aligned.json` — the 89 SfM absolute poses
with the prefix alignment applied (scale 0.0119, anchored
rotation/translation from `evaluate_sfm.py`), metric, in the
ORB reference frame. SfM poses are unit-scale; feeding them
raw would splat every frame from nearly one spot.

Rendered with untouched `render_v2.py` (same recipe as the ORB
map: splat depth capped at 3 m, only frames within 35 deg of
straight down, last-frame-wins blend):
`python3 render_v2.py --poses sfm-080033_poses_aligned.json
--bag <080033>.db3 --out map-sfm-080033 --traj`

- `map-sfm-080033_v2_lastwins.png` — the headline map, same
  variant as the ORB deliverable (`map_orbslam3_080033.png`
  is byte-identical to the ORB last-wins render).
- `map-sfm-080033_v2_avg.png`, `map-sfm-080033_v2_avg_fill.png`,
  `map-sfm-080033_v2_traj.png` — average, filled average, and
  trajectory-overlay variants.
- `map-sfm-080033_v2.json` — extent 6.60 x 4.42 m, coverage
  48.4%, 423 of 441 frames splatted. ORB map for comparison:
  6.67 x 4.44 m, 48.8%, 419 of 441.

The SfM map is coherent grass with the same footprint and
texture as the ORB map, including the same diagonal streaking
the ORB map itself has. The trajectory in the overlay variant
is a ~12 x 10 px squiggle because the camera path is only
about 0.1 m across; that is the same situation as the ORB
overlay, not a defect.

Minor cosmetic issue: on `traj-080033.png` the note box at the
top right slightly overlaps the axes top border. It hides no
data.
