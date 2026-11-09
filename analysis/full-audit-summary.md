# Full audit of the GrassVIO project work, November 9, 2026

Scope: CS 566 final project only. This audit covers the work staged in
analysis-drafts through November 7, 2026: the October 30 completion
summary, the loop-closure work, the reliability model prototype, the
November 6 takes, the three leftover tasks (normalization, the 173807
tree, ORB-SLAM3), and the 080033 pose-graph work. It also covers the
recording feature table, the stitch-all batch, the four-panel video,
and the trajectory overlay.

Method: every headline number below was re-checked against its source
file (trajectory files, poses JSONs, per-take JSONs, run stats) by
recomputing it from the file, not by trusting the summary text. Every
key image listed in the grades section was opened and judged by eye
in this audit. Where a number or a claim could not be verified from a
file on disk, that is stated plainly instead of filled in. Nothing in
this audit was uploaded anywhere, and no message was sent to any
teammate.

Two source problems found at the start, stated up front because they
affect the inventory. First, the October 30 minutes file lives at
analysis-drafts/team-chat-analysis-2026-10-30.md, not at the path
given in the audit brief. Second, that file covers the team chat from
October 25 to October 30 and does not contain a per-person assignment
list for the October 30 meeting. The per-person assignments used in
Section 1 below (Don on intensity, wind, and the reliability model;
Julius on drift and closed-loop testing; Harsh on challenges, dynamic
maps, running all recordings, labelling the dataset, and dynamic
panoramas; all on reliability models) come from the audit brief, not
from that minutes file, and could not be verified against any file
read here. The November 5 questions are in the same position: no file
read here contains them, so they are treated as brief-given items and
only the work answering them is judged from files.

## 1. Complete assignment inventory

Status words used here: done means a file on disk shows the work
finished; not done means no file on disk shows it; blocked names the
person or thing that has to move first; unverified means no file read
here settles it either way.

### Harsh

| Item | Status | Evidence |
|---|---|---|
| Run the stitch script on all recordings | Done | analysis-drafts/SUMMARY.md (Nov 1) and analysis-drafts/stitch-all/stitch-classification.md: chain SIFT on 40 usable takes, 17 worked, 6 sort-of-ok, 17 blow-ups. 23 panorama PNGs are present in analysis-drafts/stitch-all/panoramas, which matches 17 plus 6 exactly. |
| Label the dataset (recording feature table) | Done | analysis-drafts/recording-feature-table.md: 49 takes, 48 usable, 1 failed recording (172642, 0.17 s, 6 frames). Dated rows across the three tables total 146, which is 49 plus 49 plus 48, matching 49 takes in the main table and the IMU appendix and 48 in the wind appendix. |
| Dynamic panorama creation | Done | analysis-drafts/SUMMARY.md (Nov 1): four-panel video script and demo video for take 080033 in analysis-drafts/four-panel. The video file itself was not opened in this audit, so the moving picture is taken from the Nov 1 summary, not from this audit. |
| Trajectory overlay on the panorama | Done as a prototype | analysis-drafts/trajectory-overlay.md and the demo image in dataset-tools/stitch-map/demo_20261007_trajectory.png, opened in this audit: the magenta path and the start and end markers are clearly visible on the panorama. The base panorama underneath is the known stretched homography chain, so the overlay is a working figure on a weak base map. |
| Challenges (write-up portion) | Unverified | No file read here contains a challenges section or a status for it. |
| Dynamic map creation | Not done as a separate deliverable | No file read here delivers a dynamic map as its own item. The four-panel video and the orthomosaics cover parts of it, but neither is labelled as the dynamic map. |
| Loop detection through RGB-D visual odometry (Nov 5 question 1, assigned to Don in the brief; the loop-closure work itself was done by Harsh) | Done as a measurement, with a negative answer | analysis-drafts/loop-closure/SUMMARY.md: RGB-D visual odometry alone does not detect the loop precisely. On the genuine loop take 080033 it ends 0.836 m and 67.6 deg from its start, recomputed from analysis-drafts/rgbd/080033_poses.json. |
| Panorama with stable homography (Nov 5 question 3) | Done as a test, with a negative answer | analysis-drafts/loop-closure/SUMMARY.md: the homography chain is not stable on 080033 even with normalization and a loop anchor. Linear distribution of the loop correction made the canvas worse, 6766 by 7829 px against 2170 by 1767 px uncorrected. |
| November 6 takes: wind and illumination analysis | Done | analysis-drafts/new-takes-20261106/SUMMARY.md plus per-take JSONs in analysis-drafts/new-takes-20261106/checks. All 8 takes checked. No take shows a wind signal. |
| ORB-SLAM3 baseline and 080033 pose-graph work | Done | analysis-drafts/leftover-three/orbslam3/SUMMARY.md and analysis-drafts/pose-graph-080033/SUMMARY.md. See grades in Section 2. |
| Reliability model prototype (the All item from Oct 30) | Done as a prototype | analysis-drafts/reliability-models/SUMMARY.md and dataset-tools/reliability/reliability_hmm.py. See grade in Section 2. |
| Reply drafts to Julius (on loops) and to Jinming (on question 3 and the Nov 6 findings) | Unverified | The brief states two reply drafts exist and have not been sent as far as anyone knows. No draft file was found in the files read here (report-drafts holds other drafts only), so this audit cannot confirm the drafts exist, what they say, or whether they were sent. |

### Don

| Item | Status | Evidence |
|---|---|---|
| Intensity work | Not done as far as the files show | No file read here contains intensity results or an intensity analysis. |
| Wind work, including how to tell wind-blown leaves apart | Not done as far as the files show | No file read here contains a wind-separation result. The wind evidence that exists (feature-table residuals, the 173701 note, the Nov 6 batch) is Harsh's measurement work, and none of it separates wind from parallax. |
| Reliability model / empirical reliability score | Not done as far as the files show | The reliability-models summary states Don is separately building an empirical reliability score. No file for that score was found in the sources read here. |
| Simulated-data validation (Oct 25) | Done | analysis-drafts/team-chat-analysis-2026-10-30.md records the Oct 25 simulated validation: SIFT follows the true motion, ORB deviates by about 1 degree and up to 1 cm, HOG returns no pose on the combined motion. This is a minutes record, not a file re-verified in this audit. |
| Still-camera clip for the reliability score (camera still, grass moving) | Blocked on Jinming recording it | The brief states Don asked for this on Nov 4 and Jinming said he would try to record one. No file read here shows such a clip arriving. The Nov 6 batch does not contain it: its four still takes are the cleanest in the dataset by the residual test, and their frame-level events are camera shake, a moving shadow, and illumination changes, not moving grass. |

### Julius

| Item | Status | Evidence |
|---|---|---|
| Eliminate drift: find a closed loop and test it | Not done | No file read here shows a drift-elimination test or a closed-loop result from Julius. The loop testing that exists (loop-closure summary) is Harsh's measurement work. |
| Run on recordings for drifting issues, noise issues, dynamic odometry creation | Not done as far as the files show | No file read here contains Julius's drift, noise, or dynamic-odometry results. |
| Loop detection through IMU alone without drifting (Nov 5 question 2) | Not done | The loop-closure summary states this was not addressed there, and no other file read here addresses it. |
| Pushed incomplete code (Nov 5) and update before or on meetings | Blocked on repository access | analysis-drafts/julius-code-review.md (Nov 6): the team repository is not reachable with the stored credential (both the repository and branches endpoints return 404), so the code could not be read or reviewed. What the code does, what is incomplete, and whether it runs are all unknown from here. Access has to be granted by Julius or by Don as repository owner, or the code has to be pushed somewhere readable. |
| Meaning of 174702 and 174818 are loop | Blocked on one question to Julius | Both takes were measured two ways in analysis-drafts/loop-closure/SUMMARY.md and neither measurement supports the camera returning to its start: SIFT end-vs-start max 0 inliers on both, and RGB-D start-end errors of 5.100 m and 101.3 deg for 174702 and 4.331 m and 66.2 deg for 174818, recomputed from the poses JSONs. Whether Julius meant a mid-take crossing, a recording intent, or a rotation needs one question to him, not another measurement. |

### Jinming

| Item | Status | Evidence |
|---|---|---|
| Upload loop and wind recordings (Oct 26 set) | Done | The Oct 26 set is in the feature table: 21 takes, measured. |
| Upload more rotation recordings (Oct 30 set) | Done | The Oct 30 set is in the feature table: 5 takes, measured. |
| Upload wind-blowing and illumination-change recordings (Nov 6 set) | Done as an upload, with a finding attached | 8 takes, 084542 to 085611, all checked in analysis-drafts/new-takes-20261106/SUMMARY.md. The batch carries illumination variation, not wind: no take shows a wind signal by any test tried. |
| Still-camera windy clip for Don | Not done | See the Don row above. Blocked on Jinming recording it. |
| Synthesized data (simulation, abrupt intensity change from clouds) | Not done as far as the files show | No file read here contains synthesized data or a simulation run for this. This was a suggestion from Jinming, not a delivered item. |
| Feature-detection frequency and the optical-flow reliability holdout (Nov 5 question 4) | Not done | No file read here answers at what frequency feature detection is performed or tests holding frames back for reliability checking. The reliability-models summary names optical-flow tracking as the clear next step, which is the same open point from the other side. |
| Presentation slot scheduling | Unverified | The Oct 30 minutes record it as pending the Oct 30 meeting. No file read here confirms a slot was scheduled afterwards. |
| Walkthrough meeting | Not done as far as anyone knows | No file read here shows a walkthrough meeting scheduled or held. Owner and date are both unsettled. |

### All (team)

| Item | Status | Evidence |
|---|---|---|
| Reliability models: Bayesian model, Markov chain, and similar | Done as a prototype | analysis-drafts/reliability-models/SUMMARY.md. See grade in Section 2. Don's empirical score, the other half of this item, is not done as far as the files show. |

### Off-limits item, listed only

| Item | Status | Evidence |
|---|---|---|
| Recommendation-letter ask to Jinming | Blocked on Harsh | Per the audit brief this needs Harsh's inputs and is off-limits for this audit. No work was done on it here. A draft brag sheet exists at report-drafts/rec-letter-brag-sheet-jinming.md; it was not read or judged in this audit. |

## 2. Deliverable grades

Grades: GOOD, NOT GOOD BUT FIXABLE, NOT GOOD AND PROVEN UNFIXABLE,
BLOCKED (with who or what is needed). Proven unfixable is used only
where a clean null was measured, not where something was merely not
tried. Every image grade below comes from opening the image in this
audit.

### GOOD

| Deliverable | Evidence |
|---|---|
| Recording feature table, 49 takes | 49 takes, 48 usable, methods and verdict bands written down, loop and wind measured on every usable take against the 102631c control (median 0.185 px, p90 0.458 px, recomputed in the table). Genuine loops: 080033 (all 25 end-vs-start pairs at 1444 to 1882 inliers), 075947 (133 to 195), 155419 (max 728). Weak loops 172532 (max 45), 172828 (max 37), 172859 (max 51). No take anywhere is labelled as wind; 13 takes carry the non-rigid-signal label with the parallax caveat stated. |
| Stitch-all batch on 40 usable takes | 17 worked, 6 sort-of-ok, 17 blow-ups, as stated in the classification file. 23 panorama PNGs are present, matching 17 plus 6. The 102631c run reproduces the earlier baseline exactly (672 by 507 px, 0 skipped, median 1692 inliers). Two panoramas opened here (075947, 174745) are clean and coherent and match their worked calls; 075947 is crisp grass throughout, 174745 shows the lawn and the manhole cover clearly. |
| November 6 batch analysis and stitches | All 8 takes stitched, 0 skipped pairs, median inliers 1900 to 3777, from analysis-drafts/new-takes-20261106/stitch/results.jsonl. Tally 7 worked, 1 sort-of-ok, 0 blow-ups. The sort-of-ok call is a visual judgement in the summary, not a field in the results file, where all 8 takes carry the same stitched status. Opened here: 085301 is a coherent leaf-litter fan with leaves lining up; 085224 is a coherent fan with mild stretch at the top; 085422 is coherent with the moving shadow visible as a diagonal darker band; 085611 is a clean single view of grass and leaves; 084542 is a clean single view. 085453 is the one sort-of-ok: a coherent main patch with the far end stretched into a thin diagonal streak. |
| ORB-SLAM3 RGB-D-only baseline (3 takes) | End error recomputed from the trajectory files: 080033 0.061 m and 13.0 deg, 075947 0.188 m and 8.8 deg, 085611 0.0001 m and 0.01 deg (the rotation recomputes to 0.0 deg at the precision used here; the 0.01 deg in the summary is a rounding difference, not an error). Against the visual odometry: 0.836 m and 67.6 deg, 0.580 m and 14.8 deg, 0.238 m and 86.3 deg on the same takes, all recomputed from the poses JSONs. The rotation check against the gyro also holds: ORB-SLAM3 totals 183.8, 56.4, and 4.1 deg against gyro totals of 190.3, 58.6, and 4.4 deg, while the visual odometry totals 165.9, 42.6, and 86.6 deg. Overlay plots opened here confirm the picture: on 085611 the ORB-SLAM3 trajectory is a dot at the origin while the visual odometry wanders, and on 080033 the ORB-SLAM3 path is a small closed loop while the visual odometry makes a large excursion. |
| ORB-SLAM3-pose orthomosaic of 080033 | analysis-drafts/pose-graph-080033/orbslam3-pose/map_orbslam3_080033.png, opened here: a coherent grass patch, 6.67 by 4.44 m, 48.8 percent coverage, 419 of 441 frames splatted, from the render JSON. No stretch or smear across the main patch, the far right end keeps texture with a clean edge, and the comparison figure shows the start and end markers 6 px apart against 83 px apart for the visual-odometry map. This is the best map of 080033 in the project and the deliverable to show. |
| SE(3) pose-graph refinement of the ORB-SLAM3 poses | A clean null, correctly closed. Loop end error moves from 0.0608 m and 13.00 deg to 0.0578 m and 12.64 deg, from analysis-drafts/pose-graph-080033/se3-refine/refinement_results.json. The refined map, opened here, is indistinguishable by eye from the unrefined map. The summary's reading holds: most of the remaining error is genuine camera displacement, not drift, with only about 0.012 m and 0.6 deg of genuine inconsistency for the graph to fix. Nothing further to do here. |
| Loop-closure RGB-D measurements (Tasks 1 to 3) | All start-end errors recomputed from the poses JSONs and matching the summary: 155419 0.108 m and 19.6 deg, 075947 0.580 m and 14.8 deg, 080033 0.836 m and 67.6 deg, 173807 4.926 m and 149.6 deg, 172532 6.102 m and 101.0 deg, 172828 5.394 m and 67.3 deg, 172859 0.838 m and 29.4 deg, 174702 5.100 m and 101.3 deg, 174818 4.331 m and 66.2 deg. The 174702 and 174818 orthomosaics, opened here, support the numbers: 174702 is a smeared fan, 174818 is two separate patches. |
| Hartley normalization port in the stitch script | The code change is verified: normalized fitting is now the default in dataset-tools/stitch-map/stitch_map.py, with a direct-fitting flag to restore the old behaviour. The control take reproduces the baseline (direct 672 by 507 px, median 1692 inliers; normalized 676 by 502 px, median 1694). Normalization shrinks the canvas on both moving takes (080033: 2171 by 1769 px normalized against 2351 by 2101 px direct; 173807: 20373 by 24820 px normalized against 57504 by 56538 px direct). See the map outcome in the fixable row below. |
| 085611 brightness analysis | Brightness range 23.31, from the per-take JSON (maximum 113.57 minus minimum 90.26). The brightness plot, opened here, shows the repeated oscillation on a fixed view, and the stitch of the same take is a clean single view, so the light change is genuine and the scene is fixed. |
| Reliability plots (173701 and 174745) | The 173701 histogram, opened here, is bimodal with large peaks near 0 and near 1, matching the summary. The 174745 histogram, opened here, is a single peak near 1. The 173701 curve, opened here, starts near 0.95 and falls to about 0.3 mid-take. One correction to the summary wording: the curve does not stay low after falling; it oscillates between about 0.22 and 0.56 for the rest of the take, with a partial recovery toward 0.5 at the end. |

### NOT GOOD BUT FIXABLE

| Deliverable | Evidence and fix |
|---|---|
| Reliability HMM prototype as a reliability model | The take-level separation is real and verified from the tracks JSONs: 173701 has 47.2 percent of tracks above 0.8 and 32.2 percent below 0.2, against 97.8 percent above 0.8 and 0.2 percent below on 174745 and 97.5 percent above 0.8 and 0.2 percent below on 080033. Median track length is 1 on all three takes, so almost no track accumulates more than one residual and the Markov temporal part contributes little; the separation comes almost entirely from the residual likelihood, and a plain median-residual threshold separates the takes too (0.926, 0.344, 0.921). The transition matrix and the unreliable sigma of 3.0 px are set by hand, not fitted. Fix: optical-flow tracking instead of per-frame re-detection so tracks last, then fit the parameters. Wind versus parallax is still unseparated. |
| RGB-D visual-odometry orthomosaics (all takes except the ORB-SLAM3-pose 080033 map) | Opened here: 080033 visual-odometry map is a coherent wedge but vertically smeared with a hard seam across the middle; 075947 is a small heavily smeared patch with almost no texture; 173807 is a vertically smeared blotch; 155419 is fragmented and sparse; 172532 and 172828 are each two disconnected smeared wedges; 174702 is a smeared fan; 174818 is two separate patches. The Nov 6 still takes are worse, not better: the camera never moves yet the chained poses drift 80 to 118 deg, and the maps are smeared wedges and striped smears (085611 opened here: a striped wedge). Fix: render the remaining takes from ORB-SLAM3 RGB-D-only poses the way 080033 was rendered, instead of from the visual-odometry poses. The still takes need no chaining at all; their homography panoramas are the maps to show. |
| Homography chain panoramas on moving takes | The stitch-all batch proves the pattern across 17 blow-ups of 40 takes: pairwise matching succeeds but the chained transforms diverge under sustained rotation. The normalized 080033 chain, opened here, is a sheared diagonal strip with the far end smeared; the normalized 173807 preview, opened here, is a pink radial fan with no recognizable grass. Fixable only by abandoning the chain as the placement method for moving takes: the ORB-SLAM3-pose route already fixes 080033, and the same route is the fix for the other moving takes. |
| Normalized chain maps (080033, 173807) | Normalization helps (smaller canvas, borderline pair 71 on 080033 goes from 0 to 20 inliers) but does not cure divergence. Superseded by the ORB-SLAM3-pose maps; no further work on the normalized chain is needed for 080033. |
| ORB-SLAM3 RGB-D-inertial mode | Never completes initialization on these takes: 11 of 440 poses on 080033 with datasheet noise, 82 of 440 with the looser noise values, 69 and 27 of 235 on 075947, 0 of 386 on 085611 with a segfault on saving the empty trajectory. Two noise settings were tried, so further noise tuning is exhausted. Fixable only by a different route: takes with faster, more varied motion, or a different initialization path such as a vision-first warm start. |
| Loop detection by RGB-D visual odometry alone | 0.836 m and 67.6 deg of error on a true visual loop (080033). Fix: the visual match plus a pose-graph step, or the ORB-SLAM3 trajectories, which already close the loop to 0.061 m and 13.0 deg. |
| Trajectory overlay | The overlay figure itself works (magenta path, start and end markers visible, opened here), but its base panorama is the stretched homography chain, and its path is built from homography frame centres, not from the RGB-D visual-odometry poses, so it should not be read as the visual-odometry trajectory. Fix: re-render the overlay on the ORB-SLAM3-pose map of 080033 with the ORB-SLAM3 path. |
| Four-panel video | Taken as done from the Nov 1 summary; the video file was not opened in this audit, so its moving picture is unverified here. The base panorama panel carries the known far-end stretch. Fix, if the video is reused: rebuild it on the ORB-SLAM3-pose map. |

### NOT GOOD AND PROVEN UNFIXABLE

| Deliverable | Evidence |
|---|---|
| Homography pose graph with a loop edge, on 080033 | Converged to 1.14 px RMS over all edges, yet the map, opened here, is a large sheared diamond with radial smear throughout: the stretch moved into the middle instead of being removed. A weight-10 sensitivity run gives essentially the same map. The pairwise homographies agree with each other but are jointly wrong, each carrying the same systematic bias from fitting a planar model to a translating camera over 3D grass. Joint optimization can only redistribute that bias, not remove it. Closed as an approach for this data. |
| Linear distribution of the loop correction, on 080033 | Correction Frobenius norm 755.8, from the stable-homography JSON. Distributing it linearly gives a 6766 by 7829 px canvas against 2170 by 1767 px uncorrected, and the render was skipped as too large. Worse than doing nothing. Closed. |
| Tree placement, on 173807 | 19,110 of 19,110 pairs completed, 2,252 usable edges, 195 tree edges used, 0 non-neighbour edges used, from the tree stats JSON. The tree canvas is 57,504 by 56,538 px, identical to the direct chain to the pixel (57,503.96 by 56,537.68). The tree panorama, opened here, is two hair-thin diagonal streaks on a black void, byte-identical to the direct-chain render. The take has no loop closures (all 25 end-vs-start pairs at 0 inliers), so there is nothing for a tree to exploit. Closed for this take; a tree could still help on a take that does close, but no such long take exists in the dataset. |
| Homography chain panorama of 173807, in any placement tried | Direct chain canvas 57,504 by 56,538 px, normalized chain 20,373 by 24,820 px, tree identical to the direct chain. All three renders are diverged streaks or fans, opened here. No homography placement yields a usable map of this take. The visual-odometry orthomosaic is no better (smeared blotch, opened here). The take needs the ORB-SLAM3-pose route, which has not been run on it. |

### BLOCKED

| Deliverable | Blocked on |
|---|---|
| Julius's code review, drift work, noise work, dynamic odometry, IMU-only loop detection | Repository access. The team repository returns 404 to the stored credential. Needs Julius or Don to grant read access, or the code pushed somewhere readable. |
| Don's empirical reliability score and wind separation | A still-camera windy clip (camera still, grass moving) from Jinming, plus Don's own score file, neither of which exists in the sources read here. The Nov 6 batch does not contain such a clip. |
| Meaning of 174702 and 174818 are loop | One question to Julius. Both measurements are already done and disagree with a return-to-start reading. |
| Recommendation-letter ask | Harsh's inputs. Listed only; no work done here. |
| Walkthrough meeting | No owner or date set. Nothing in the files read here schedules it. |

## 3. Fix list for Phase 2, in priority order

Confirmed from the parent task, corrected or extended where the
audit found something different. Items already good need no work and
are listed at the end.

1. **Render ORB-SLAM3 RGB-D-only maps for the weak takes.** This is
   the highest-value item: it is the one route already proven to
   produce a good map (080033), and it fixes the whole class of
   smeared visual-odometry orthomosaics at once. Priority takes:
   075947 (genuine loop, visual-odometry map heavily smeared),
   173807 (no usable map in any placement tried), 172532 and
   172828 (weak loops, orthomosaics in two disconnected wedges),
   174702 and 174818 (orthomosaics smeared or split). Also re-run
   the 085611-style check: ORB-SLAM3 RGB-D-only already runs on
   085611, so its map should be rendered from those poses instead
   of the drifted visual-odometry ones. Do not spend further effort
   on homography chain, tree, or pose-graph placement for these
   takes; those approaches are closed for this data.
2. **Depth warp into the color frame.** The depth map is not
   aligned to the color frame (5.9 cm baseline between the cameras).
   Both the visual odometry and the ORB-SLAM3 baseline use depth
   as-is with the depth intrinsics, so the comparison is fair, but
   the warp is the open improvement named in both the ORB-SLAM3
   summary and the pose-graph summary. Re-render the 080033
   ORB-SLAM3-pose map with warped depth and check whether it
   sharpens further. If it does, apply the warp to the item 1
   renders before finalizing them.
3. **Optical-flow holdout for the reliability model.** Replace
   per-frame SIFT re-detection with optical-flow tracking so
   feature tracks last longer than a median length of 1, then
   re-run the HMM on 173701, 174745, and 080033 and check whether
   the Markov temporal part starts contributing. This is also the
   concrete form of the open question 4 about leaving frames for
   reliability checking: hold out frames from the tracking and
   test whether the reliability score predicts the held-out
   residuals. Fit the transition matrix and the unreliable sigma
   from data instead of setting them by hand.
4. **Wind versus parallax on 173701.** The HMM flags non-rigidity,
   not wind, and the feature table labels 173701 as non-rigid
   signal not separated from parallax. The clean test needs the
   still-camera windy clip from Jinming (camera still, grass
   moving): on a still camera the rotation-only predicted pixel
   motion gives a per-feature residual that is wind plus noise,
   with no parallax when the camera does not translate. Until that
   clip exists, do not claim any take shows wind. If the clip
   never arrives, the honest fallback is a synthetic test (item 6).
5. **IMU channel in the HMM.** Designed but not implemented in the
   reliability prototype: per pair, compare the gyro-integrated
   rotation with the visual rotation as a pair-level prior weight.
   Implement it after item 3, since it needs the same tracking
   machinery, and test it on the same three takes.
6. **Synthetic brightness steps.** The Nov 6 batch shows
   illumination change thins SIFT matches but does not break
   tracking (14 percent drop on high-jump pairs on 085611, 9
   percent on 085301, 13 percent on 085453; no pair fails in any
   take). The batch has no abrupt step, only slow oscillations and
   one 10.14 jump that is a white sign entering the frame, not a
   light change. Apply controlled brightness steps to a fixed-view
   take (085611 is the cleanest candidate) and measure where
   matching actually breaks. This also covers the synthesized-data
   suggestion for abrupt intensity change, using existing frames
   instead of waiting on new recordings.
7. **Gyro prior for the visual odometry.** The visual-odometry
   rotation is short on the moving takes and badly wrong on the
   near-static take (86.6 deg of drift where the gyro says 4.4 deg
   on 085611). A gyro prior on the per-pair rotation would attack
   the single biggest measured weakness of the visual odometry.
   This is lower priority than items 1 and 2 because the
   ORB-SLAM3 route already bypasses the visual odometry; do it
   only if the visual odometry itself still needs to be shown
   working.
8. **Ask Julius the one loop question, and resolve repository
   access.** One question (what loop means for 174702 and
   174818) and one access grant unblock the whole Julius column:
   the drift test, the noise work, the dynamic odometry, and the
   IMU-only loop detection. Nothing in Phase 2 should re-measure
   those two takes before the question is answered.
9. **ORB-SLAM3 RGB-D-inertial: do not tune further; change the
   input instead.** Two noise settings were tried and the failure
   is the initializer's motion gate, not the noise values. The
   only routes are takes with faster, more varied motion, or a
   vision-first warm start. This is the lowest priority item
   because the RGB-D-only baseline already answers the comparison
   the project needs.

Already good, needs no work: the 49-take feature table; the
stitch-all batch and its classification; the Nov 6 batch analysis
and its stitches (7 worked, 1 sort-of-ok); the 080033 ORB-SLAM3-pose
map; the SE(3) refinement null; the Hartley normalization default;
the loop-closure measurements; the reliability plots as figures.
Do not re-run any of these.

## 4. Staged versus uploaded

Everything in this audit is staged locally only, as far as the local
records show. Every summary read here states that nothing from its
work was uploaded: the Oct 30 completion summary states nothing was
uploaded to GitHub and everything is staged for review; the
normalization summary states no upload was made; the ORB-SLAM3
summary states nothing in the task was uploaded; the pose-graph
SE(3) notes state nothing in the task was uploaded.

What the local records do show about the shared repositories:

- The local clone at repo-review/grassvio has a remote pointing at
  a local bundle file, not at a hosted repository, and its log
  holds only the early commits (initial commit, bag extraction,
  ORB method, BFM/HOG/SIFT methods). No recent work is in that
  clone.
- The November 6 code-review note records that the shared
  grassvio-tools repository's recent commits were all by Harsh
  (Nov 2 and Nov 6, render version 2 and loop-closure work), with
  no commits by Julius and no branches besides main. That note is
  from November 6 and is the latest local record of the shared
  repository's state.
- The team repository holding Julius's pushed code is not
  reachable with the stored credential, so its contents cannot be
  verified from here at all.

Upload state beyond those local records is unverified in this
audit: no network calls were made to check a hosted repository,
and no upload was attempted. If Phase 2 needs a definitive answer
on what is on the shared repository, that check has to be done by
someone with access, from the hosted side.

## Appendix: verification notes and discrepancies

- The October 30 minutes file is at
  analysis-drafts/team-chat-analysis-2026-10-30.md. The brief gave
  a different path, which does not exist.
- The loop-closure summary's file list does not include the 080033
  poses JSON or orthomosaic. Those files exist under
  analysis-drafts/rgbd, not under analysis-drafts/loop-closure,
  and the numbers in the summary were recomputed from them here.
- The normalized 080033 canvas is 2170 by 1767 px in the summaries
  and 2169.5 by 1767.2 px in the stable-homography JSON: a rounding
  difference, not a disagreement. The verify JSON records 2171 by
  1769 px, which is the same render measured with a 1 to 2 px
  difference.
- The 085611 ORB-SLAM3 rotation end error is 0.01 deg in the
  summary and recomputes to 0.0 deg at the precision used here.
  Immaterial.
- The 173701 reliability curve oscillates after its mid-take fall
  (about 0.22 to 0.56) rather than staying low, as noted in the
  grades section.
- The Nov 6 stitch results file gives all 8 takes the same
  stitched status; the 7 worked and 1 sort-of-ok tally is a visual
  judgement recorded in the summary, verified by eye in this
  audit for 085224, 085301, 085422, 085453, 085611, and 084542.
  The 084741 and 084852 panoramas were not individually opened
  in this audit.
- The four-panel video file was not opened in this audit; its
  content is taken from the Nov 1 summary.
- The Nov 5 questions and the Oct 30 per-person assignment list
  could not be verified against any file read here; they are
  treated as brief-given throughout.
- The two reply drafts (to Julius on loops, to Jinming on
  question 3 and the Nov 6 findings) were not found in the files
  read here, so their existence, wording, and sent status are
  unverified.
- Don was absent from the Oct 30 meeting, per the brief. No file
  read here lists the meeting attendance, so this audit does not
  independently verify it.

---

## Phase 3 — What Phase 2 fixed, what stays blocked, what is proven unfixable (coordinator update, Nov 9)

All eight Phase 2 items plus three follow-ups were run. The coordinator
re-opened the key new images (173807 and 174818 ORB maps, depth-warp
comparison, gyro-prior 085611 maps, new overlay, new video frame 88,
window-HMM comparison figure) and re-checked headline numbers against
their source JSONs. What follows is the coordinator's verified update.

### Fixed in Phase 2 (before / after, all staged under analysis-drafts/full-audit-20261109/)

1. **ORB-SLAM3-pose maps for weak takes** (`orb-maps/`). ORB better on
   5 of 6 takes, tie on 1. 174702: wedge (30 frames) to one coherent
   patch, 4.95 x 4.34 m, 57.7%, 358 frames. 174818: two patches to one
   patch, 5.76 x 4.93 m, 57.9%, manhole cover sharp and round.
   172532: two patches to one patch, 6.42 x 3.02 m, 53.8%; ORB loop
   ends 0.81 m / 9.3 deg from start vs VO 6.10 m / 101.0 deg.
   172828: two patches to one patch, 5.85 x 4.23 m, 43.2%, ends
   0.83 m from start. 155419: tie, keep the VO map (same wall and
   grass strip; trajectories agree to 3 cm / 0.7 deg). 173807: ORB
   better but STILL STREAKY — larger footprint with real texture,
   7.76 x 5.08 m, 56.0%, 730 frames, but radially streaked with
   holes; better, not good. 172859 skipped (brick wall, not grass).
   ORB tracked 100% of frames on every take; total rotation matches
   the gyro within 1% on five takes, 8% on 173807. Build note: a VM
   restart wiped apt packages and the full -dev set would not
   reinstall (libzstd-dev blocked by a patched libzstd1); runtime
   libraries only plus LD_LIBRARY_PATH were enough for the
   prebuilt binary, verified by a smoke test reproducing 080033
   exactly (440 poses).
2. **Depth warp** (`depth-warp/`). Clean null: warping depth into the
   color frame does not sharpen the 080033 ORB map (indistinguishable
   by eye at full scale and 2x zoom; average render blurrier,
   Sobel -6.4%, Laplacian variance -23.9%). Warp direction A settled
   by three checks. The unwarped ORB map stays the best map.
3. **IMU channel in the HMM** (`imu-hmm/`). Negative: does NOT
   separate better. Signal take 173701 falls 0.472 to 0.219 above
   0.8, but clean control 174745 collapses 0.978 to 0.162 and clean
   rotating takes lose 8-25 points. Calibration findings: points
   move by the inverse gyro rotation; gyro timestamps lag colour by
   ~50 ms; the gyro-mean "still" gate is too loose (median visual
   rotation under ~0.001 rad/pair separates the still groups
   cleanly). Channel (b) works where the camera is really still
   (Nov-06 stills match residual-only; 084542 micro-shake caught).
4. **Wind vs parallax on 173701** (`wind-parallax/`). Signal persists
   while the camera is still — not parallax. Still-segment residual
   median 0.67 px vs control 0.19 px; onset at ~0.8 s with gyro and
   proxy at control level; residual-proxy correlation vanishes after
   detrending (-0.14). Wind is the leading explanation, not proof.
   Label updated to "non-rigid signal, persists while still" for
   onset and floor; the peak amplitude keeps the parallax qualifier
   (second-half gyro nearly doubles; top-3 gyro pairs are the
   top-3 residual pairs).
5. **Optical-flow holdout** (`optflow/`). Works at the window scale,
   not per frame step. Flow tracks span whole takes (median 123 /
   253 / 385 frames vs SIFT's 1 pair). Window-scale flow-vs-rigid
   residual separates ~100x (173701 median 6.0-12.4 px per window
   vs 0.02-0.13 px on controls). SIFT anchor matches collapse
   2,803 to 67-166 per window where the signal appears. Per-step
   flow HMM separates less than the SIFT HMM (0.834 vs 0.472 on
   173701 against ~0.98 controls). Detection can drop to every
   25th frame with flow filling the gaps — Jinming's question 4
   answered.
6. **Synthetic brightness steps** (`synth-brightness/`). Null, and a
   useful one: SIFT barely notices uniform global steps (loss under
   3% up to +/-50 gray levels; no pair ever failed; worst pair still
   1479 inliers). The real ~14% loss on 085611 at real jumps of at
   most 4.39 gray levels must come from the spatial structure of
   real light change (moving shadows), not its size. Stated plainly
   as a synthetic manipulation of real data, not a simulator.
7. **Gyro prior for the VO chain** (`gyro-prior/`). Partial help for
   rotation only. Seeding PnP is a null (bit-identical). Veto/replace
   cuts end rotation 86.33 to 0.21 deg on the still take and 67.62
   to 18.92 deg on the loop take, but translation drift is untouched
   (0.26 m still, 0.88 m loop, mostly vertical), so maps stay
   smeared. Ceiling is the gyro itself (its own net rotation is
   21.1 deg on the loop take). Does not beat ORB-SLAM3 RGB-D.
   Stopped after two takes per the stop rule.
8. **Window-scale HMM, follow-up** (`window-hmm/`). Negative as a
   general model: crushes the signal take (0.008 above 0.8 vs
   SIFT's 0.472) but breaks the clean rotating take 080033 (0.637
   vs SIFT's 0.975). On still takes alone it beats SIFT decisively
   (gap 0.952 vs 0.509). Diagnosis: the fitted sigma_R (0.128 px)
   sits below the rotation take's ordinary rigid misfit and 17
   windows compound it; a plain threshold on the same window
   residuals beats the fitted HMM (gap 0.924, clean min 0.856).
   The 0.5/5.0 px sensitivity run looks good on all takes, but any
   fix must be pre-specified and fit on held-out takes, not tuned
   after seeing the results.
9. **Small correctness fixes** (`small-fixes/`). K_COLOR in
   rgbd_odometry.py fixed to the bag values (fx/fy were 0.21 px too
   high) — immaterial result: the 86 deg drift survives exact-match
   intrinsics (0.238 to 0.245 m / 86.33 to 86.80 deg). Side finding:
   every session carries its own calibration, so a single hard-coded
   K_COLOR can never match all takes; a per-bag calibration read
   would be the fuller fix. Feature-table proxy column corrected:
   cause found (Oct-30 table built with an earlier explicit-Euler
   integration variant; the script changed for the Nov-1 extension
   and old rows were never recomputed); all 36 affected appendix
   rows replaced, 4 prose fixes, correction note added. Wind/loop
   verdicts verified unaffected. What changes: 12 takes cross
   translation bands; 6 motion-class labels need re-check (172859,
   172912, 172955, 173250, 173507, 155419) — left as published and
   flagged, not silently rewritten. One row (102608b) matches
   neither variant; unexplained.
10. **Visual refresh** (`visual-refresh/`). New trajectory overlay on
    the ORB map kept: the old magenta loop spans 417x289 px and
    that size is chain drift, not travel — the real camera stayed
    in a 9x5 cm patch; the new figure shows the coherent patch
    with ring start/end markers plus a centimetre-scale detail
    figure. Four-panel video rebuilt on the ORB map and verified
    (final panorama panel matches the ORB map at 99.0% footprint
    overlap; IMU arc still drifts off while the ORB loop closes).
    Depth-based translation check on 173701: elevated (median 3D
    displacement 22.6 mm vs 4.1 mm control) but INCONCLUSIVE —
    artifact-dominated at centimetre level (fitted translation
    implies ~1 m of travel, ruled out by a long-baseline check;
    PnP rotation is 2.3x the gyro on both takes). Two corrections
    to the wind record: "frame 0 vs 200, same viewpoint, camera
    did not move" is too strong (the camera rotates ~30 deg over
    the take); the sharp next step is to warp depth into the color
    frame and re-run this check before trusting any depth-based
    translation claim at centimetre level or below.

### Still BLOCKED (exactly who or what is needed)

- Julius's code review, drift work, noise work, dynamic odometry,
  IMU-only loop detection: repository access from Julius or Don
  (team repo returns 404 to the stored credential). Nothing new
  was measured here; re-measuring is pointless until access lands.
- What "loop" means for 174702 and 174818: one question to Julius.
  (Partial new evidence: ORB maps show 172532 and 172828 ARE big
  loops returning to ~0.8 m, and 174702's path crosses itself once
  mid-take — but whether Julius meant a crossing, an intent, or a
  rotation for 174702/174818 is still his to answer.)
- Don's empirical reliability score and wind separation: the
  still-camera windy clip from Jinming (never recorded; the Nov 6
  batch contains none) plus Don's own score file.
- Walkthrough meeting: no owner or date set.
- Recommendation-letter ask: Harsh's inputs (off-limits, listed
  only).
- Two team-chat reply drafts (to Julius on loops, to Jinming on
  question 3 and the Nov 6 findings): the coordinator verified by grep
  that they exist NOWHERE as files in the workspace or goal
  folder — they live only in chat history. Wording and sent
  status are unverified from files.

### PROVEN UNFIXABLE (with the evidence)

- Homography chain, tree, linear loop distribution, and homography
  pose graph on this data (Phase 1 grades stand; the pose-graph
  null was re-confirmed in Phase 2 item 7's spirit by the
  gyro-prior ceiling and the ORB comparison).
- Depth-warp sharpening of the 080033 map (clean null, Phase 2
  item 2).
- IMU channel improving HMM separation (negative with diagnosis,
  item 3).
- Window-scale HMM with data-fitted sigmas as a general model
  (negative with diagnosis, item 8).
- Gyro prior fixing the VO chain (partial only; translation drift
  untouched; ceiling is the gyro, item 7).
- RGB-D-inertial mode on these takes (initializer motion gate;
  needs faster/more varied motion or a vision-first warm start).
- SIFT failing under uniform brightness steps (null: it does not
  fail; robustness result, item 6).

### Not yet GOOD — the one remaining open deliverable

- **173807's map.** The ORB map is the best available and is better
  than every earlier attempt, but it is still radially streaked
  with holes — better, not good. No method tried produces a clean
  map of this take (homography chain/tree/pose-graph all diverge;
  VO map is a smeared blotch; ORB map is streaky). Nothing is
  proven unfixable here — it is unfinished, not closed. The
  plausible next attempts, none run: render with a wider tilt
  allowance or different blending to fill the holes, or a longer
  ORB run with loop closure on a take that closes (173807 does
  not close).

### Staged vs uploaded — CORRECTED

The Phase 1 audit's Section 4 said upload state was unverified
because no network calls were made. The coordinator has now checked
the hosted repository (gh-list tree, Nov 9): 125 files. The Nov 6-7
work IS on GitHub, including the leftover-three summaries, the
pose-graph summary and maps, the new-takes summary, the reliability
summary and plots, the ORB-SLAM3 drivers, and the updated
stitch_map.py with normalized fitting as the default. The local
repo-review clone is stale (early commits only) and must not be used
as the record of the shared repo. Only the Nov 9 full-audit work is
staged-only: `analysis-drafts/full-audit-20261109/` (audit, orb-maps,
depth-warp, imu-hmm, wind-parallax, optflow, synth-brightness,
gyro-prior, window-hmm, small-fixes, visual-refresh). Two files
edited during Phase 2 are staged, not yet uploaded: the corrected
`analysis-drafts/recording-feature-table.md` (36 proxy rows +
correction note) and `dataset-tools/stitch-map/rgbd_odometry.py`
(K_COLOR fix). The corrected table also needs uploading to the
shared repo before anyone cites the proxy column again.

### Remaining open items (small, real — not done, not blocked)

- 6 motion-class labels need re-check after the proxy correction
  (172859, 172912, 172955, 173250, 173507, 155419); 102608b proxy
  row unexplained.
- Per-bag calibration read for rgbd_odometry.py (fuller K fix).
- Depth-warped re-run of the 173701 translation check.
- Window-HMM sigma question: pre-specify the sigmas (or the
  fitting procedure on held-out takes) and re-run; do not tune
  after seeing results.
- Spatially-varying synthetic brightness test (sweeping dark band)
  — the follow-up that would actually test the cloud case.
- ORB-SLAM3 inertial: needs new motion or a new init path.
- Don's Q1 and Julius's Q2 remain theirs; Jinming's still clip
  still missing; walkthrough unscheduled; rec letter needs
  Harsh's inputs; reply drafts exist only in chat history.
