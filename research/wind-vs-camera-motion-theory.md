# Wind vs camera motion: theory survey and ORB-SLAM3 notes

Notes on three questions: (1) what theory and papers exist for telling wind
movement apart from camera movement, and which parameters to tweak; (2) what
real datasets we could use; (3) how our code could plug into ORB-SLAM3 for
testing.

## 1. The core geometric distinction

Camera ego-motion explains ALL image motion with ONE rigid transform between frames
(epipolar geometry / essential matrix / homography). Wind moves individual blades
non-rigidly: each blade oscillates around its own rest position with high-frequency,
low net displacement, locally correlated with neighbors but globally incoherent.
So after compensating ego-motion, wind is exactly what is LEFT OVER: residual flow
that no single rigid transform explains.

This is the standard dynamic-SLAM problem, but wind is the hard case: the
literature mostly handles rigid movers (people, cars). Wind-driven vegetation is
non-rigid, distributed, and self-occluding — which is precisely why GrassVIO's
per-feature reliability weighting is the right framing rather than a binary
dynamic/static mask.

## 2. Papers and theories that apply

### 2a. Moving consistency check — DS-SLAM (Yu et al.)
Combines semantic segmentation (SegNet) with a **moving consistency check** based
on epipolar geometry: after estimating camera motion, each matched feature is
checked against the epipolar constraint; features whose residual is inconsistent
with the rigid motion are rejected as dynamic. This is the directly reusable
algorithm for our pipeline: we already estimate camera motion (homography in the
sim pipeline, essential/homography in VO), so the same residual check slots in
with no new sensor and no GPU.

Parameters to tune for this test:
- **Residual threshold** (pixels): how far off the epipolar line / reprojection a
  feature must be before it counts as inconsistent. Start near the RANSAC inlier
  threshold, then sweep.
- **RANSAC inlier threshold** on the motion model itself: tighter threshold =
  fewer wind-corrupted features poison the ego-motion estimate, but too tight
  drops good matches in texture-poor grass.
- **Minimum feature track length**: wind-fluttered blades produce short,
  broken tracks (the descriptor fails as the blade bends). Requiring tracks of
  length >= N frames is a cheap wind filter; sweep N = 3..8.

### 2b. Geometry-only dynamic detection — DynaSLAM (Bescos et al., ORB-SLAM2 based)
Detects dynamic objects via **multi-view geometry, Mask R-CNN semantics, or both**,
then inpaints the background. The geometry-only branch is the relevant precedent:
it needs no neural network — dynamic content is found from depth/reprojection
inconsistency across views. For us: with stereo depth from the D455f, the same
idea becomes a **depth-consistency check** — a feature whose triangulated depth
disagrees with the rigid-motion prediction across frames is environmental motion.

### 2c. Coarse-to-fine static probabilities — CFP-SLAM
Computes per-feature static probabilities and uses them as **weights in pose
optimization** instead of binary inlier/outlier labels. This is the closest
published template for GrassVIO's core idea (per-feature reliability weights).
Cite it as the "soft weighting" precedent; GrassVIO's novelty is the
vegetation-specific reliability cues (BFM descriptors, stereo-depth quality,
IMU consistency, persistence), not weighting itself.

### 2d. Distributed + local RANSAC — DM-SLAM
Extracts static features with distributed and local RANSAC rather than one global
fit. Relevant if global RANSAC fails when a large fraction of the frame is grass
in wind: local RANSAC on image tiles can still find a rigid consensus where the
global fit collapses.

### 2e. Survey taxonomy — ACM Computing Surveys 2018 on dynamic SLAM/SfM
Three families, each with a GrassVIO-relevant note:
- **Optical-flow-based**: no prior knowledge needed, real-time friendly; but
  sensitive to illumination change and degenerate motion (object moving with the
  camera looks static). Our controlled takes + fixed lighting make this the
  weakest risk.
- **Ego-motion-constraint-based**: segments static features by fitting the known
  ego-motion. Needs prior camera-motion knowledge — **our IMU gives exactly
  that** (see §3), which turns this from a weakness into our main lever.
- **Statistical model selection**: automatic, handles degenerate motion, no
  environment priors. The fallback if the IMU path underperforms.

### 2f. Visual anemometry — the physics of what wind does to vegetation
- **"Seeing the Wind" (Cardona, Howland, Dabiri, NeurIPS 2019)**: a
  CNN+RNN predicts wind speed 0.75–11 m/s from flag/tree motion, matching a
  co-located cup anemometer, with error approaching natural turbulence.
  Generalizes across flags and a wind-tunnel test. Physics scaling relates
  flapping dynamics to frame rate and duration.
- **EarthArXiv OFTV preprint (2026)**: physics-based tree simulation +
  Optical Flow Tracking Velocimetry; modeled tree velocities increase
  approximately linearly with input wind speed (R^2 ~ 0.9); the response is
  angularly isotropic (camera angle 0–180 deg does not matter much).

Why this matters for GrassVIO: it establishes the signature of wind motion —
amplitude scales ~linearly with wind speed, concentrates in a flutter band, and
oscillates around a rest position. That gives two concrete tests: (a) temporal
bandpass on feature tracks — wind energy sits at ~1–5 Hz oscillation, camera
translation is smooth low-frequency drift; (b) net-displacement / path-length
ratio per track — wind-fluttered tracks have long paths but near-zero net
displacement; translating camera gives net displacement in one direction.

### 2g. Honest overlap check (from vegetation-vio-competitors.md)
- DynaVINS (Song et al., IEEE RA-L 2022) already rejects features using IMU pose
  priors. Do NOT claim novelty for IMU-consistency weighting alone.
- GrassVIO's defensible contribution: vegetation-specific reliability cues
  (flutter-band analysis, BFM descriptors, depth quality, persistence) packaged
  as one per-feature reliability front end, evaluated on a dataset where wind is
  the controlled variable — which no public dataset provides (see §4).

## 3. The IMU anchor — our cleanest lever

We already integrated the IMU (accel filtering → per-frame-pair distance, compared
against visual motion). The theory above says: use the IMU-integrated motion to
**predict the optical flow field**, subtract it from the observed flow, and treat
the residual as environmental motion. In still-camera takes the predicted flow is
~zero, so the residual IS the wind field — this is exactly the 2x2 experiment
matrix's wind-on/wind-off, camera-still column, and it needs no motion model fit
at all. In moving-camera takes, the same subtraction separates the two sources
before any descriptor work. This is the ego-motion-constraint family from the
survey, with the IMU supplying the "prior knowledge" it normally lacks.

## 4. Real datasets

From the dataset survey in dataset-landscape.md (14 datasets):
- **No public dataset combines wind as a labeled factor with RGB-D/stereo + IMU
  for VO/SLAM.** The only wind-labeled source is the RWS wind subset (WACV 2025
  workshop): static surveillance cameras at 30 m, detection task, no IMU, no
  depth — unusable for VIO, but citable as proof the community recognizes
  wind-blown vegetation as a distinct challenge.
- For **real outdoor camera motion**, the three baselines
  already surveyed stand: **ROVER** (RGB-D + IMU, gardens/parks, mm-accurate
  total-station GT), **BotanicGarden** (stereo + IMU hw-sync, 17 km, includes
  grasslands), **Rosario v2** (best agricultural VI sync + 6-DOF GT). Run the
  descriptor study on ROVER garden sequences to show generalization beyond our
  lawn.
- Bottom line: our own controlled takes, plus the wind videos still to be
  recorded, remain the only wind data; existing datasets serve as real-motion
  baselines, not substitutes.

## 5. ORB-SLAM3 integration investigation

### What ORB-SLAM3 assumes
- ORB end to end: custom ORB extractor at 8 pyramid levels, Hamming-distance
  matching, and a **DBoW2 ORB vocabulary** for place recognition / loop closure /
  relocalization.
- Sensor modes include exactly what we need: **stereo-inertial** (our D455f =
  stereo + IMU) and RGB-D-inertial. Existing RealSense integrations exist
  (orb_slam3_ros2 configs; the urbste fork supports D435i/T265). Sensor config is
  YAML: intrinsics, stereo baseline, IMU noise/bias parameters.

### What a descriptor swap touches
Replacing ORB with SIFT/HOG/BFM is not a config change; the swap points are:
1. **Extractor** (include/ORBextractor.h, src/ORBextractor.cc): must still emit
   per-pyramid-level keypoints + descriptors. A HOG/BFM extractor would need the
   same interface.
2. **Matcher** (ORBmatcher): Hamming distance is hardcoded for binary
   descriptors; float descriptors (SIFT/HOG/BFM) need a Euclidean-distance
   matcher path.
3. **Loop closure / relocalization vocabulary**: the DBoW2 vocabulary is trained
   on ORB descriptors — a new descriptor needs a retrained vocabulary, or loop
   closure must be disabled for the experiment.
- Precedent: **SUPERSLAM3** (isarlab fork) implements exactly this pattern —
  interchangeable ORB and SuperPoint backends behind one matcher interface
  (SPDetector.cpp vs ORBextractor.cc, libtorch dependency for the learned one).

### Recommended plan
- Midterm scope is the sim VO pipeline only; IMU + real data moved to the
  final project. So ORB-SLAM3 work is correctly **final-project scope**:
  (1) build stock ORB-SLAM3 and run **stereo-inertial** on our bag data as the
  trajectory baseline — no code changes, just a D455f YAML; (2) keep descriptor
  experiments (SIFT/ORB/BFM/HOG) in our own VO pipeline for the midterm;
  (3) descriptor swap inside ORB-SLAM3 only if the final project needs it, via
  the SUPERSLAM3 pattern (extractor + matcher metric + retrained vocabulary).
- Build note: ORB-SLAM3 wants OpenCV 4.x — consistent with our existing
  opencv<5 pin, so no dependency fight.

## 6. Suggested parameters-to-tweak table for the wind test

| Knob | What it does | Sweep direction |
|---|---|---|
| RANSAC inlier threshold (motion fit) | decides which features set the ego-motion | tighten until wind stops poisoning the fit, stop before good matches drop |
| Moving-consistency residual threshold | per-feature wind label after motion fit | start at RANSAC threshold, sweep in pixels |
| Min feature track length | drops flutter-shorted tracks | N = 3..8 frames |
| Flutter-band energy (new cue) | per-track 1–5 Hz energy / net-displacement ratio | threshold on ratio; wind ~ low net/path |
| IMU-predicted-flow residual weight | blends visual vs IMU evidence per feature | 0 (visual only) → full IMU anchor |
| Depth-consistency threshold | stereo depth vs rigid prediction | sweep in depth units at blade range |

## Caveats
- Most paper summaries above come from search results, so open the original
  papers before citing any of them in the report. CFP-SLAM and DM-SLAM in
  particular still need a read of the original paper.
- The anemometry papers estimate wind FROM video; we do the inverse (remove wind
  from VO). The shared math is the flutter signature, not the full method.
- For our own takes we found no evidence of wind, so the theory above applies
  to the new wind recordings and the synthetic deformation study until real
  wind takes exist.
