# Dataset Landscape: Visual / Visual-Inertial Odometry & SLAM in Vegetation, Agricultural, and Natural Outdoor Environments

Research notes for the GrassVIO dataset & experimental-infrastructure work package.
Compiled 2026-09-27. Purpose: determine whether any existing public dataset already
covers what GrassVIO needs, or whether collecting a new controlled dataset is justified.

## Target combination (what GrassVIO needs)

1. RGB-D or stereo + IMU, synchronized (the D455f-class sensor suite)
2. Vegetation-dominant, close-range scenes (lawn / grass, not distant tree lines)
3. Wind as an explicit, controlled or labeled experimental factor (the non-rigid motion source)
4. Ground truth or pseudo-ground-truth usable for descriptor and reliability evaluation

## Datasets surveyed

### 1. Rosario Dataset — Pire et al., IJRR 2019
- Sensors: stereo RGB 672x376 @ 15 Hz, IMU 142 Hz, wheel odometry, RTK-GNSS (position GT)
- Environment: soybean field, 6 sequences
- Wind: not a factor; listed challenges are repetitive appearance, direct-sunlight burn-in, rough terrain
- Access: paper https://arxiv.org/abs/1809.06413

### 2. Rosario Dataset v2 — Soncini et al., IJRR 2025
- Sensors: stereo infrared + color camera, accelerometer/gyroscope/magnetometer, GNSS (SPP/RTK/PPK), wheel odometry; hardware-synchronized; 6-DOF ground truth
- Environment: soybean crop field, 2+ hours of data
- Wind: not a factor; challenges listed are natural lighting variation, motion blur, rough terrain, perceptual aliasing
- Access: paper https://arxiv.org/abs/2508.21635v1 ; data released on the authors' site (cifasis.github.io/rosariov2)
- Note: the strongest agricultural VI dataset on sensor sync and GT quality; still no wind, no dense depth

### 3. FinnForest — Ali et al., Robotics and Autonomous Systems 2020
- Sensors: 4x Basler RGB cameras, KVH 1750 IMU, NovAtel GNSS; non-drifting timestamps
- Environment: Finnish coniferous forest + suburban, summer AND winter revisits
- Wind: not a factor; the studied variation is seasonal (summer/winter), not wind
- No RGB-D. Access: paper https://trepo.tuni.fi/bitstream/handle/10024/216831/FinnForest_dataset_2020.pdf?sequence=1&isAllowed=y

### 4. ROVER — arXiv 2412.02506, 2025
- Sensors: monocular + stereo + RGB-D cameras, camera-internal and external IMUs; millimeter-accurate total-station ground truth
- Environment: parks, gardens, campus — 39 recordings, 7.2 km, 311 min, ~1 TB, all four seasons, day/dusk/night
- Wind: not a factor; studied factors are seasonality, lighting, vegetation density
- Access: paper https://arxiv.org/abs/2412.02506v3
- Note: closest overall match on sensors (RGB-D + IMU) + vegetation, but wind is absent and vegetation is background, not the controlled variable

### 5. BotanicGarden — Liu et al., arXiv 2306.14137, 2023 (submitted IEEE RA-L)
- Sensors: high-res stereo gray+RGB, rotational + forward 3D LiDARs, low-cost and industrial-grade IMUs; calibrated, hardware-synchronized; accurate ego-motion + 3D map GT + semantic annotations
- Environment: 48,000 m^2 botanic garden incl. thick woods, riversides, narrow paths, bridges, AND grasslands; 33 sequences, 17.1 km, 2.3M images
- Wind: not a factor; emphasis is GNSS-denied, texture-monotonous, unstructured navigation
- Access: paper https://arxiv.org/abs/2306.14137
- Note: grasslands present and sensor suite is excellent, but no wind labeling and scenes are traversal-scale, not close-range controlled takes

### 6. Sugar Beets (Bonn) — Chebrolu et al., IJRR 2017
- Sensors: multi-spectral + RGB-D cameras, LiDAR, GNSS, wheel encoders
- Environment: sugar-beet field; cameras point straight DOWN for plant classification
- Wind: no. No IMU-for-VO, no SLAM evaluation at all (per Cremona et al.'s survey of it)
- Verdict: wrong task (classification), wrong camera geometry for VIO

### 7. Flourish / Sapienza mapping dataset — Imperoli et al., 2018
- Sensors: forward-looking stereo camera + IMU (same robot platform family as Sugar Beets)
- Environment: agricultural field; intended for SLAM/VO/VIO testing
- Wind: no. Small-scale evaluation only

### 8. UMA-VI — Zuniga-Noel et al., IJRR 2020
- Sensors: stereo color (Bumblebee2) + custom stereo rig + Xsens MTi IMU @ 250 Hz; 32 sequences, ~80 min
- Environment: University of Malaga buildings — corridors, parking, classrooms, halls (INDOOR)
- Wind: no. Covers low texture + dynamic illumination, the illumination half of GrassVIO's problem, but no vegetation
- Access: http://mapir.isa.uma.es/mapirwebsite/?p=2108

### 9. UZH-FPV — Delmerico et al., ICRA 2019
- Sensors: event camera, monocular RGB, stereo gray, IMU; aggressive drone flight
- Environment: hangar + GRASSLAND (outdoor)
- Wind: no; the challenge is aggressive motion, not environmental motion
- Verdict: grassland present but the scientific variable is ego-motion, not vegetation motion

### 10. CTU MRS forest dataset — ctu-mrs/slam_datasets (GitHub)
- Sensors: OS0-128 LiDAR + IMU, RealSense D435 RGB-D (forward-facing), down-facing rangefinder, RTK GPS
- Environment: ~1.25 km, 19-min UAV flight through sparse forest
- Wind: no. RGB-D + IMU in a forest exists here, but it is one uncontrolled drone flight, not a controlled vegetation-motion study
- Access: https://github.com/ctu-mrs/slam_datasets/blob/HEAD/forest/README.md

### 11. Low-viewpoint forest depth dataset — Niu et al., IROS 2020
- Sensors: custom rig, RGB + depth, GPS, IMU, wheel encoders; ~200k RGB-D pairs (9.7k curated)
- Environment: forest floor — grass, bushes, fallen trees — from a small rover viewpoint
- Wind: no. Built for monocular depth prediction, not VIO benchmarking
- Access: paper https://arxiv.org/abs/2003.04359v2

### 12. FieldSAFE — Kragh et al., 2017
- Sensors: stereo RGB, 360 RGB, thermal, IMU; obstacle-detection task in grass
- Wind: no. Not a VIO/SLAM dataset

### 13. RWS adverse-weather "Wind" subset — van Lier et al., WACV 2025 workshops
- The ONLY surveyed source with wind as an explicit labeled factor: grassy dune scene, wind > 35 km/h, extensive moving vegetation, 99 frames / 366 annotated objects
- BUT: static surveillance cameras at 30 m elevation, small-object DETECTION task — no camera motion, no IMU, no depth, unusable for VIO
- Access: http://openaccess.thecvf.com/content/WACV2025W/RWS/papers/van_Lier_Evaluation_of_Spatio-Temporal_Small_Object_Detection_in_Real-World_Adverse_Weather_WACVW_2025_paper.pdf
- Significance: proves the community recognizes wind-blown vegetation as a distinct challenge, and that nobody has built the VIO version of it

### 14. VIODE — Minoda et al., IEEE RA-L 2021
- Simulated stereo-inertial sequences with moving objects injected; the standard dynamic-VIO benchmark
- Dynamics are rigid moving OBJECTS, not distributed non-rigid vegetation — the exact distinction GrassVIO draws

## Q1: Does ANY existing public dataset combine wind-blown vegetation with RGB-D (or stereo) + IMU?

NO. Across 14 surveyed datasets, not one combines (a) wind as an explicit/labeled factor with
(b) RGB-D or stereo + IMU and (c) vegetation-dominant scenes for odometry/SLAM evaluation.
Wind appears exactly once as a labeled factor (RWS wind subset) and that source has static
cameras, no IMU, no depth, and a detection task. Every VI-capable vegetation dataset
(Rosario v2, ROVER, BotanicGarden, FinnForest, CTU forest) studies other factors —
lighting, season, texture, terrain, ego-motion — while treating vegetation as static background.

## Q2: Closest misses and exactly why they fall short

| Dataset | Has | Missing for wind-induced non-rigid vegetation motion |
|---|---|---|
| ROVER (2025) | RGB-D + IMU, gardens/parks, 4 seasons, mm-accurate GT | No wind factor; vegetation is backdrop, not the controlled variable; no still-camera takes to isolate environmental motion |
| BotanicGarden (2023) | Stereo + IMU hw-sync, grasslands, 17 km, GT + semantics | No wind labeling; traversal-scale, not close-range controlled takes |
| Rosario v2 (2025) | Best ag VI sync + 6-DOF GT | No wind; no dense depth; soybean crop, not lawn; robot always moving (no still-camera baseline) |
| CTU forest (UAV) | RGB-D + IMU in forest | Single uncontrolled flight; no wind conditions; no ground-truth reference for vegetation motion |
| RWS wind subset (2025) | Labeled wind + moving vegetation | Static cameras, no IMU, no depth, detection task — unusable for VIO |
| UMA-VI (2020) | Stereo + IMU, controlled illumination stress | Indoor, zero vegetation |
| TB-Places (2019) | Garden visual place-recognition dataset | Recognition task, not odometry; no wind factor, no IMU [title-level] |
| EDEN (WACV 2021) | Synthetic gardens, 300K+ images, depth/flow/segmentation | Synthetic (no real wind); single images, not VIO sequences; no IMU |

The pattern: datasets either have the right SENSORS or the right PHENOMENON, never both.

## Q3: Bottom line — use/augment existing, or collect?

Collecting a new controlled dataset is JUSTIFIED as a contribution, and the gap is clean:
no public dataset isolates wind-driven non-rigid vegetation motion with VI sensing.
The proposed 2x2 condition matrix (camera still/moving x vegetation still/moving, with
illumination as a second factor) is precisely the experimental design missing from the literature.

Recommendation for the team:
- USE existing datasets as baselines/comparisons, not as substitutes: run the descriptor
  study (RQ1) on ROVER garden sequences and Rosario v2 to show the method generalizes
  beyond the team's own lawn; cite ROVER/BotanicGarden as the "vegetation SLAM" state of data.
- COLLECT the controlled set (short tripod + handheld takes, fan + natural wind, rigid
  controls in frame) as the core novel data contribution — this is what makes the
  reliability study (RQ2) possible at all, since no existing data labels wind.
- The RWS wind subset is worth one citation as evidence the problem is recognized but
  unsolved for VIO.
