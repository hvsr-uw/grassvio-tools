# Multiview reconstruction and motion validation

I ran a longer image-only reconstruction on all six November 19 videos. Five videos use the ordinary keyframe schedule. The fast video needs shorter temporal windows, closer keyframes and a larger landmark budget. The refined gyro rotations agree more closely with the visual models than the raw gyro did. Independent IMU translation still fails validation.

This follows the earlier [motion-recovery results](motion-recovery-results.md). The earlier homography and RGB-D measurements remain separate experiments.

## Multiview reconstruction

I track the same feature sites through successive windows, apply the temporal residual filter, build consistent landmark tracks, triangulate structure and register additional cameras with PnP. I then jointly refine camera poses and 3D points with robust bundle adjustment. I fix the first camera and one translation coordinate in the seed camera to remove the world-frame and scale freedoms. Neither depth nor IMU enters that camera/structure fit.

After fitting, I estimate one scale from registered depth at the initial camera only. All six scale fits pass the initial-depth dispersion check, but this calibration does not make the trajectory an independent metric reference.

| Video | Original sampled views placed | Reconstruction views placed | Training landmarks | Reserved-track median error | Reserved-track 90th percentile |
|---|---:|---:|---:|---:|---:|
| 20261119_073137.db3 | 37/37 | 37/37 | 3320 | 0.967 px | 4.484 px |
| 20261119_073608.db3 | 34/34 | 34/34 | 3032 | 0.903 px | 4.243 px |
| 20261119_073711.db3 | 29/29 | 29/29 | 2586 | 1.017 px | 4.335 px |
| 20261119_074041.db3 | 34/34 | 34/34 | 3006 | 1.252 px | 4.949 px |
| 20261119_074125.db3 | 20/20 | 20/20 | 1800 | 1.086 px | 3.323 px |
| 20261119_074200.db3 | 19/19 | 49/49 | 4409 | 0.662 px | 1.698 px |

I placed 173/173 of the original sampled views. Including the extra fast-video keyframes, I placed 203/203 views. I rendered a separate registered-depth map from each final SfM result, including all selected views. The maps retain measured sampling holes and have the explicit pose source `image_only_multiview_SfM_initial_depth_scale`.

For the reserved-track check, I exclude one fifth of feature tracks from the seed fit, PnP and bundle adjustment. I triangulate each reserved track using its first view and earliest adequate baseline, then score its later observations with fixed camera poses. Correspondence filtering occurs before this split, some tracks can be spatially correlated, and the camera fit uses the full video. The errors measure image consistency rather than unseen-scene generalization or absolute camera accuracy. The several-pixel tails remain relevant even when the medians look small.

## Fast-video recovery

With ordinary 25-frame feature support, the fast video places 13/19 original views. Some frames admit zero supported SIFT sites. I retry unsupported ranges with added five-frame keyframes and five-frame feature support. Carried flow sites still pass the temporal filter over each actual keyframe interval. The shorter support is a different setting, so it is not an ablation showing that the 25-frame filter solved the same motion.

The final retry places all 49 reconstruction views and all 19 original views. The first dense retry stopped at 43/49 because its 3,600-landmark cap had filled. I now scale the cap with the requested camera count; the fast video uses a cap of 4,900 and retains 4,409 fitted landmarks. Increasing that capacity connected the remaining views without lowering the PnP, triangulation or temporal-filter thresholds.

The new visual reconstruction supplies SfM estimates at all five original frames omitted from the earlier RGB-D map: 175, 187, 200, 225 and 237. The new map uses those explicitly labelled SfM poses. It does not relabel them as successful RGB-D tracking. The interval is visibly blurred and has much weaker feature support, so connected poses still need the reserved-track checks and an independent metric reference.

## Refined timing and later rotation checks

I widened the gyro clock-shift grid to -200 through 200 milliseconds, then searched the winning prefix region at 2.5-millisecond steps. Bias and timing are fitted only on the first 20% of each video. Every selected shift is now inside the grid, between -47.5 and -45 milliseconds, where gyro query time equals camera time plus the shift. These values are model-fit hypotheses rather than measured hardware latency or a claim of 2.5-millisecond accuracy.

| Video | Prefix windows | Selected query shift | Raw gyro / RGB-D later median | Refined gyro / RGB-D later median |
|---|---:|---:|---:|---:|
| 20261119_073137.db3 | 6 | -47.5 ms | 1.335 degrees | 0.340 degrees |
| 20261119_073608.db3 | 5 | -47.5 ms | 1.031 degrees | 0.328 degrees |
| 20261119_073711.db3 | 4 | -45 ms | 1.006 degrees | 0.283 degrees |
| 20261119_074041.db3 | 5 | -45 ms | 0.995 degrees | 0.280 degrees |
| 20261119_074125.db3 | 2 | -45 ms | 1.214 degrees | 0.380 degrees |
| 20261119_074200.db3 | 2 | -45 ms | 1.375 degrees | 0.272 degrees |

Finer prefix fitting does not improve every later median relative to the older coarse -50 ms result. I keep the prefix-selected hypothesis instead of selecting the clock from later errors.

I also compare later windows with the new SfM rotations and translations. The next table uses the covered windows that both methods can evaluate; its window counts can differ from the timing table. SfM uses no IMU, but shares image content with RGB-D and obtains scale from initial depth.

| Video | Gyro / SfM windows | Gyro / SfM median rotation | RGB-D / SfM windows | RGB-D / SfM median rotation | RGB-D / SfM median translation-vector difference |
|---|---:|---:|---:|---:|---:|
| 20261119_073137.db3 | 28 | 0.185 degrees | 28 | 0.256 degrees | 3.84 mm |
| 20261119_073608.db3 | 26 | 0.161 degrees | 26 | 0.240 degrees | 4.48 mm |
| 20261119_073711.db3 | 22 | 0.148 degrees | 22 | 0.173 degrees | 3.54 mm |
| 20261119_074041.db3 | 26 | 0.152 degrees | 26 | 0.122 degrees | 3.04 mm |
| 20261119_074125.db3 | 15 | 0.233 degrees | 15 | 0.278 degrees | 6.24 mm |
| 20261119_074200.db3 | 14 | 0.233 degrees | 9 | 0.082 degrees | 1.11 mm |

The small per-window differences are model agreement, not millimetre-accurate trajectories. Chaining can still accumulate drift, the scene is largely planar, and the grass can deform. Independent IMU translation is not included as a successful channel in this table.

## Constrained inertial initialization

I tested a nonlinear prefix fit for initial velocity, accelerometer bias and gravity direction. I assume gravity magnitude 9.80665 m/s^2, use a 0.25 m/s^2 accelerometer-bias prior and apply the gyro-derived clock shift to both IMU streams. Those are explicit modelling assumptions rather than separately measured calibration. Initial velocity is estimated rather than forced to zero.

I check the data-only Jacobian separately from the bias prior, so regularization cannot hide missing observability. The later camera positions only score propagation; they do not adjust the fitted parameters. The reference is the RGB-D trajectory, including its loop corrections, rather than metric ground truth.

| Video | Data rank / unknowns | Constrained later position RMSE | Constant-velocity later RMSE | Initialization usable |
|---|---:|---:|---:|---|
| 20261119_073137.db3 | 8/8 | 8.76 m | 2.46 m | no |
| 20261119_073608.db3 | 8/8 | 18.30 m | 4.21 m | no |
| 20261119_073711.db3 | 8/8 | 6.57 m | 3.60 m | no |
| 20261119_074041.db3 | 8/8 | 14.18 m | 4.05 m | no |
| 20261119_074125.db3 | 6/8 | 3.09 m | 4.22 m | no |
| 20261119_074200.db3 | 6/8 | 2.81 m | 2.30 m | no |

All six fits remain rejected. The four full-rank fits still predict later positions worse than the simple baseline. The final two have only six data dimensions for eight unknowns; even an improvement on one position score does not identify a reliable solution. I do not apply these rejected estimates to the maps. This fit is not a complete optimized VIO estimator.

## Reproduce and verify

```bash
python -m pip install -r requirements-dev.txt
python scripts/run_motion_recovery.py --input-dir recordings --output-dir local-results/recovery
python scripts/run_multiview_validation.py --input-dir recordings --recovery-dir local-results/recovery --output-dir local-results/multiview --adaptive-gaps
python -m pytest -q tests
```

The multiview runner saves stage logs, input/source fingerprints, numerical-library versions, poses, structure, comparison tables and separate SfM maps. The recording identity uses its resolved path, size and modification time. A repeat reused all 31 stages, including the adaptive retry. I ran 78 regression tests successfully. I also checked that the saved structure is finite and every training observation has positive camera depth across all six reconstructions. The recordings, detailed outputs and map images stay outside the repository.

I still need independent metric motion measurements, labelled wind/camera-motion conditions and a reliable inertial trajectory before claiming all three channels agree. A slower repeat of the difficult interval, with overlap and sharper images, would provide a useful additional control.

Implementation references: [COLMAP SfM workflow](https://colmap.github.io/tutorial), [SciPy robust sparse least squares](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.least_squares.html), and [OpenVINS initialization](https://docs.openvins.com/classov__init_1_1InertialInitializer.html). My implementation is a separate small reconstruction and diagnostic pipeline; it does not run COLMAP or OpenVINS.
