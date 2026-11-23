# 080033 SfM in a shared ground frame

This folder contains the RGB-only reconstruction and a ground-aligned version of the camera path and sparse point cloud. The new origin is the Frame 0 camera center projected onto the fitted grass plane. X follows the initial camera's forward direction projected onto that plane, Y completes a right-handed basis, and Z points up. All coordinates are in metres.

The transform is applied once to SfM, RGB-D ORB-SLAM3, and IMU orientations. I did not independently fit one trajectory to another, so the position residuals are in the same fixed frame. ORB-SLAM3 is a model reference, not ground truth. IMU translation is excluded because its integrated translation drifts; the comparison uses gyro-derived rotations only.

## Results

The 19-view reconstruction contains 1,709 sparse points. A deterministic RANSAC fit marks 698 points as within 8 mm of the grass plane. The inlier median distance is 3.66 mm and the 90th percentile is 7.25 mm. The fitted plane places the Frame 0 camera 0.767 m above ground.

Across the 19 registered views, the median camera-position difference from RGB-D is 6.1 mm (90th percentile 11.5 mm). The median absolute orientation difference is 0.34 degrees against RGB-D and 7.43 degrees against the integrated IMU orientation. Across the 18 consecutive SfM intervals, the median displacement residual is 2.60 mm; the median relative-rotation residual is 0.209 degrees against RGB-D and 0.838 degrees against IMU.

Metric scale comes from registered depth at frame 25. The existing held-out depth check covers 14 frames and 5,757 observations, with 1.63 cm median absolute depth error and 1.92% median absolute relative error. This checks metric scale against later depth observations. A direct visual check of depth-to-RGB remapping/calibration still needs synchronized RGB and depth frames.

The ground estimate uses the sparse SfM cloud, so it is a frame construction and consistency check, not independent ground truth. Points outside the RANSAC threshold are preserved in the exported cloud.

## Files

- `080033-reconstruction.json`: source SfM poses in the initial-camera frame.
- `080033-sparse-reconstruction.ply`: source sparse cloud.
- `080033-heldout-depth.json`: existing later-depth validation.
- `080033-ground-world-frame.svg`: both the transformed scene/path and an overhead path comparison.
- `080033-ground-world-pose-residuals.csv`: absolute same-frame position and orientation residuals.
- `080033-ground-world-interval-residuals.csv`: consecutive displacement and relative-rotation residuals.
- `080033-ground-world-summary.json`: transform, plane fit, and summary statistics.
- `080033-ground-world-reconstruction.json` and `080033-ground-world-sparse-reconstruction.ply`: transformed camera poses and points.

## Recreate the outputs

Run from the repository root with Python, NumPy, and the SfM input files:

```sh
python scripts/ground_frame_sfm.py \
  --sfm analysis/check-list-report/sfm/080033-reconstruction.json \
  --cloud analysis/check-list-report/sfm/080033-sparse-reconstruction.ply \
  --rgbd analysis/audit-imu-preint/runs/080033_orb_poses.json \
  --imu analysis/audit-imu-preint/runs/20261007_080033_preint.json \
  --outdir analysis/check-list-report/sfm
```
