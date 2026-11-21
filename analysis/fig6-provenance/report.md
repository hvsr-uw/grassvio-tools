# Original Fig. 6 provenance

I recovered the original midterm report and the archived project code locally so
the September comparison could be checked against the source artifacts. The raw
report and the full code archive stay outside this repository because they
contain the original project data and outputs.

## What the report establishes

The report's Section V-B says that the physical recordings had no per-frame
ground truth. It describes the IMU comparison as a cross-check against a visual
two-frame pose, and says that the IMU estimate was converted into the colour
camera frame before comparison.

The Fig. 6 caption gives the remaining plotting facts:

- recording: `20260926_155419`
- 53 consecutive frame pairs
- visual descriptors: ORB, SIFT, HOG and BFM
- visual fit: RANSAC threshold of 2 px
- left column: rotation angle in degrees
- right column: translation magnitude in metres

The surrounding text reports an IMU translation below 0.6 mm per pair, while
the visual estimates are 2--57 mm per pair depending on the descriptor. This is
an observation from the original report, not an independent accuracy claim,
because the same passage explicitly says that no per-frame camera ground truth
was available.

## What the recovered code establishes

The recovered `code` directory contains final SIFT stitching outputs, an RGB-D
PnP script, and 6-DoF transform outputs. It does not contain the script that
generated Fig. 6, the descriptor comparison implementation, or a file that
names the original IMU translation quantity. A search of the recovered text
and source files found no Fig. 6 plotting script and no ORB/SIFT/HOG/BFM
comparison trace for `20260926_155419`.

The repository's older `imu_fusion.py::imu_distance` diagnostic is therefore
not identified as the source of the original curves. That function resets
velocity for each interval and demeans acceleration by the video-wide mean,
which can create a small acceleration-only displacement, but the recovered
artifacts do not prove that it produced Fig. 6.

## What the matching simulation archive can and cannot validate

The recovered `sim_20260926_155419` archive contains 110 pose records: 55
source/target pairs with a fixed synthetic camera motion, plus RGB/depth frames,
timestamps, gyro samples and accelerometer samples. Its `simulation_ground_truth`
poses are useful visual ground truth for the synthesized image sequence.

The accelerometer file has 330 samples, and every sample is exactly
`[0, 0, -9.80665]` m/s^2. There are no non-gravity translational acceleration
samples. The archive therefore cannot independently validate the inertial
translation integrator: its visual pose ground truth and its IMU acceleration
stream do not describe a physically accelerating camera. The nonzero gyro
samples can support a rotation-only sanity check, but they do not repair the
missing translational reference.

## Conclusion

The source report now resolves the original Fig. 6 recording, pair count,
descriptors, RANSAC threshold, units, and frame-conversion convention. The
exact plotting implementation and the independent camera-motion reference
remain unavailable. The current validation correctly treats Fig. 6 as a
visual cross-check rather than proof of independent IMU translation accuracy.

The measured archived controls and their limitations are in
`analysis/archived-controls/report.md`.
