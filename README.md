# GrassVIO tools

Scripts I wrote for our CS 566 GrassVIO project. The main stitching entry points now share one implementation; older experiment folders also contain historical results.

## Homography and window-motion update

The current implementation and validation status are in
[analysis/meeting-followup.md](analysis/meeting-followup.md). Start there for
what changed and what still needs the original recordings. The related-work
check is in [research/motion-filter-related-work.md](research/motion-filter-related-work.md).

Install and check:

```bash
python -m pip install -r requirements-dev.txt
python -m pytest -q tests
python scripts/validate_meeting_work.py --outdir analysis/meeting-followup-validation
```

Normal descriptor stitching, with source frames beside the warped map:

```bash
python stitch-map/stitch_map.py --video input.mp4 --out map.png \
  --blend best --debug-out debug.png
```

Window filtering on a recording independently known to be stationary:

```bash
python stitch-map/stitch_map.py --bag stationary.db3 --out filtered.png \
  --motion-filter --flow-window 25 --camera-still --debug-out debug.png
```

For a moving camera, replace `--camera-still` with
`--camera-motion motion.json` containing full pixel motion, including
translation. The schema is in the follow-up. Omitting both uses a visual
plane proxy and cannot independently distinguish wind from camera motion.
The historical `dataset-tools/stitch-map/stitch_map.py` entry point calls
this same implementation.

SfM keyframe conditioning (undistorted images, matching input calibration):

```bash
python stitch-map/sfm_keyframes.py --video input.mp4 \
  --calibration camera.json --out keyframes.json --min-parallax-deg 2
```

`camera.json` contains `{"K": [[fx,0,cx],[0,fy,cy],[0,0,1]]}` with numeric
values for the input image resolution. This selects candidate pairs; it
does not run a full SfM reconstruction or recover metric scale.

## stitch-map/stitch_map.py

It stitches all the frames in a video into one big map. It reads a bag file, a normal video file, or a folder of frames. Neighbouring frames are lined up with feature matches and a homography, then pasted onto one canvas.

Run it like this:

python stitch_map.py --bag <take>.db3 --out map.png

The --stride option uses every Nth frame and the default is 5. The --detector option picks sift or orb and the default is sift. The --loop-check option also matches each frame against earlier frames at least 5 frames back, and a strong loop match re-anchors the frame instead of the drifted chain. The --place tree option places each frame along the strongest spanning tree over all frame pairs instead of the neighbour chain, so a frame whose neighbour pair fails can still be placed from a frame that matches it well, and --lowe-ratio tunes the matcher.

Tested on two takes. The September take gave a clean map of 672 by 507 pixels. The October 7 take gave a wider map of 2204 by 1891 pixels, but the far end stretches a bit because small errors add up over many frames.

## stitch-map/detector-comparison/

The detector comparison on the same recordings, with SIFT and ORB maps and stats, the tuning sweep over the Lowe ratio and the RANSAC threshold, the loop check over all frame pairs, and the loop closure feature note. Tuning helps at the margins but does not rescue a hard input. The loop check found one real closure in the phone photos, the last photo against the first one with 96 inliers.

## imu-integration-filtering/imu_fusion.py

The IMU motion analysis script. It reads the sensor data in a bag file, works out how far the camera moved between frame pairs from the accelerometer, and compares that with the distance the camera sees. It writes per_pair_distances.csv, accel_filtered.csv, and results.json. It needs the team features package on PYTHONPATH, same as when I run it here.

## bag-exploration/explore_bag.py

The shared bag reader both scripts use to read our .db3 recordings without a ROS install.

## descriptor-benchmark/benchmark.py

Runs ORB and HOG over every consecutive frame pair in each take and scores them with a RANSAC homography fit, plus a per take IMU motion score from the gyro. Needs the team features package on PYTHONPATH and the folder of bag files with --bag-dir.

## friday-tasks/friday_tasks.py

Three small tasks: feature based distance between frames, RANSAC filtering with the inlier ratio, and IMU integration between frame timestamps. Same inputs as the benchmark.

## dataset-validation/

Validates a sequence folder against the dataset spec. grassvio/checks.py and grassvio/schema.py hold the checks and the schema, validate.py runs them, and make_test_sequence.py generates small test sequences for the validator.

## hog-work/

The HOG descriptor method for the shared feature interface, with Shi-Tomasi corners and a HOG window around each corner, plus its matching test. HOG has no keypoint detector of its own, so the corners come from goodFeaturesToTrack.

## stage4-motion/

The synthetic lawn video generator, the Lucas-Kanade visual odometry pipeline, and the persistent track statistics tool, with a one command script that runs the whole thing on the gentle and aggressive profiles.
