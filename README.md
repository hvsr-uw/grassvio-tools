# GrassVIO tools

Scripts I wrote for our CS 566 GrassVIO project. The folders match the layout the scripts expect, so the imports work as they are.

## stitch-map/stitch_map.py

It stitches all the frames in a video into one big map. It reads a bag file, a normal video file, or a folder of frames. Neighbouring frames are lined up with feature matches and a homography, then pasted onto one canvas.

Run it like this:

python stitch_map.py --bag <take>.db3 --out map.png

The --stride option uses every Nth frame and the default is 5. The --detector option picks sift or orb and the default is sift. The --loop-check option also matches each frame against earlier frames at least 5 frames back, and a strong loop match re-anchors the frame instead of the drifted chain.

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
