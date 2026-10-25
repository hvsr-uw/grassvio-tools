# Stitch tuning and loop-closure check

Follow-up to the SIFT vs ORB comparison. Two questions from the October 23 meeting, answered with runs, not guesses.

## Question 1: does tuning the descriptor and RANSAC parameters fix anything?

The sweep varies the Lowe ratio (0.6, 0.75, 0.85) and the RANSAC threshold (1, 3, 5, 8 px) for both detectors, on the September take (24 pairs) and on the phone photos (6 pairs). The script's own default is ratio 0.75 and RANSAC 3 px, and the sweep reproduces the script's numbers exactly at that setting, so the sweep is measuring the same thing the script does.

September take, SIFT, median inliers: ratio 0.6 gives 1508 to 1519, ratio 0.75 gives 1676 to 1693, ratio 0.85 gives 1741 to 1765. No pair is skipped at any setting.

September take, ORB, median inliers: ratio 0.6 gives 731 to 1109, ratio 0.75 gives 861 to 1412, ratio 0.85 gives 915 to 1612. No pair is skipped at any setting. The tight RANSAC threshold of 1 px costs ORB the most (861 against 1382 at the default).

Phone photos, SIFT, pairs skipped: ratio 0.6 skips 4 of 6, ratio 0.75 skips 2 of 6 (the script's result), ratio 0.85 skips 1 of 6. The rescued pair is weak (median inliers drop from 67 to 53), so loosening the ratio buys one more pair, not a good map.

Phone photos, ORB, pairs skipped: ratio 0.6 skips 6 of 6, ratio 0.75 skips 5 of 6 (the script's result), ratio 0.85 still skips 5 of 6. No setting in this grid makes ORB work on the phone photos.

Reading: tuning moves the inlier counts, and a looser ratio plus a RANSAC threshold of 3 px or more is the better corner for both detectors. But tuning does not rescue a hard input. The phone photos fail for a structural reason (3D scene, big rotation, weak overlap), not a parameter reason.

## Question 2: does checking older frames find a loop closure?

The loop check matches every phone photo against every earlier photo with the script's settings (ratio 0.75, RANSAC 3 px, minimum 20 inliers). SIFT inlier counts, frame j against earlier frame i:

- Frame 2 against 1: 0
- Frame 3 against 2: 102, against 1: 0
- Frame 4 against 3: 11, against the rest: 0
- Frame 5 against 4: 21, against the rest: 0
- Frame 6 against 5: 32, against the rest: 0
- Frame 7 against 6: 135, against 1: 96, against the rest: 0

Two things follow. First, the chain pairs in this table reproduce the script's phone result exactly (2 skipped, median 67), so the check is consistent with the stitch.

Second, the meeting's loop idea is confirmed, but the exact pair is different from the example given in the meeting. Frame 6 against frame 1 finds nothing (0 inliers). Frame 7 against frame 1 finds 96 inliers. So the loop closes at the last photo, not the sixth, and the current script throws that 96-inlier constraint away because it only checks neighbours. Adding a check of each frame against all previous frames would give the map a real correction signal on this set.

With ORB, no non-neighbour pair finds anything. The only working ORB pair in the whole set is frame 3 against frame 2 (29 inliers). So the loop fix helps SIFT here; it does not rescue ORB.

## ORB-SLAM3 baseline: status

Not run yet, and this is stated plainly rather than implied. ORB-SLAM3 is not installed on this machine. The September bag does carry what a stock RGB-D-inertial run needs: 124 color frames, 124 depth frames, 828 gyro messages, and 417 accel messages, plus camera info strings. One correction to the earlier note: the bag has one color stream plus depth, not a stereo pair, so the fitting stock mode is RGB-D-inertial, not stereo-inertial. What a run needs before it can happen: build ORB-SLAM3 with its dependencies, convert the ROS 2 bag into the file format ORB-SLAM3 reads (timestamped RGB, depth, and IMU files plus a calibration file), and then run it. That is a separate job, not a quick command.

## What remains

- The loop check as a real feature in the stitch script (match against all previous frames, use the extra matches to correct drift), not just this experiment.
- The ORB-SLAM3 build and bag conversion above.
- The wind separation test, still blocked on data: no take has wind in it.
