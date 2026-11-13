check list results

1. methods

inlier rate: sift matching on stride 5 frame pairs, lowe ratio 0.75, ransac at 3.0 px, inliers over good matches. 14 takes, 505 pairs.

r and t accuracy: sim test with true motion known, 30 pairs on 155419. loop closing end error on the real takes, orbslam3 rgbd against our own vo.

loop closing test: end against start sift matching, 25 pairs per take, 20 inliers or more means a loop. rgbd paths end against start.

fig 6: imu against 4 descriptors on 155419, 53 frame pairs, midterm report section 5.

reliability check: each feature gets a score for how reliable it is, from its fit residual. 3 takes: 174745 still, 173701 wind signal, 080033 rotation.

2. results

2.1 inlier rate

every take clears 50 percent. overall median 0.987. lowest take 173250 at 0.698. only 1 pair out of 505 falls below 0.50.

one more thing: the normalized fit counted inliers wrong, the threshold was in the wrong units, so it just counted good matches. fixed now, and the normalized count matches the direct count.

![median inlier rate per take](fig1_inlier_rate.png)

fig 1. median inlier rate per take. the dashed line is the 50 percent bar.

2.2 r and t accuracy

sim test, 30 pairs on 155419, true motion known. sim numbers are from the oct 25 team check.

| descriptor | result |
|---|---|
| sift | stays on the true motion |
| bfm | close behind sift |
| orb | off by about 1 degree in rotation and up to 1 cm in translation |
| hog | no pose at all |

sensitivity, frame to frame jitter.

| descriptor | jitter |
|---|---|
| sift | about 0.3 mm |
| hog | about 1 mm |
| bfm | about 1 mm |
| orb | 5 to 8 mm |

2.3 loop closing test

genuine loops are 080033, 075947 and 155419. the weak takes and the oct 30 takes are not loops.

| take | orbslam3 | our vo | loop |
|---|---|---|---|
| 080033 | 0.061 m, 13.0 deg | 0.836 m, 67.6 deg | yes |
| 075947 | 0.188 m, 8.8 deg | 0.580 m, 14.8 deg | yes |
| 155419 | - | 0.108 m, 19.6 deg | yes |
| 085611 | 0.0001 m, 0.01 deg | 0.238 m, 86.3 deg | still take |
| 173807 | - | 4.926 m, 149.6 deg | no |
| 172532 | 0.81 m, 9.3 deg | 6.102 m, 101.0 deg | no |
| 172828 | 0.83 m | 5.394 m, 67.3 deg | no |
| 172859 | - | 0.838 m, 29.4 deg | nearly still |

the 174702 and 174818 paths end 5.1 m and 4.3 m from the start, and end against start matching finds 0 inliers on both. they are not loops.

![080033 orbslam3 pose map](fig2_loop_map.png)

fig 2. 080033, orbslam3 pose map. 6.67 by 4.44 m, end lands 0.060 m from start.

2.4 fig 6 translation comparison

rotation lines up well. sift follows the imu almost exactly. translation does not line up. the imu stays under 0.6 mm per pair. the visual numbers run 2 to 57 mm per pair.

the likely cause is the visual translation only gives a direction, not a real distance, and nothing in the figure says how it was lined up to the imu scale. the axes also change from row to row, so the rows are hard to compare.

the fig 6 graphic is in the midterm report, section 5, and is not in this folder.

2.5 reliability check

on 173701 only 47.2 percent of tracks score above 0.8, against 97.8 percent on the still control and 97.5 percent on the rotating take. the score falls from about 0.95 to about 0.3 halfway through the take, then moves between about 0.2 and 0.6. the signal stays while the camera is still, so it is not parallax. wind is the leading explanation, not proof.

![reliability over time on 173701](fig3_reliability_curve.png)

fig 3. mean reliability over time on 173701. it starts at control level and falls halfway through the take.

![reliability histogram on 173701](fig4_reliability_hist.png)

fig 4. reliability histogram on 173701. the tracks split into two groups, near 0 and near 1.

2.6 add list status

| item | status |
|---|---|
| optical flow wind check | done, works over a 25 frame window |
| imu preintegration | done, does not help. translation blows up to 80 m, gravity about 6 deg off |
| visual pose init | done, accel bias not pinned down, gyro bias part works |
| sfm | on hold |
| semantic cues | not started |

3. findings

- every take clears the 50 percent inlier bar, by a wide margin.
- sift stays on the true motion in sim. orb is off by about 1 deg and up to 1 cm. hog gives no pose.
- orbslam3 closes the loop on 080033 at 0.061 m and 13.0 deg, against 0.836 m and 67.6 deg for our own vo.
- the genuine loops are 080033, 075947 and 155419. the weak takes and the oct 30 takes are not loops.
- fig 6 rotation lines up, translation does not, and the visual translation has no real scale of its own.
- the reliability score separates the wind take from both controls, and finds when it goes unreliable.
- the imu does not fix translation on these takes. gravity is off by about 6 deg, and the accel bias is not pinned down by the data.

4. conclusion

the inlier rate clears the bar on every take, the loop test sorts the real loops from the rest, and the open gaps are fig 6 translation and imu translation.

appendix. inlier rate per take

| take | pairs | median | p10 | below 0.50 |
|---|---|---|---|---|
| 080033 | 40 | 1.000 | 0.975 | 0 |
| 075947 | 40 | 1.000 | 0.999 | 0 |
| 155419 | 10 | 0.962 | 0.853 | 0 |
| 172532 | 40 | 0.992 | 0.986 | 0 |
| 172828 | 40 | 0.930 | 0.779 | 0 |
| 172859 | 32 | 0.949 | 0.721 | 0 |
| 102631c | 24 | 0.999 | 0.997 | 0 |
| 174745 | 39 | 0.999 | 0.999 | 0 |
| 085611 | 40 | 0.999 | 0.998 | 0 |
| 173701 | 40 | 0.851 | 0.654 | 0 |
| 173807 | 40 | 0.843 | 0.670 | 1 |
| 172912 | 40 | 0.842 | 0.655 | 0 |
| 173250 | 40 | 0.698 | 0.562 | 0 |
| 173507 | 40 | 0.958 | 0.812 | 0 |
| all 505 pairs | 505 | 0.987 | 0.703 | 1 |
