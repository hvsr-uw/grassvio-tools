# GrassVIO tools

Scripts I wrote for our CS 566 GrassVIO project. The folders match the layout the scripts expect, so the imports work as they are.

## stitch-map/stitch_map.py

It stitches all the frames in a video into one big map. It reads a bag file, a normal video file, or a folder of frames. Neighbouring frames are lined up with feature matches and a homography, then pasted onto one canvas.

Run it like this:

python stitch_map.py --bag <take>.db3 --out map.png

The --stride option uses every Nth frame and the default is 5. The --detector option picks sift or orb and the default is sift.

Tested on two takes. The September take gave a clean map of 672 by 507 pixels. The October 7 take gave a wider map of 2204 by 1891 pixels, but the far end stretches a bit because small errors add up over many frames.

## imu-integration-filtering/imu_fusion.py

The IMU motion analysis script. It reads the sensor data in a bag file, works out how far the camera moved between frame pairs from the accelerometer, and compares that with the distance the camera sees. It writes per_pair_distances.csv, accel_filtered.csv, and results.json. It needs the team features package on PYTHONPATH, same as when I run it here.

## bag-exploration/explore_bag.py

The shared bag reader both scripts use to read our .db3 recordings without a ROS install.
