# ORB-SLAM3 RGB-D-inertial baseline on the GrassVIO rosbags

Date: 2026-11-06. Work area: `analysis-drafts/leftover-three/orbslam3/`
(under the CS 566 project). Source trees live outside the project in
`orbslam3-deps/` at the workspace root. Nothing in this task was
uploaded to GitHub.

## Short version

ORB-SLAM3 **RGB-D without IMU beats our RGB-D VO clearly on all three
takes** — smaller end error, and its rotation matches the gyro while
the VO's does not. But the **RGB-D-inertial mode does not work on
these takes**: it never completes initialization on the slow hand-held
motion, resets its map every few seconds, and only ever outputs poses
for the last seconds of a take (or segfaults on a near-static take).
The usable baseline from this work is RGB-D-only, not
RGB-D-inertial. The failure is in ORB-SLAM3's inertial
initialization gate, not in the data: the same frames track fine
without the IMU.

## What was built

- ORB-SLAM3, UZ-SLAMLab/ORB_SLAM3, commit
  `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4` (2022-02-10, "Update
  Dependencies.md").
- Pangolin, stevenlovegrove/Pangolin, commit
  `42963caffb354f8e0db127ed90ff5fa4396dd864` (2026-10-26), built from
  source (no apt package exists) and installed to a workspace prefix.
- Two small drivers written for this task (both in this directory):
  `rgbd_inertial_offline.cc` feeds converted frames plus IMU samples
  to `System::IMU_RGBD`; `rgbd_only.cc` is the same harness for
  `System::RGBD`. Both run headless (viewer off). ORB-SLAM3 ships no
  offline RGB-D-inertial example — only a live-camera D435i one that
  needs librealsense2 and a plugged-in camera — so the offline driver
  was necessary.
- Converter `convert_bag.py` turns a rosbag2 `.db3` into TUM-VI style
  files (`rgb.txt`, `depth.txt`, `imu.txt` plus `info.json`), reusing
  the CDR layout documented in the project's bag explorer
  (header without `seq`, 324-byte IMU blobs, string padding quirks).

## Build steps (exact commands)

System packages (Ubuntu 24.04, installed with apt):

```
sudo apt-get update
sudo apt-get install -y git libeigen3-dev libopencv-dev libglew-dev \
  libboost-serialization-dev libboost-system-dev libboost-filesystem-dev \
  libboost-thread-dev libssl-dev libgl1-mesa-dev libglu1-mesa-dev \
  libx11-dev libxext-dev libxi-dev libxrandr-dev libxinerama-dev \
  libxcursor-dev ffmpeg libavcodec-dev libavutil-dev libavformat-dev \
  libswscale-dev libjpeg-dev libpng-dev libtiff-dev \
  libepoxy-dev libwayland-dev libxkbcommon-dev
```

Versions installed: Eigen 3.4.0, OpenCV 4.6.0, GLEW 2.2.0,
Boost 1.83, Pangolin 0.9.6 (built). Note: `libepoxy-dev` is required
by Pangolin's configure step; it is not in the stock ORB-SLAM3
dependency list.

```
git clone --depth 1 https://github.com/stevenlovegrove/Pangolin.git
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd Pangolin && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=<deps>/install \
  -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF
make -j4 && make install
```

ORB-SLAM3 third-party libraries, then the main build (from the
ORB-SLAM3 checkout; `<deps>` is the `orbslam3-deps` directory):

```
cd Thirdparty/DBoW2/build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j4
cd ../../g2o/build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j4
cd ../../Vocabulary && tar --no-same-owner -xf ORBvoc.txt.tar.gz
cd ../build && cmake .. -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=<deps>/install
make -j1
```

Driver build (script `build_driver.sh` in this directory):

```
bash build_driver.sh   # produces ./rgbd_inertial_offline
bash build_rgbd_only.sh  # produces ./rgbd_only
```

### Patches to the stock tree (only `CMakeLists.txt` changed)

1. C++11 to C++14. Stock ORB-SLAM3 forces `-std=c++11`, but the
   Pangolin headers use C++14 aliases (`std::enable_if_t` in
   sigslot) and the build fails with `'enable_if_t' is not a member
   of 'std'`. Changed the flag to `-std=c++14`, keeping the
   `COMPILEDWITHC11` define the code relies on.
2. `Optimizer.cc` and `Tracking.cc` compile at `-O1` instead of
   `-O3` (appended per-file flag). Each of those translation units
   peaks near 2.6 GB of compiler memory at `-O3`, and this VM has
   8 GB RAM: the compiler was OOM-killed at `-j4`, `-j2`, and even
   `-j1` on `Optimizer.cc`. Adding swap was not possible (`swapon`:
   Operation not permitted). The rest of the library stays at `-O3`.
3. Sophus's own test target `test_sim2` fails under GCC 13
   (`-Werror=array-bounds` in Eigen headers). Sophus is header-only
   for ORB-SLAM3's purposes, so its tests were simply skipped; the
   library itself was never needed as a built artifact.

### Incidents during the build

- The first ORB-SLAM3 clone failed mid-download (`fatal: early EOF`,
  `fetch-pack: invalid index-pack output`); a retry succeeded.
- The VM restarted mid-task (uptime showed 2 minutes afterwards).
  All apt-installed packages vanished with the restart; everything
  under the workspace survived (Pangolin install, ORB-SLAM3 clone,
  converted data, staging files). The apt install was repeated with
  the same versions and the build redone. System packages on this VM
  are ephemeral; workspace files are not.
- The stock `rgbd_tum` example aborts headless with `Pangolin X11:
  Failed to open X display` because it enables the viewer. That is
  why the RGB-D-only control run uses the headless `rgbd_only`
  driver instead.

## Data: bag facts verified from the bags themselves

Camera: RealSense D455F, firmware 5.17.3.10, IMU BMI085. All four
topics share one clock starting at 0 and are monotonic. Header
stamps equal the rosbag record timestamps.

- Color: 640x480 RGB8, 30 Hz. Depth: 640x480 mono16, 30 Hz, units
  0.001 m. Gyro ~200 Hz, accel ~100 Hz.
- Depth intrinsics (from the bag's Depth camera_info, zero
  distortion): fx = fy = 386.009827, ppx = 320.77182,
  ppy = 240.889587.
- Color intrinsics differ slightly (fx = 386.040802,
  fy = 385.300934, ppx = 323.054626, ppy = 250.530975, with nonzero
  distortion), and the color camera sits 5.9 cm from the depth
  camera (translation -0.059037, 0.000078, 0.000128 m). The depth
  map is therefore **not** aligned to the color frame. Both our VO
  and this baseline use the depth map as-is with the depth
  intrinsics, so the comparison is fair; a depth-to-color warp
  remains an open improvement.
- IMU to depth-camera extrinsics (from the bag's Gyro tf topic):
  rotation identity, translation (-0.030220, 0.007400, 0.016020) m.
- IMU noise (BMI085 product flyer, typical): gyro 0.014 deg/s/sqrtHz
  = 2.44e-4 rad/s/sqrtHz; accel 120 ug/sqrtHz = 1.18e-3
  m/s^2/sqrtHz; gyro bias instability under 2 deg/h. Random walks
  are assumed, not measured: GyroWalk 1e-5, AccWalk 1e-4.
- A second settings file, `D455F-vins.yaml`, uses the stock
  RealSense values from ORB-SLAM3's own D435i example instead
  (NoiseGyro 1e-2, NoiseAcc 1e-1, GyroWalk 1e-6, AccWalk 1e-4).

Takes used (resolved by filename stamp; the two 085611 downloads
are byte-identical, md5 `0b7de10d7030202ada40f076ea4eac26`; the Oct 17
and Oct 27 downloads of 080033 and 075947 are also byte-identical
pairs, so the VO comparison is against the same recordings):

| Take | Bag (stamp) | Frames paired | Duration | Depth valid | Pair offset mean / max |
| --- | --- | --- | --- | --- | --- |
| 080033 | 20261007_080033 | 440 / 441 | 14.68 s | 91% | 6.4 ms / 9.3 ms |
| 075947 | 20261007_075947 | 235 / 235 | 7.82 s | 88% | 6.4 ms / 7.2 ms |
| 20261106_085611 | 20261106_085611 | 386 / 386 | 12.86 s | 73% | 6.3 ms / 7.0 ms |

Accel is interpolated linearly onto the gyro timestamps for the
combined `imu.txt` (gyro 200 Hz, accel 101 Hz). Depth frames pair
to color frames by nearest timestamp; pairs over 20 ms apart are
dropped (1 dropped on 080033, 0 on the others). IMU sanity: mean
|accel| is 9.73–9.75 m/s^2 on all three takes, as expected.

## Runs

All runs use `D455F.yaml` unless noted, with vocabulary
`ORBvoc.txt` (145 MB, from the repo). Example command:

```
./rgbd_inertial_offline <vocab> D455F.yaml data/080033 runs/080033_traj_tum.txt
./rgbd_only <vocab> D455F.yaml data/080033 runs/080033_rgbd_only_traj.txt
```

Mean tracking time: about 52 ms/frame RGB-D-only, about 59 ms/frame
RGB-D-inertial (offline, no real-time constraint).

### RGB-D-only (the working baseline)

Coverage is 100% on all three takes. End error is the distance and
rotation between the first and last pose (loop-closure error on a
take that returns to its start). The VO numbers are the reference
values from the project's earlier RGB-D VO (`analysis-drafts/rgbd/`
and `analysis-drafts/new-takes-20261106/rgbd/`); the VO is a
comparison point, not ground truth.

| Take | ORB-SLAM3 RGB-D end error | VO end error | ORB path length | VO path length |
| --- | --- | --- | --- | --- |
| 080033 | **0.061 m / 13.0 deg** | 0.836 m / 67.6 deg | 0.69 m | 0.99 m |
| 075947 | **0.188 m / 8.8 deg** | 0.580 m / 14.8 deg | 0.46 m | 0.65 m |
| 20261106_085611 | **0.0001 m / 0.01 deg** | 0.238 m / 86.3 deg | 0.04 m | 0.28 m |

No loop-closure event was logged in any RGB-D-only run (no "Loop
detected" / global-BA line), so these end errors come from the
odometry and local bundle adjustment alone, not from a loop fix.

### RGB-D-inertial (does not work on these takes)

| Take | Settings | Poses out / frames | Time span covered | End error within span |
| --- | --- | --- | --- | --- |
| 080033 | datasheet noise | 11 / 440 | 14.35–14.68 s (last 0.33 s) | 0.003 m |
| 080033 | VINS noise | 82 / 440 | 11.98–14.68 s (last 2.7 s) | 0.008 m |
| 075947 | datasheet noise | 69 / 235 | 5.55–7.82 s (last 2.3 s) | 0.033 m |
| 075947 | VINS noise | 27 / 235 | 6.95–7.82 s (last 0.9 s) | 0.015 m |
| 20261106_085611 | datasheet noise | 0 / 386 | none (segfault, see below) | — |

### Why the inertial mode fails

The visual part works: on 080033 the map initializes at frame 0
with about 1200 points and tracks. Then the local mapper resets
the whole map, about every 80 frames, with:

```
Not enough motion for initializing. Reseting...
TRACK: Reset map because local mapper set the bad imu flag
```

The gate is in `LocalMapping.cc`: once the map is IMU-initialized
but the second inertial bundle adjustment has not run yet, if the
accumulated "moving time" is under 10 s **and** the camera moved
under 2 cm across the last two keyframe steps, the map is reset.
Our takes are slow hand-held sweeps (about 5 cm/s with pauses), so
that gate fires repeatedly for the whole take; only near the end,
when enough moving time has accumulated, does a map survive. A
second, earlier gate in `StereoInitialization` also refuses to start
when the mean acceleration barely changes between frames ("not
enough acceleration": 11 times on 080033, 384 times on
20261106_085611). Looser VINS-mono noise values change the outcome
only slightly (11 to 82 poses on 080033; fewer on 075947), so the
noise parameters are not the cause — the motion itself is below what
ORB-SLAM3's initializer demands.

The 20261106_085611 take is nearly static (gyro max 0.019 rad/s,
accel std 0.02 m/s^2), so the map never initializes at all, and the
run ends in a segfault inside `SaveTrajectoryTUM` while saving an
empty trajectory (exit code 139). That is a stock ORB-SLAM3 edge
case with empty trajectories, not a driver bug: the log's last line
is the "Saving camera trajectory..." message.

## Independent check against the gyro

Total angular path length (sum of |gyro| over the take, versus the
sum of rotation steps along each trajectory) is an independent
check, since the gyro never enters the RGB-D-only runs:

| Take | Gyro | VO | ORB-SLAM3 RGB-D |
| --- | --- | --- | --- |
| 080033 | 190.3 deg | 165.9 deg | 183.8 deg |
| 075947 | 58.6 deg | 42.6 deg | 56.4 deg |
| 20261106_085611 | 4.4 deg | 86.6 deg | 4.1 deg |

The ORB-SLAM3 rotation matches the gyro on all three takes. The
VO's rotation is short on the two moving takes and badly wrong on
the near-static take (86.6 deg of drift where the gyro says 4.4
deg). That is the strongest evidence that the ORB-SLAM3 RGB-D
trajectories, not the VO ones, are the credible ones — including on
080033, where the VO's 0.82 m excursion is most likely drift from
a mostly-rotational pan (a known failure shape for VO without bundle
adjustment), while ORB-SLAM3's small loop with 184 deg of rotation
matches the gyro.

ATE of the ORB-SLAM3 RGB-D trajectory against the VO trajectory
after SE(3) alignment (for reference only, since the VO drifts):
0.20 m RMSE on 080033, 0.12 m on 075947, 0.07 m on 20261106_085611.

Plots (top view x-vs-z and side view x-vs-y, VO in blue,
ORB-SLAM3 in orange; both in the first-camera frame): 
`runs/080033_rgbd_only_overlay.png`,
`runs/075947_rgbd_only_overlay.png`,
`runs/20261106_085611_rgbd_only_overlay.png`. All three were
checked by eye: on 20261106_085611 the ORB-SLAM3 trajectory is a
dot at the origin while the VO wanders 0.24 m; on 075947 the two
agree on the outward leg and part ways on the return; on 080033
the ORB-SLAM3 loop is small and closed while the VO makes a large
excursion.

## Judgement

- **ORB-SLAM3 RGB-D-only beats our RGB-D VO on these takes, clearly.**
  End error improves from 0.836 to 0.061 m on the loop take 080033,
  from 0.580 to 0.188 m on 075947, and from 0.238 m / 86.3 deg to
  effectively zero on the near-static 20261106_085611 take. Its
  rotation agrees with the gyro on all three takes; the VO's does
  not. Use the RGB-D-only trajectories as the baseline going
  forward.
- **ORB-SLAM3 RGB-D-inertial does not beat anything here, because it
  never runs.** It is not a data problem (the frames, depth, and IMU
  are all sane, and the same frames track fully without the IMU) and
  not a settings problem (two noise settings tried). It is a motion
  problem: these slow hand-held takes do not excite ORB-SLAM3's
  inertial initializer, which resets the map until 10 s of
  sufficiently fast motion has accumulated. Any future inertial run
  on this data needs either faster, more varied motion in the take
  itself, or a different initialization path (for example a
  vision-first warm start, or the `mFastInit` path), before it can
  be compared fairly.
- Open follow-ups, not done here: warp depth into the color frame
  (5.9 cm baseline) and re-run to see if the end errors shrink
  further; check whether loop closure ever fires on a longer take;
  try the color-camera intrinsics in place of the depth intrinsics.

## Files in this directory

- `SUMMARY.md` — this file.
- `PROGRESS.md` — dated work log (build incidents included).
- `convert_bag.py` — rosbag2 `.db3` to TUM-VI style converter.
- `rgbd_inertial_offline.cc`, `build_driver.sh` — inertial driver.
- `rgbd_only.cc`, `build_rgbd_only.sh` — RGB-D-only driver.
- `D455F.yaml`, `D455F-vins.yaml` — settings (datasheet noise /
  stock VINS-mono noise).
- `compare.py` — trajectory comparison and overlay plots.
- `data/{080033,075947,20261106_085611}/` — converted sequences
  (`rgb/`, `depth/`, `rgb.txt`, `depth.txt`, `imu.txt`, `info.json`).
- `runs/` — trajectories (`*_traj_tum.txt`,
  `*_rgbd_only_traj.txt`), run logs, and overlay plots.
- `build.log`, `build2.log`, `build3.log`, `driver_build.log`,
  `rgbd_only_build.log` — full build logs.
