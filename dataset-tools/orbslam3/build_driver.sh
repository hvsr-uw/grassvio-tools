#!/bin/bash
# Build the offline RGB-D-inertial driver against the ORB-SLAM3 library
# Run from this directory.
set -e
SLAM=$HOME/workspace/orbslam3-deps/ORB_SLAM3
PANG=$HOME/workspace/orbslam3-deps/install
OPENCV_LIBS=$(pkg-config --libs opencv4)
PANGO_LIBS="$PANG/lib/libpango_glgeometry.so.0.9.6 $PANG/lib/libpango_geometry.so.0.9.6 \
$PANG/lib/libpango_plot.so.0.9.6 $PANG/lib/libpango_python.so $PANG/lib/libpango_scene.so.0.9.6 \
$PANG/lib/libpango_tools.so.0.9.6 $PANG/lib/libpango_display.so.0.9.6 $PANG/lib/libpango_vars.so.0.9.6 \
$PANG/lib/libpango_video.so.0.9.6 $PANG/lib/libpango_packetstream.so.0.9.6 \
$PANG/lib/libpango_windowing.so.0.9.6 $PANG/lib/libpango_opengl.so.0.9.6 \
$PANG/lib/libpango_image.so.0.9.6 $PANG/lib/libpango_core.so.0.9.6"
g++ -std=c++14 -O2 -Wall \
  -I$SLAM -I$SLAM/include -I$SLAM/include/CameraModels \
  -I$SLAM/Thirdparty/Sophus -isystem /usr/include/eigen3 \
  -isystem /usr/include/opencv4 -isystem $PANG/include \
  -DCOMPILEDWITHC11 -DHAVE_EIGEN -DHAVE_EPOXY -D_LINUX_ \
  -DPANGO_DEFAULT_WIN_URI=\"x11\" \
  rgbd_inertial_offline.cc -o rgbd_inertial_offline \
  -Wl,-rpath,$SLAM/lib:$PANG/lib:$SLAM/Thirdparty/DBoW2/lib:$SLAM/Thirdparty/g2o/lib \
  $SLAM/lib/libORB_SLAM3.so $OPENCV_LIBS $PANGO_LIBS \
  -lrt -lEGL -lOpenGL -lepoxy $PANG/lib/libtinyobj.so.0.9.6 \
  $SLAM/Thirdparty/DBoW2/lib/libDBoW2.so $SLAM/Thirdparty/g2o/lib/libg2o.so \
  -lboost_serialization -lcrypto
echo "built rgbd_inertial_offline"
