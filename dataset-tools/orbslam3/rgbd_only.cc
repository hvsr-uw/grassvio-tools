// Offline RGB-D-inertial driver for ORB-SLAM3.
//
// Reads a converted sequence directory produced by convert_bag.py:
//   rgb.txt    <timestamp_s> rgb/<timestamp>.png
//   depth.txt  <timestamp_s> depth/<timestamp>.png
//
// Feeds frames to System::RGBD via TrackRGBD, following the pattern
// of the stock rgbd_tum example. Runs headless (no viewer).
//
// Usage:
//   rgbd_inertial_offline <vocabulary> <settings.yaml> <seq_dir>
//       <output_traj_tum.txt>
//
// The settings file follows the stock RealSense_D435i.yaml layout with
// Camera1 intrinsics and RGBD.DepthMapFactor.

#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#include <System.h>

using namespace std;

void LoadImages(const string &strPath,
                vector<string> &vstrRGB, vector<string> &vstrD,
                vector<double> &vTimestamps) {
    ifstream fRGB((strPath + "/rgb.txt").c_str());
    ifstream fD((strPath + "/depth.txt").c_str());
    if (!fRGB.is_open() || !fD.is_open()) {
        cerr << "Could not open rgb.txt / depth.txt in " << strPath << endl;
        exit(1);
    }
    string line;
    while (getline(fRGB, line)) {
        if (line.empty() || line[0] == '#') continue;
        istringstream ss(line);
        double t; string path;
        ss >> t >> path;
        vTimestamps.push_back(t);
        vstrRGB.push_back(strPath + "/" + path);
    }
    while (getline(fD, line)) {
        if (line.empty() || line[0] == '#') continue;
        istringstream ss(line);
        double t; string path;
        ss >> t >> path;
        vstrD.push_back(strPath + "/" + path);
    }
    if (vstrRGB.size() != vstrD.size()) {
        cerr << "rgb/depth count mismatch: " << vstrRGB.size()
             << " vs " << vstrD.size() << endl;
        exit(1);
    }
}


int main(int argc, char **argv) {
    if (argc != 5) {
        cerr << "Usage: " << argv[0]
             << " <vocabulary> <settings.yaml> <seq_dir>"
             << " <output_traj_tum.txt>" << endl;
        return 1;
    }

    vector<string> vstrRGB, vstrD;
    vector<double> vTimestampsCam;
    LoadImages(argv[3], vstrRGB, vstrD, vTimestampsCam);
    int nImages = vstrRGB.size();


    cout << "Images: " << nImages << endl;

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD,
                           false, 0, argv[3]);
    float imageScale = SLAM.GetImageScale();

    vector<float> vTimesTrack(nImages);

    for (int ni = 0; ni < nImages; ni++) {
        cv::Mat imRGB = cv::imread(vstrRGB[ni], cv::IMREAD_UNCHANGED);
        cv::Mat imD = cv::imread(vstrD[ni], cv::IMREAD_UNCHANGED);
        double tframe = vTimestampsCam[ni];
        if (imRGB.empty() || imD.empty()) {
            cerr << "Failed to load image pair at index " << ni << endl;
            return 1;
        }
        if (imageScale != 1.f) {
            int width = imRGB.cols * imageScale;
            int height = imRGB.rows * imageScale;
            cv::resize(imRGB, imRGB, cv::Size(width, height));
            cv::resize(imD, imD, cv::Size(width, height));
        }


        auto t1 = chrono::steady_clock::now();
        SLAM.TrackRGBD(imRGB, imD, tframe);
        auto t2 = chrono::steady_clock::now();
        double ttrack = chrono::duration_cast<chrono::duration<double, milli>>(t2 - t1).count();
        vTimesTrack[ni] = ttrack;

        if (ni % 50 == 0)
            cout << "frame " << ni << "/" << nImages
                 << " t=" << tframe << " track_ms=" << ttrack
                 << endl;
    }

    SLAM.Shutdown();
    SLAM.SaveTrajectoryTUM(argv[4]);

    double tmean = 0;
    for (float t : vTimesTrack) tmean += t;
    tmean /= nImages;
    cout << "median tracking time is not computed; mean: " << tmean << " ms" << endl;
    cout << "Trajectory saved to " << argv[4] << endl;
    return 0;
}
