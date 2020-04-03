#include <iostream>
#include <string>
#include <vector>
#include <stdint.h>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "tracker.h"

#include <boost/format.hpp>

#include <opengv/relative_pose/methods.hpp>
#include "opengv/relative_pose/CentralRelativeAdapter.hpp"
#include "opengv/relative_pose/NoncentralRelativeAdapter.hpp"

using namespace std;
using namespace cv;

/*
 * Test estimates egomotion using openGV using 2d-2d correspondences
*/


/* something taken form libviso */
int main (int argc, char** argv) {
    if (argc<2) {
        cerr << "Usage: ./viso2 path/to/sequence/2010_03_09_drive_0019" << endl;
        return 1;
    }

    string dir = argv[1];

    // calibration parameters for sequence 2010_03_09_drive_0019
    double f  = 645.24; // focal length in pixels
    double cu = 635.96; // principal point (u-coordinate) in pixels
    double cv = 194.13; // principal point (v-coordinate) in pixels
    double base     = 0.5707; // baseline in meters
    Matcher::parameters params;
    params.cu = cu;
    params.cv = cv;
    params.base = base;
    params.f = f;
    Tracker tracker(f, cu, cv, params);

    Mat img_l;
    Mat img_r;

    for (int32_t i=0; i<20; i++) {
       char base_name[256]; sprintf(base_name,"%06d.png",i);
       string left_img_file_name  = dir + "/I1_" + base_name;
       string right_img_file_name = dir + "/I2_" + base_name;
       img_l = imread(left_img_file_name);
       img_r = imread(right_img_file_name);

       tracker.push_back(img_l, img_r);

       /* building opengv's adapter: */
       opengv::bearingVectors_t a1;
       opengv::bearingVectors_t a2;
       vector<RegularFrame::StablePoint> &points1 = tracker.current->points;
       vector<RegularFrame::StablePoint> &points2 = tracker.previous->points;

       for (int i = 0; i < points1.size(); i++) {
           a1.push_back(points1[i]);
           int index2 = points1[i].buffer.at(points1[i].buffer.size() - 1);
           a2.push_back(points2[index2]);
       }

       vector<int> camCorrsp0;
       vector<int> camCorrsp1;
       for (int i = 0; i < a1.size(); i++) {
           camCorrsp0.push_back(0);
           camCorrsp1.push_back(1);
       }

       vector<Eigen::Vector3d> camOffsets(2);
       camOffsets[0] = Eigen::Vector3d(0, 0, 0);
       camOffsets[1] = Eigen::Vector3d(0, base, 0);

       vector<Eigen::Matrix3d> camRotations(2);
       camRotations[0] << 1, 0, 0,
               0, 1, 0,
               0, 0, 1;

       camRotations[1] << 1, 0, 0,
               0, 1, 0,
               0, 0, 1;

       opengv::relative_pose::NoncentralRelativeAdapter adapter(a1 ,a2, camCorrsp0, camCorrsp1, camOffsets, camRotations);
        apa
    }

    return 0;
}
