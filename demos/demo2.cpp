#include <iostream>
#include <string>
#include <vector>
#include <stdint.h>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "tracker.h"
#include "viso_stereo.h"

#include <boost/format.hpp>

using namespace std;
using namespace cv;

/* normalization test:
 * now we hold bearing vectors for StablePoint instead of image-point,
 * it is test that points normalized correctly and we can get image-coordinates for stabe points
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

    Mat img0_l = imread(dir + "/I1_000000.png");
    Mat img0_r = imread(dir + "/I2_000000.png");
    Mat img1_l = imread(dir + "/I1_000001.png");
    Mat img1_r = imread(dir + "/I2_000001.png");

    tracker.push_back(img0_l, img0_r);
    tracker.push_back(img1_l, img1_r);

    vector<RegularFrame::StablePoint> &points = tracker.previous->points;

    for (auto point : points) {
        Eigen::Vector3d p = point.p_left;
        double x = p[0] / p[2];
        x = x * f + cu;

        double y = p[1] / p[2];
        y = y * f + cv;

        circle(img0_l, Point(x, y), 5, Scalar(0, 255, 0), 2);
    }

    imwrite("result.png", img0_l);
    return 0;
}
