#include <iostream>
#include <string>
#include <vector>
#include <stdint.h>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "tracker.h"

using namespace std;
using namespace cv;

Scalar getColor(int seed) {
    return Scalar(
                seed * 2 % 255,
                seed * 3 % 255,
                seed * 5 % 255
                );
}

Point vectorToCv(Eigen::Vector2d v) {
    return Point(v[0], v[1]);
}

/* something taken form libviso */
int main (int argc, char** argv) {
    // we need the path name to 2010_03_09_drive_0019 as input argument
    if (argc<2) {
        cerr << "Usage: ./viso2 path/to/sequence/2010_03_09_drive_0019" << endl;
        return 1;
    }

    // sequence directory
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

    vector<shared_ptr<RegularFrame>> frames;

    cv::Mat img_l;
    cv::Mat img_r;
    for (int32_t i=0; i<200; i++) {
       char base_name[256]; sprintf(base_name,"%06d.png",i);
       string left_img_file_name  = dir + "/I1_" + base_name;
       string right_img_file_name = dir + "/I2_" + base_name;
       img_l = imread(left_img_file_name);
       img_r = imread(right_img_file_name);

       tracker.push_back(img_l, img_r);
       frames.push_back(tracker.current);
    }

    RegularFrame &last_frame = *frames.back();

//    vector<int> result = bucketing(img_l.size[1], img_l.size[0], 50, 50, 2, last_frame.points);
    vector<int> result;
    for (int i = 0; i < last_frame.amount_points(); i++) {
        result.push_back(i);
    }
    cout << result.size() << endl;

    for (int i = 0; i < result.size(); i++) {
        RegularFrame::feature_additional p = last_frame.additionals[result[i]];

        if (p.age < 5)
            continue;

        Scalar color = getColor(p.age + p.index_left + p.index_right); /* emulating random */

        circle(img_l, vectorToCv(last_frame.image_points_left[p.index]), 3, color, 2);
        circle(img_r, vectorToCv(last_frame.image_points_right[p.index]), 3, color, 2);

        RegularFrame::PointCommon &common = *p.common;
        for (int i = 0 ; i < common.buffer.size(); i++) {
            RegularFrame::point_reference &ref = common.buffer[i];

            RegularFrame &f = *ref.frame;
            int index = ref.index;

            circle(img_l, vectorToCv(f.image_points_left[index]), 3, color);
            circle(img_r, vectorToCv(f.image_points_right[index]), 3, color);
        }
    }

    vconcat(img_l, img_r, img_l);
    imwrite("resultllllll.png", img_l);

    return 0;
}
