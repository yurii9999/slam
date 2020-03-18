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
    Tracker tracker(params);

    vector<shared_ptr<RegularFrame>> frames;

    cv::Mat img_l;
    cv::Mat img_r;
    for (int32_t i=0; i<20; i++) {
       char base_name[256]; sprintf(base_name,"%06d.png",i);
       string left_img_file_name  = dir + "/I1_" + base_name;
       string right_img_file_name = dir + "/I2_" + base_name;
       img_l = imread(left_img_file_name);
       img_r = imread(right_img_file_name);

       tracker.push_back(img_l, img_r);
       frames.push_back(tracker.current);
    }

    cout << frames.size();
    RegularFrame &last_frame = *frames.back();
    int amount = 0;
    for (vector<RegularFrame::StablePoint>::iterator it = last_frame.points.begin();
         it < last_frame.points.end();
         it++) {

        if (it->age < 5)
            continue;

        Scalar color = getColor(it->age + it->index1 + it->index2); /* emulating random */
        circle(img_l, Point(it->p_left[0], it->p_left[1]), 3, color);
        circle(img_r, Point(it->p_right[0], it->p_right[1]), 3, color);

        /* draw point(u, v)*/
        circular_buffer<int> &buff = *(it->buffer);

        for (int i = 1; i < it->age; i++) {
            int index = *(buff.end() - 1 - i);
            RegularFrame::StablePoint previous_observation = (*(frames.end() - 1 - i))->points[index];
            circle(img_l, Point(previous_observation.p_left[0], previous_observation.p_left[1]), 3, color);
            circle(img_r, Point(previous_observation.p_right[0], previous_observation.p_right[1]), 3, color);
        }
    }

    hconcat(img_l, img_r, img_l);
    imwrite("resultllllll.png", img_l);

    return 0;
}
