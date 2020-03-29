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

/*
        max_features  = 2;
        bucket_width  = 50;
        bucket_height = 50;
*/

/* returns indexs of points that should be considered to egomotion estimation */
vector<int> bucketing(int frame_widht, int frame_height, int widht, int height, int amount /* points per each bucket */, vector<RegularFrame::StablePoint> points) {
    vector<int> result;
    int rows = ceil((float)frame_height / height);
    int collums = ceil((float)frame_widht / widht);
    vector<vector<int>> buckets(rows * collums);

    sort(
                points.begin(),
                points.end(),
                [] (RegularFrame::StablePoint a, RegularFrame::StablePoint b) { return a.age > b.age; }
    );

    for (int i = 0; i < points.size(); i++) {
        RegularFrame::StablePoint &cur_point = points[i];
        int row = (int)(cur_point.p_left[0] / widht);
        int collum = (int)(cur_point.p_left[1] / height);

        int index = collum * collums + row;
        if (buckets[index].size() > amount)
            continue;

        buckets[index].push_back(i);
    }

    for (auto bucket = buckets.begin(); bucket != buckets.end(); bucket++) {
//        if (bucket->size() > amount) {
//            bucket->erase(bucket->begin() + amount, bucket->end());
//        }

        for (int j = 0; j < bucket->size(); j++) {
            int a = bucket->at(j);
            result.push_back(a);
        }
    }

    return result;
}

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
    Tracker tracker(f, cu, cv, params);

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

    RegularFrame &last_frame = *frames.back();

//    vector<int> result = bucketing(img_l.size[1], img_l.size[0], 50, 50, 2, last_frame.points);
    vector<int> result;
    for (int i = 0; i < last_frame.points.size(); i++) {
        result.push_back(i);
    }
    cout << result.size() << endl;

    for (int i = 0 ; i < result.size(); i++) {
        RegularFrame::StablePoint &p = last_frame.points[result[i]];

        if (p.age < 5)
            continue;

        Scalar color = getColor(p.age + p.index1 + p.index2); /* emulating random */
        circle(img_l, Point(p.p_left[0], p.p_left[1]), 3, color, 2);
        circle(img_r, Point(p.p_right[0], p.p_right[1]), 3, color, 2);

        circular_buffer<int> &buff = *(p.buffer);
        for (int i = 1; i < p.age; i++) {
            int index = *(buff.end() - 1 - i);
            RegularFrame::StablePoint previous_observation = (*(frames.end() - 1 - i))->points[index];
            circle(img_l, Point(previous_observation.p_left[0], previous_observation.p_left[1]), 3, color);
            circle(img_r, Point(previous_observation.p_right[0], previous_observation.p_right[1]), 3, color);
        }

    }
//       for (vector<RegularFrame::StablePoint>::iterator it = last_frame.points.begin();
//         it < last_frame.points.end();
//         it++) {

////        if (it->age < 5)
////            continue;

//        Scalar color = getColor(it->age + it->index1 + it->index2); /* emulating random */
//        circle(img_l, Point(it->p_left[0], it->p_left[1]), 3, color);
//        circle(img_r, Point(it->p_right[0], it->p_right[1]), 3, color);

        /* draw point(u, v)*/
//        circular_buffer<int> &buff = *(it->buffer);

//        for (int i = 1; i < it->age; i++) {
//            int index = *(buff.end() - 1 - i);
//            RegularFrame::StablePoint previous_observation = (*(frames.end() - 1 - i))->points[index];
//            circle(img_l, Point(previous_observation.p_left[0], previous_observation.p_left[1]), 3, color);
//            circle(img_r, Point(previous_observation.p_right[0], previous_observation.p_right[1]), 3, color);
//        }
//    }

    vconcat(img_l, img_r, img_l);
    imwrite("resultllllll.png", img_l);

    return 0;
}
