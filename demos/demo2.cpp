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

vector<int> getInliers() {

}


void select1(vector<RegularFrame::StablePoint> &points, vector<RegularFrame::StablePoint*> &dst) {
    dst.resize(points.size());

    for (int i = 0; i < points.size(); i++) {
        dst[i] = &points[i];
    }

    sort(
                dst.begin(),
                dst.end(),
                [] (RegularFrame::StablePoint *a, RegularFrame::StablePoint *b) { return a->age > b->age; });

    dst.erase(dst.begin() + 100, dst.end());

}

/* returns indexs of points that should be considered to egomotion estimation */
vector<int> bucketing(int frame_widht, int frame_height, int widht, int height, int amount /* points per each bucket */, vector<RegularFrame::StablePoint> &points) {
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
    Tracker tracker(params);

    VisualOdometryStereo::parameters param;

    // calibration parameters for sequence 2010_03_09_drive_0019
    param.calib.f  = f; // focal length in pixels
    param.calib.cu = cu; // principal point (u-coordinate) in pixels
    param.calib.cv = cv; // principal point (v-coordinate) in pixels
    param.base     = base; // baseline in meters

    // init visual odometry
    VisualOdometryStereo viso(param);

    string output_dit = "out";

    cv::Mat img_l;
    cv::Mat img_r;

    int first_frame_index = 310;

    for (int32_t frame_index=first_frame_index; frame_index<345; frame_index++) {
        char base_name[256]; sprintf(base_name,"%06d.png",frame_index);
        string left_img_file_name  = dir + "/I1_" + base_name;
        string right_img_file_name = dir + "/I2_" + base_name;
        img_l = imread(left_img_file_name);
        img_r = imread(right_img_file_name);

        tracker.push_back(img_l, img_r);

        if (frame_index == first_frame_index)
            continue;

        vector<RegularFrame::StablePoint> &points = tracker.current->points;
        vector<RegularFrame::StablePoint*> selection;
        select1(points, selection);

        vector<Matcher::p_match> p_matches_libviso(selection.size());

        for (int i = 0; i < selection.size(); i++) {
            RegularFrame::StablePoint *curr = selection[i];
            int prev_index = curr->buffer->at(curr->buffer->size() - 2);
            RegularFrame::StablePoint &prev = tracker.previous->points[prev_index];

            p_matches_libviso[i] = Matcher::p_match(
                        prev.p_left[0], prev.p_left[1], 0,
                        prev.p_right[0], prev.p_right[1], 0,
                        curr->p_left[0], curr->p_left[1], 0,
                        curr->p_right[0], curr->p_right[1], 0
                    );
        }

        viso.estimateMotion(p_matches_libviso);
        vector<int> &inliers = viso.inliers;
        int inliers_index = 0;

        for (int i = 0; i < p_matches_libviso.size(); i++) {
            Scalar color = (0, 0, 255);
            double res = abs(viso.p_residual[4 * i + 0] * viso.p_residual[4 * i + 0]) + abs(viso.p_residual[4 * i + 1] * viso.p_residual[4 * i + 1]);

            if (inliers[inliers_index] == i) {
                inliers_index++;
                color = Scalar(0, 255, 0);
            }

            string t = boost::str(boost::format("%.1f") % res);
            Matcher::p_match &cur_match = p_matches_libviso[i];
            Point p(cur_match.u1c, cur_match.v1c);
            putText(img_l, t, p, FONT_HERSHEY_COMPLEX_SMALL, 0.6, color);

        }

        cout << "Matches: " << selection.size() << "\tInliers: " << inliers.size() << endl;

        viso.release_memory();

        string outname = output_dit + "/frame" + to_string(frame_index) + ".png";
        imwrite(outname, img_l);
    }

    return 0;

}

























































////       vector<int> selection;
////       for (int i = 0; i < tracker.current->points.size(); i++)
////           selection.push_back(i);

//       vector<Matcher::p_match> p_matches_libviso;
//       p_matches_libviso.reserve(selection.size());

//       sort(selection.begin(), selection.end());

//       for (int i = 0; i < selection.size(); i++) {
//           RegularFrame::StablePoint &curr = tracker.current->points[selection[i]];
//           int prev_index = curr.buffer->at(curr.buffer->size() - 2);
//           RegularFrame::StablePoint prev = tracker.previous->points[prev_index];

//           Matcher::p_match a(
//                        prev.p_left[0], prev.p_left[1], 0,
//                        prev.p_right[0], prev.p_right[1], 0,
//                        curr.p_left[0], curr.p_left[1], 0,
//                        curr.p_right[0], curr.p_right[1], 0
//                   );

//           p_matches_libviso.push_back(a);
//       }

//       vector<double> tr = viso.estimateMotion(p_matches_libviso);
////       vector<int32_t> &inliers = viso.inliers;

//       Scalar green = Scalar(0, 255, 0);
//       Scalar red = Scalar(0, 0, 255);

//       vector<int32_t> &inliers = viso.inliers;
//       int inliers_index = 0;
//       int selection_index = 0;

//       int amount = 0;

//       for (int j = 0; j < tracker.current->points.size(); j++) {
//           if (j == selection[selection_index]) {
//               selection_index++;
//           }
//           else
//               continue;

//           Scalar color;
//           double res;

//           if (inliers_index >= inliers.size())
//               color = red;
//           else
//               if (j == selection[inliers[inliers_index]]) {
//                   tracker.current->points[j].increaseReliability();
//                   color = green;
//                   inliers_index++;
//                   amount++;
//                   cout << "Inlier! ";
//               }
//               else
//                   color = red;

////           double res = abs(viso.p_residual[4 * selection_index + 0] * viso.p_residual[4 * selection_index + 0]) + abs(viso.p_residual[4 * selection_index + 1] * viso.p_residual[4 * selection_index + 1]);
//           res = abs(viso.p_residual[4 * selection_index + 0]) +
//                   abs(viso.p_residual[4 * selection_index + 1]) +
//                   abs(viso.p_residual[4 * selection_index + 2]) +
//                   abs(viso.p_residual[4 * selection_index + 3]);

//           cout << "res \t" << res << endl;

//           string t = boost::str(boost::format("%.1f") % res);

//           Point p(tracker.current->points[j].p_left[0], tracker.current->points[j].p_left[1]);

//           putText(img_l, t, p, FONT_HERSHEY_COMPLEX_SMALL, 0.6, color);
////           circle(img_l, p, 3, color, 2);
////           cout << "selection_index: " << selection_index << "\tresidual:: " << res << endl;
//       }

////       waitKey();
//       cout << "Matches: " << selection.size() << "\tInliers: " << inliers.size() << endl;
//       cout << "Amount of my inliers: " << amount << endl;
//       viso.release_memory();
