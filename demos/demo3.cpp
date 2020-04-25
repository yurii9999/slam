#include "additional/sequence_parameters.h"
#include <cxxopts.hpp>

#include "matcher.h"
#include "tracker.h"
#include "segmentation.h"

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace cv;
using namespace std;

void draw(Mat &img, Vector2d feature, Scalar color) {
    Point a(feature[0], feature[1]);
    circle(img, a, 3, color, 2);
}

int main (int argc, char** argv) {
    cxxopts::Options options("Something", "try to use on kitti dataset");
    options.add_options()
            ("input", "Input parameters such as calibration, path to sequence", cxxopts::value<string>());
    auto result = options.parse(argc, argv);

    sequence_parameters input_params(result["input"].as<string>());
    input_params.print();

    double f  = input_params.focal;
    double cu = input_params.cu;
    double cv = input_params.cv;
    double base = input_params.base;

    Matcher::parameters params_matcher;
    params_matcher.cu = cu;
    params_matcher.cv = cv;
    params_matcher.base = base;
    params_matcher.f = f;
    params_matcher.refinement = 1;
    params_matcher.multi_stage = 0;

    Tracker tracker(f, cu, cv, params_matcher); /* tracker does not need f, cu cv; only matcher need */

    Segmentation segmentation(f, cu, cv, base);

    vector<shared_ptr<RegularFrame>> frames;

    int i_frame = input_params.first_frame;
    Mat img_l, img_r;
    do {
        char image_name[256]; sprintf(image_name,"/%06d.png",i_frame );
        string left_img_file_name  = input_params.sequence_path + "/image_0" + image_name;
        string right_img_file_name = input_params.sequence_path + "/image_1" + image_name;
        img_l = imread(left_img_file_name, CV_LOAD_IMAGE_GRAYSCALE);
        img_r = imread(right_img_file_name, CV_LOAD_IMAGE_GRAYSCALE);

        tracker.push_back(img_l, img_r);

        frames.push_back(tracker.current);

        RegularFrame &current_frame = *tracker.current;
        if (i_frame  == input_params.first_frame) {
            i_frame++;
            continue;
        }
        RegularFrame &previous_frame = *tracker.previous;

        segmentation.exec(current_frame);

        double max_norm = segmentation.derivatives[0].norm();
        double min_norm = segmentation.derivatives[0].norm();

        for (auto p : segmentation.derivatives) {
            double cur_norm = p.norm();
            if (cur_norm > max_norm)
                max_norm = cur_norm;

            if (cur_norm < min_norm)
                min_norm = cur_norm;
        }


        // Draw points according to the derivatives:
        // smoth gradient from green(no motion) to red(big motion)

        Scalar green(0, 255, 0);
        Scalar red(0, 0, 255);
        vector<Scalar> colors(segmentation.derivatives.size());
        for (int i = 0; i < segmentation.derivatives.size(); i++) {
            double cf = (max_norm - segmentation.derivatives[i].norm()) / max_norm;
            colors[i] = cf * green + (1 - cf) * red;
        }

        Mat res;
        cv::cvtColor(img_l, res, cv::COLOR_GRAY2BGR);

        for (int i = 0; i < segmentation.derivatives.size(); i++) {
            draw(res, current_frame.image_points_left[i], colors[i]);
        }
        imwrite("destt"+ string(image_name), res);


        i_frame++;
    } while(img_l.data && i_frame != input_params.amount_frames + input_params.first_frame);
}
