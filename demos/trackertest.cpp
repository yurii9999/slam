#include "additional/sequence_parameters.h"
#include "additional/segmentation_parameters.h"
#include "additional/egomotion_parameters.h"
#include "additional/factory.h"

#include <cxxopts.hpp>

#include "viso2/matcher.h"
#include "tracker.h"
#include "segmentation.h"
#include "bucketing.h"

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <chrono>
#include <ctime>

using namespace cv;
using namespace std;

void draw_points(Mat &img, RegularFrame &frame, vector<int> indices, Scalar color) {
    for (auto idx : indices) {
        Point a(frame.image_points_left[idx][0], frame.image_points_left[idx][1]);
        circle(img, a, 3, color, 2);
    }
}

Scalar generate_color(int num) {
    RNG rng(12345 + num * 10);
    Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
    return color;
}

int main (int argc, char** argv) {
    cxxopts::Options options("Something", "try to use on kitti dataset");
    options.add_options()
            ("io", "Input parameters such as calibration, path to sequence", cxxopts::value<string>())
            ("bucketing", "bucketing parameters", cxxopts::value<string>());
    auto result = options.parse(argc, argv);

    io_parameters sequence_params(result["io"].as<string>());
    bucketing_parameters bucketing_params(result["bucketing"].as<string>());

    sequence_params.print();
    bucketing_params.print();

    Tracker tracker = Factory::get_with_params(sequence_params);
    Bucketing bucketing = Factory::get_with_params(bucketing_params);

    vector<shared_ptr<RegularFrame>> frames;

    int i_frame = sequence_params.first_frame;
    Mat img_l, img_r;
    do {
        char image_name[256]; sprintf(image_name,"/%06d.png",i_frame );
        string left_img_file_name  = sequence_params.sequence_path + "/image_0" + image_name;
        string right_img_file_name = sequence_params.sequence_path + "/image_1" + image_name;
        img_l = imread(left_img_file_name, CV_LOAD_IMAGE_GRAYSCALE);
        img_r = imread(right_img_file_name, CV_LOAD_IMAGE_GRAYSCALE);

        tracker.push_back(img_l, img_r);
        frames.push_back(tracker.current);


        if (i_frame  == sequence_params.first_frame) {
            bucketing.set_frame_size(img_l.rows, img_l.cols);
            i_frame ++;
            continue;
        }

        i_frame ++;
    } while(img_l.data && i_frame < sequence_params.amount_frames + sequence_params.first_frame);

    Mat res;
    cv::cvtColor(img_l, res, cv::COLOR_GRAY2BGR);

    RegularFrame &current_frame = *tracker.current;
    bucketing.apply_bucketing(current_frame);

    for (int idx : bucketing.selection) {
        Scalar color = generate_color(idx);
        auto point = current_frame.additionals[idx];

        for (int i = 0; i < point.age; i++) {
            RegularFrame::point_reference &prev_observation = point.get_it_on(i);
            Point a(prev_observation.get_image_point_left()[0], prev_observation.get_image_point_left()[1]);
            circle(res, a, 3, color, 2);
        }
    }

    imwrite("tracker_features.png", res);
}
