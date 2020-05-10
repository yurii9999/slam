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

Vector3d triangulate(double focal, double base, double cu, double cv, Vector2d left, Vector2d right) {
    double d = fmax(left[0] - right[0], 0.001);

    return Vector3d((left[0] - cu) * base / d, (left[1] - cv) * base / d, focal * base / d);
}

Scalar generate_color(int num) {
    RNG rng(12345 + num * 10);
    Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
    return color;
}

int main (int argc, char** argv) {
    cxxopts::Options options("Something", "try to use on kitti dataset");
    options.add_options()
            ("segmentation", "xml with segmentation parameters", cxxopts::value<string>())
            ("io", "Input parameters such as calibration, path to sequence", cxxopts::value<string>())
            ("egomotion", "Egomotion parameters", cxxopts::value<string>())
            ("bucketing", "bucketing parameters", cxxopts::value<string>());
    auto result = options.parse(argc, argv);

    segmentation_parameters segmentation_params(result["segmentation"].as<string>());
    io_parameters sequence_params(result["io"].as<string>());
    egomotion_parameters egomotion_params(result["egomotion"].as<string>());
    bucketing_parameters bucketing_params(result["bucketing"].as<string>());

    sequence_params.print();
    segmentation_params.print();
    egomotion_params.print();
    bucketing_params.print();

    Tracker tracker = Factory::get_with_params(sequence_params);
    Segmentation segmentation = Factory::get_with_params(segmentation_params, sequence_params);
    EgomotionEstimation egomotion = Factory::get_with_params(egomotion_params, sequence_params);
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

        RegularFrame &current_frame = *tracker.current;
        RegularFrame &previous_frame = *tracker.previous;

        egomotion.estimate_motion(current_frame, previous_frame);

        bucketing.apply_bucketing(current_frame);

        egomotion.determine_inliers(bucketing.selection);

        segmentation.exec(current_frame, bucketing.selection);

        Mat res(1000, img_l.cols, CV_8UC3, Scalar(0, 0, 0));
        Point center(500, 1000);

        Mat res2;
        cv::cvtColor(img_l, res2, cv::COLOR_GRAY2BGR);

        int i = 0;
        for (auto &segment : segmentation.components) {
            Scalar color = generate_color(i);
            ++i;
            for (int idx : segment) {
                RegularFrame::point_reference cur_observation = current_frame.additionals[idx].get_it_on(0);
                Vector3d cur_point = triangulate(sequence_params.focal, sequence_params.base, sequence_params.cu, sequence_params.cv, cur_observation.get_image_point_left(), cur_observation.get_image_point_right());

                int order = min(current_frame.additionals[idx].age + 1, 2); // order of finite difference
                vector<Vector3d> o(order); // observations


                for (int k = 0; k < order; k++) {
                    RegularFrame::point_reference prev_observation = current_frame.additionals[idx].get_it_on(k);
                    o[k] = triangulate(sequence_params.focal, sequence_params.base, sequence_params.cu, sequence_params.cv, prev_observation.get_image_point_left(), prev_observation.get_image_point_right());
                }

                Vector3d velocity;

                switch (order) {
                case 6:
//                    velocity = 137./60 * o[0] - 5 * o[1] + 5 * o[2] - 10./3 * o[3] + 5./4 * o[4] - 1./5 * o[5];
                    velocity = (o[0] - o[5]) / 5;
                    break;
                case 5:
//                    velocity = 25./12 * o[0] - 4 * o[1] + 3 * o[2] - 4./3 * o[3] + 1./4 * o[4];
                    velocity = (o[0] - o[4]) / 4;
                    break;
                case 4:
//                    velocity = 11./6 * o[0] - 3 * o[1] + 3./2 * o[2] - 1./3 * o[3];
                    velocity = (o[0] - o[3]) / 3;
                    break;
                case 3:
//                    velocity = 3./2 * o[0] - 2 * o[1] + 1./2 * o[2];
                    velocity = (o[0] - o[2]) / 2;
                    break;
                case 2:
                    velocity = o[0] - o[1];
                    break;
                }

                Vector3d prev_point = cur_point - velocity;


                Point a(cur_point[0] * 3, -cur_point[2]);
                a *= 10;
                Point b(prev_point[0] * 3, -prev_point[2]);
                b *= 10;

                line(res, a + center, b + center, color, 1);
                circle(res, a + center, 2, color, 2);

                Point f(cur_observation.get_image_point_left()[0], cur_observation.get_image_point_left()[1]);
                circle(res2, f, 2, color, 2);
            }
        }

        vconcat(res, res2, res);
        imwrite("destt"+ string(image_name), res);



        i_frame ++;
    } while(img_l.data && i_frame < sequence_params.amount_frames + sequence_params.first_frame);
}
