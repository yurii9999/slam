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

void draw_component(Mat &res, vector<int> &segment, RegularFrame &current_frame, Scalar color = Scalar(0, 255, 0)) {
    for (auto idx_a : segment) {
        Point a(current_frame.image_points_left[idx_a][0], current_frame.image_points_left[idx_a][1]);
        for (auto idx_b : segment) {
            Point b(current_frame.image_points_left[idx_b][0], current_frame.image_points_left[idx_b][1]);
            line(res, a, b, color, 1);

        }
    }
}

void draw(Mat &img, Vector2d feature, Scalar color) {
    Point a(feature[0], feature[1]);
    circle(img, a, 3, color, 2);
}

Scalar generate_color(int num) {
    RNG rng(12345 + num * 10);
    Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
    return color;
}

void draw_graph(Mat &img, Segmentation segmentation, RegularFrame &current_frame) {
    int num = 213312;
    for (auto component : segmentation.components) {
        if (component.size() > 1) {
            Scalar color = generate_color(num);
            for (auto idx : component) {
                Point a(current_frame.image_points_left[idx][0], current_frame.image_points_left[idx][1]);
//                circle(img, a, 3, color, 2);

                for (auto idx2 : component) {
                    Point b(current_frame.image_points_left[idx2][0], current_frame.image_points_left[idx2][1]);
                    line(img, a, b, color, 1);
                }
            }

            num += 157;
        }
    }
    num = 213312;
    for (auto component : segmentation.components) {

        Scalar color = generate_color(num);
        for (auto idx : component) {
            Point a(current_frame.image_points_left[idx][0], current_frame.image_points_left[idx][1]);
            circle(img, a, 3, color, 2);

            num += 157;
        }

    }
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
            i_frame++;
            continue;
        }

        RegularFrame &current_frame = *tracker.current;
        RegularFrame &previous_frame = *tracker.previous;

        bucketing.apply_bucketing(current_frame);

        /* Se3d delta = */ egomotion.estimate_motion(current_frame, previous_frame, bucketing.selection);
        egomotion.determine_inliers();

        segmentation.exec(current_frame, bucketing.selection);

        Mat res;
        cv::cvtColor(img_l, res, cv::COLOR_GRAY2BGR);
//        draw_graph(res, segmentation, current_frame);

//        int num = 0;
//        for (auto segment : segmentation.components) {
//            Scalar color = generate_color(num);
//            for (auto idx_a : segment) {
//                for (auto idx_b : segment) {
//                    Vector2d l1 = current_frame.image_points_left[idx_a];
//                    Vector2d l2 = current_frame.image_points_left[idx_b];
//                    line(res, Point(l1[0], l1[1]), Point(l2[0], l2[1]), color, 1);
//                }
//            }

//            color = generate_color(num + 10);
//            for (auto idx_a : segment) {
//                Vector2d l2 = current_frame.image_points_left[idx_a];
//                circle(res, Point(l2[0], l2[1]), 5, color, 2);
//            }
//            ++num;
//        }


        vector<double> residuals(segmentation.components.size());
        for (int i = 0 ; i < segmentation.components.size(); i++ ) {
            vector<int> &segment = segmentation.components[i];
            if (segment.size() < 3) {
                residuals[i] = -1;
                continue;
            }

            double average_residual = 0.0;
            for (int idx : segment) {
                average_residual += egomotion.residual_l[idx] * egomotion.residual_l[idx];
            }

            average_residual /= segment.size();

            residuals[i] = average_residual;
        }

        vector<int> a;
        for (int i = 0 ; i < residuals.size(); i++) {
            a.push_back(i);
        }

        sort(a.begin(), a.end(), [&residuals] (int a, int b) { return residuals[a] > residuals[b]; });
        draw_component(res, segmentation.components[a[0]], current_frame, Scalar(255, 0, 0));

        imwrite("destt"+ string(image_name), res);



        i_frame++;
    } while(img_l.data && i_frame != sequence_params.amount_frames + sequence_params.first_frame);
}
