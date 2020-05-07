#include <iostream>
#include <string>
#include <vector>
#include <stdint.h>
#include <sstream>
#include <fstream>

#include <cxxopts.hpp>

#include <boost/format.hpp>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "tracker.h"
#include "bucketing.h"
#include "egomotionestimation.h"
#include "sophus/se3.hpp"

#include "additional/sequence_parameters.h"
#include "additional/egomotion_parameters.h"
#include "additional/factory.h"

using namespace std;
using namespace cv;

Scalar geterateColor(int i) {
    RNG rng(12345 + i * 10);
    Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
    return color;
}

Mat somedrawing_A(Mat &img_l, Mat &img_r, EgomotionEstimation &ego, RegularFrame &current_frame, RegularFrame &previous_frame, vector<int> selection) {
    Mat res_l, res_r;
    cv::cvtColor(img_l, res_l, cv::COLOR_GRAY2BGR);
    cv::cvtColor(img_r, res_r, cv::COLOR_GRAY2BGR);

    vector<Scalar> colors(current_frame.additionals.size(), Scalar(0, 0, 128));
    for (auto idx : ego.inliers)
        colors[idx] = Scalar(0, 128, 0);

    for (auto p : ego.selection)
        colors[p.index] = colors[p.index] * 2;


    for (auto idx : selection) {
        auto p = current_frame.additionals[idx];
        Vector2d l = current_frame.image_points_left[p.index];
        Vector2d r = current_frame.image_points_right[p.index];
        circle(res_l, Point(l[0], l[1]), 3, colors[p.index], 2);
        circle(res_r, Point(r[0], r[1]), 3, colors[p.index], 2);
    }

    for (auto idx : ego.ransac.inliers_) {
        if (idx < ego.selection.size()) {
            Vector2d l = current_frame.image_points_left[ego.selection[idx].index];
            circle(res_l, Point(l[0], l[1]), 3, Scalar(255, 0, 0), 2);
        }
        else {
            Vector2d r = current_frame.image_points_right[ego.selection[idx - ego.selection.size()].index];
            circle(res_r, Point(r[0], r[1]), 3, Scalar(255, 0, 0), 2);
        }
    }


    vconcat(res_l, res_r, res_l);
    return res_l;
}

void write_pose(ofstream &output, Sophus::SE3d m) {
    double *a = m.matrix3x4().data();
    /* data now is [r00, .. r22 t1 t2 t3], so need to do this: */
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++)
            output << a[j * 3 + i] << " ";
        output << a[9 + i] << " ";
    }

    output << endl;
}

/* something taken form libviso */
/* --parameters="../../something" --input="../../sequence" */

int main (int argc, char** argv) {
    cxxopts::Options options("Something", "try to use on kitti dataset");
    options.add_options()
            ("parameters", "Parameters such as ransac threshold, max iteretions etc", cxxopts::value<string>())
            ("input", "Input parameters such as calibration, path to sequence", cxxopts::value<string>())
            ;
    auto result = options.parse(argc, argv);

    egomotion_parameters params(result["parameters"].as<string>());
    sequence_parameters input_params(result["input"].as<string>());

    if (!params.is_ransac_threshold_defined())
        params.set_default_ransac_threshold(input_params.focal);

    params.print();
    input_params.print();

    Tracker tracker = Factory::get_with_params(input_params);
    EgomotionEstimation egomotion_estimation = Factory::get_with_params(params, input_params);

    vector<shared_ptr<RegularFrame>> frames;

    string left_img_dir = input_params.sequence_path + "/image_0";
    string right_img_dir = input_params.sequence_path + "/image_1";

    cv::Mat img_l;
    cv::Mat img_r;

    Sophus::SE3d delta;

    int i_frame = input_params.first_frame;

    ofstream output;
    SE3d current_pose;
    output.open("result_pose");
    int e_iter = 0;
    int e_getin = 0;
    int e_better = 0;
    int e_amount = 0;

    Bucketing bucketing(2, 50, 50, 1300, 300);


    do {
        char image_name[256]; sprintf(image_name,"/%06d.png",i_frame );
        string left_img_file_name  = left_img_dir + image_name;
        string right_img_file_name = right_img_dir + image_name;
        img_l = imread(left_img_file_name, CV_LOAD_IMAGE_GRAYSCALE);
        img_r = imread(right_img_file_name, CV_LOAD_IMAGE_GRAYSCALE);

        /* works strange..... */
        if (i_frame > input_params.first_frame + 1 && params.try_predict_motion) /* could make motion prediction */
            tracker.push_back(img_l, img_r, &delta);
        else
            tracker.push_back(img_l, img_r);

        frames.push_back(tracker.current);

        if (i_frame  == input_params.first_frame) {
            i_frame++;
            bucketing.set_frame_size(img_l.rows, img_l.cols);

            write_pose(output, current_pose);

            continue;
        }

        RegularFrame &current_frame = *tracker.current;
        RegularFrame &previous_frame = *tracker.previous;

        bucketing.apply_bucketing(current_frame);

//        delta = egomotion_estimation.estimate_motion(current_frame, previous_frame, bucketing.selection);

        current_pose = current_pose * delta;
        write_pose(output, current_pose);

        cout << "EgomotionEstimation: i = " << i_frame << "\tAmount features: " << current_frame.additionals.size() << "\t Amount inliers: " << egomotion_estimation.inliers.size() << "\ncorrespondences: " << egomotion_estimation.selection.size() << "\tRansac inliers: " << egomotion_estimation.ransac.inliers_.size() << endl;
        cout << "Iterations " << egomotion_estimation.ransac.iterations_ << endl;
        e_iter += egomotion_estimation.ransac.iterations_;
        ++e_amount;

        if (params.output_images) {
//            Mat res = somedrawing_A(img_l, img_r, egomotion_estimation, current_frame, previous_frame, bucketing.selection);
            Mat res;
            cv::cvtColor(img_l, res, cv::COLOR_GRAY2BGR);

            int num = 0;
            Scalar color = geterateColor(num);

            for (auto idx : bucketing.selection) {
                Vector2d l = current_frame.image_points_left[idx];
                circle(res, Point(l[0], l[1]), 3, color, 2);
            }


            imwrite(params.output_dir+ string(image_name), res);
        }

        i_frame++;
    } while(img_l.data && i_frame != input_params.amount_frames + input_params.first_frame);

    output.close();

    cout << endl << "Results: " << endl;
    cout << "Average convergence (iterations) : " << ((double) e_iter) / e_amount << endl;

    return 0;
}
