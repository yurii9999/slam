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
#include "egomotionestimation.h"
#include "../../external/include/sophus/se3.hpp"

#include "additional/sequence_parameters.h"
#include "additional/egomotion_parameters.h"
#include "additional/factory.h"

using namespace std;
using namespace cv;

Mat somedrawing_A(Mat &img_l, Mat &img_r, EgomotionEstimation &ego, RegularFrame &current_frame, RegularFrame &previous_frame) {
    Mat res_l, res_r;
    cv::cvtColor(img_l, res_l, cv::COLOR_GRAY2BGR);
    cv::cvtColor(img_r, res_r, cv::COLOR_GRAY2BGR);

    vector<Scalar> colors(current_frame.additionals.size(), Scalar(0, 0, 128));
    for (auto idx : ego.inliers)
        colors[idx] = Scalar(0, 128, 0);

    for (auto p : ego.selection)
        colors[p.index] = colors[p.index] * 2;


    for (auto p : current_frame.additionals) {
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

Mat somedrawing_B0reproj(Mat &img_l, Mat &img_r, EgomotionEstimation &ego, RegularFrame &current_frame, RegularFrame &previous_frame) {
    int resize_coeff_u = 4;
    int resize_coeff_v = 4;
    Mat res_l, res_r;
    cv::cvtColor(img_l, res_l, cv::COLOR_GRAY2BGR);
    cv::cvtColor(img_r, res_r, cv::COLOR_GRAY2BGR);

    resize(res_l, res_l, Size(res_l.cols * resize_coeff_u, res_l.rows * resize_coeff_v));
    resize(res_r, res_r, Size(res_r.cols * resize_coeff_u, res_r.rows * resize_coeff_v));

    Scalar color(255, 0, 0);

    for (auto p : current_frame.additionals) {
        Vector2d l = current_frame.image_points_left[p.index];
        Vector2d r = current_frame.image_points_right[p.index];

        stringstream ss1;
        ss1.setf(ios::fixed);
        ss1.precision(1);
        ss1 << ego.residual_l[p.index];

        stringstream ss2;
        ss2.setf(ios::fixed);
        ss2.precision(1);
        ss2 << ego.residual_r[p.index];

        putText(res_l, ss1.str(), Point2f(l[0] * resize_coeff_u,l[1] * resize_coeff_v), FONT_HERSHEY_PLAIN, 0.8,  color);
        circle(res_l, Point2f(l[0] * resize_coeff_u,l[1] * resize_coeff_v), 2, color);
        putText(res_r, ss2.str(), Point2f(r[0] * resize_coeff_u,r[1] * resize_coeff_v), FONT_HERSHEY_PLAIN, 0.8,  color);
        circle(res_r, Point2f(r[0] * resize_coeff_u,r[1] * resize_coeff_v), 2, color);
    }

    vconcat(res_l, res_r, res_l);
    return res_l;
}

Mat somedrawing_B1disp(Mat &img_l, Mat &img_r, EgomotionEstimation &ego, RegularFrame &current_frame, RegularFrame &previous_frame) {
    int resize_coeff_u = 4;
    int resize_coeff_v = 4;
    Mat res_l, res_r;
    cv::cvtColor(img_l, res_l, cv::COLOR_GRAY2BGR);
    cv::cvtColor(img_r, res_r, cv::COLOR_GRAY2BGR);

    resize(res_l, res_l, Size(res_l.cols * resize_coeff_u, res_l.rows * resize_coeff_v));
    resize(res_r, res_r, Size(res_r.cols * resize_coeff_u, res_r.rows * resize_coeff_v));

    Scalar color(0, 255, 0);

    for (auto p : current_frame.additionals) {
        Vector2d l = current_frame.image_points_left[p.index];
        Vector2d r = current_frame.image_points_right[p.index];

        stringstream ss1;
        ss1.setf(ios::fixed);
        ss1.precision(3);
        ss1 << ego.disparities[p.index];

        stringstream ss2;
        ss2.setf(ios::fixed);
        ss2.precision(1);
        ss2 << ego.disparities[p.index];

        putText(res_l, ss1.str(), Point2f(l[0] * resize_coeff_u,l[1] * resize_coeff_v), FONT_HERSHEY_PLAIN, 0.8,  color);
        circle(res_l, Point2f(l[0] * resize_coeff_u,l[1] * resize_coeff_v), 2, color);
        putText(res_r, ss2.str(), Point2f(r[0] * resize_coeff_u,r[1] * resize_coeff_v), FONT_HERSHEY_PLAIN, 0.8,  color);
        circle(res_r, Point2f(r[0] * resize_coeff_u,r[1] * resize_coeff_v), 2, color);
    }

    vconcat(res_l, res_r, res_l);
    return res_l;
}

Mat somedrawing_B2angle(Mat &img_l, Mat &img_r, EgomotionEstimation &ego, RegularFrame &current_frame, RegularFrame &previous_frame) {
    int resize_coeff_u = 4;
    int resize_coeff_v = 4;
    Mat res_l, res_r;
    cv::cvtColor(img_l, res_l, cv::COLOR_GRAY2BGR);
    cv::cvtColor(img_r, res_r, cv::COLOR_GRAY2BGR);

    resize(res_l, res_l, Size(res_l.cols * resize_coeff_u, res_l.rows * resize_coeff_v));
    resize(res_r, res_r, Size(res_r.cols * resize_coeff_u, res_r.rows * resize_coeff_v));

    Scalar color(0, 0, 255);

    for (auto p : current_frame.additionals) {
        Vector2d l = current_frame.image_points_left[p.index];
        Vector2d r = current_frame.image_points_right[p.index];

        stringstream ss1;
        ss1.setf(ios::fixed);
        ss1.precision(8);
        ss1 << ego.residual_l[p.index];

        stringstream ss2;
        ss2.setf(ios::fixed);
        ss2.precision(8);
        ss2 << ego.residual_r[p.index];

        putText(res_l, ss1.str(), Point2f(l[0] * resize_coeff_u,l[1] * resize_coeff_v), FONT_HERSHEY_PLAIN, 0.8,  color);
        circle(res_l, Point2f(l[0] * resize_coeff_u,l[1] * resize_coeff_v), 2, color);
        putText(res_r, ss2.str(), Point2f(r[0] * resize_coeff_u,r[1] * resize_coeff_v), FONT_HERSHEY_PLAIN, 0.8,  color);
        circle(res_r, Point2f(r[0] * resize_coeff_u,r[1] * resize_coeff_v), 2, color);
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

            write_pose(output, current_pose);

            continue;
        }

        RegularFrame &current_frame = *tracker.current;
        RegularFrame &previous_frame = *tracker.previous;

        delta = egomotion_estimation.estimate_motion(current_frame, previous_frame);

        current_pose = current_pose * delta;
        write_pose(output, current_pose);

        cout << "EgomotionEstimation: i = " << i_frame << "\tAmount features: " << current_frame.additionals.size() << "\t Amount inliers: " << egomotion_estimation.inliers.size() << "\ncorrespondences: " << egomotion_estimation.selection.size() << "\tRansac inliers: " << egomotion_estimation.ransac.inliers_.size() << endl;

        if (params.output_images) {
            Mat res = somedrawing_A(img_l, img_r, egomotion_estimation, current_frame, previous_frame);
            imwrite(params.output_dir+ string(image_name), res);
        }

        i_frame++;
    } while(img_l.data && i_frame != input_params.amount_frames + input_params.first_frame);

    output.close();

    return 0;
}
