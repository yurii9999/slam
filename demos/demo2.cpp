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

    EgomotionEstimation::configuration c(
                params.inliers_determination_policy_,
                params.triangulation_policy_,
                params.using_nonlinear_optimization,
                params.ransac_threshold,
                params.ransac_max_iterations,
                params.final_threshold
                );
    c.bucketing_height = params.bucketing_size;
    c.bucketing_widht = params.bucketing_size;
    c.bucketing_amount = params.bucketing_amount;

    c.close_coeff = params.close_coeff;
    c.far_coeff = params.far_coeff;

    EgomotionEstimation ego(f, cu, cv, base, c);


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

        RegularFrame &current_frame = *tracker.current;
        if (i_frame  == input_params.first_frame) {
            i_frame++;

            write_pose(output, current_pose);

            continue;
        }

        RegularFrame &previous_frame = *tracker.previous;

        delta = ego.estimate_motion(current_frame, previous_frame);

        current_pose = current_pose * delta;
        write_pose(output, current_pose);

        cout << "EgomotionEstimation: i = " << i_frame << "\tAmount features: " << current_frame.additionals.size() << "\t Amount inliers: " << ego.inliers.size() << "\ncorrespondences: " << ego.selection.size() << "\tRansac inliers: " << ego.ransac.inliers_.size() << endl;

        if (params.output_images) {
            Mat res = somedrawing_A(img_l, img_r, ego, current_frame, previous_frame);
            imwrite(params.output_dir+ string(image_name), res);

//            Mat disp = somedrawing_B1disp(img_l, img_r, ego, current_frame, previous_frame);
//            imwrite(params.output_dir+ "/disp" + string(image_name), disp);

//            ego.determine_inliers_reprojection_error();
//            Mat reproj = somedrawing_B0reproj(img_l, img_r, ego, current_frame, previous_frame);
//            imwrite(params.output_dir+ "/reproj" + string(image_name), reproj);

//            ego.determine_inliers();
//            Mat angle = somedrawing_B2angle(img_l, img_r, ego, current_frame, previous_frame);
//            imwrite(params.output_dir+ "/angle" + string(image_name), angle);
        }

        i_frame++;
    } while(img_l.data && i_frame != input_params.amount_frames + input_params.first_frame);

    output.close();

    return 0;
}
