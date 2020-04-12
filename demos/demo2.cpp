#include <iostream>
#include <string>
#include <vector>
#include <stdint.h>
#include <sstream>

#include "cxxopts.hpp"

#include <rapidxml/rapidxml.hpp>
#include <rapidxml/rapidxml_utils.hpp>

#include <fstream>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "tracker.h"
#include "viso_stereo.h"

#include <boost/format.hpp>

#include "egomotionestimation.h"

using namespace std;
using namespace rapidxml;
using namespace cv;

struct parameters
{
    int ransac_max_iterations = 200;
    double ransac_threshold = 0.; // set it after calibration parameters parse
    bool is_ransac_threshold_defined() { return !(ransac_threshold == 0.); }
    void set_default_ransac_threshold(double focal) { ransac_threshold = 1.0 - cos(atan(sqrt(2.0)*0.5/focal)); }

    double final_threshold = 0.0; // set it after defenition of polity of final inliers detection
    const double ft_reprojection_error_default = 1.;
    const double ft_angel_between_rays_default = 0.01;

    double far_coeff = 10;
    double close_coeff = 40;

    int bucketing_amount = 2;
    int bucketing_size = 50;

    EgomotionEstimation::configuration::inliers_determination_policy inliers_determination_policy_ = EgomotionEstimation::configuration::inliers_determination_policy::REPROJECTION_ERROR;
    EgomotionEstimation::configuration::triangulation_policy triangulation_policy_ = EgomotionEstimation::configuration::triangulation_policy::FROM_PREVIOUS_FRAME;

    bool using_nonlinear_optimization = true;

    bool try_predict_motion = false;

    bool output_images = false;
    string output_dir = "";

    parameters(string filename) {
        rapidxml::file<> xmlFile(filename.c_str());
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());

        xml_node<> *rmi_n = doc.first_node("ransac_max_iterations");
        if (rmi_n)
            ransac_max_iterations = stoi(rmi_n->value());

        xml_node<> *rth = doc.first_node("ransac_threshold");
        if (rth)
            ransac_threshold = stod(rth->value());

        xml_node<> *fc = doc.first_node("far_coeff");
        if (fc)
            far_coeff = stod(fc->value());

        xml_node<> *cc = doc.first_node("close_coeff");
        if (cc)
            close_coeff = stod(cc->value());

        xml_node<> *idp = doc.first_node("inliers_determination_policy");
        if (idp) {
            string val(idp->value());
            if (val == "REPROJECTION_ERROR" || val == "0") {
                inliers_determination_policy_ = EgomotionEstimation::configuration::inliers_determination_policy::REPROJECTION_ERROR;
                final_threshold = ft_reprojection_error_default;
            } else if(val == "ANGEL_BETWEEN_RAYS" || val == "1") {
                inliers_determination_policy_ = EgomotionEstimation::configuration::inliers_determination_policy::ANGEL_BETWEEN_RAYS;
                final_threshold = ft_angel_between_rays_default;
            }


        xml_node<> *fth = doc.first_node("final_threshold");
        if (fth)
            final_threshold = stod(fth->value());

        xml_node<> *tp = doc.first_node("triangulation_policy");
        if (tp) {
            string val(tp->value());
            if (val == "FROM_PREVIOUS_FRAME" || val == "0")
                triangulation_policy_ = EgomotionEstimation::configuration::triangulation_policy::FROM_PREVIOUS_FRAME;
            else if (val == "WITH_BEST_DISPARITY" || val == "1")
                triangulation_policy_ = EgomotionEstimation::configuration::triangulation_policy::WITH_BEST_DISPARITY;
        }

        }

        xml_node<> *nlo = doc.first_node("using_nonlinear_optimization");
        if (nlo)
            using_nonlinear_optimization = stoi(nlo->value());

        xml_node<> *mp = doc.first_node("motion_prediction");
        if (mp)
            try_predict_motion = stoi(mp->value());

        xml_node<> *bs = doc.first_node("bucketing_size");
        if (bs)
            bucketing_size = stoi(bs->value());

        xml_node<> *ba = doc.first_node("bucketing_amount");
        if (ba)
            bucketing_amount = stoi(ba->value());

        rapidxml::xml_node<> *output_images_node = doc.first_node("save_images");
        if (output_images_node) {
            output_images = stoi(output_images_node->value());

            if (output_images) {
                rapidxml::xml_node<> *dir = doc.first_node("save_dir");
                if (dir)
                    output_dir = dir->value();
                else
                    output_dir = "newdir";

            }
        }

    }

    void print() {
        cout << "Run with config: " << endl;
        cout << "ransac_max_iterations: " << ransac_max_iterations << endl;
        cout << "ransac_threshold: " << ransac_threshold << endl;
        cout << "final_threshold: " << final_threshold << endl;
        cout << "far_coeff: " << far_coeff << endl;
        cout << "close_coeff: " << close_coeff << endl;

        string inliers_determination_policy_str [] = {"ANGEL_BETWEEN_RAYS", "REPROJECTION_ERROR"};
        cout << "inliers_determination_policy: " << inliers_determination_policy_str[inliers_determination_policy_] << endl;

        string triangulation_policy_str [] = {"FROM_PREVIOUS_FRAME", "WITH_BEST_DISPARITY"};
        cout << "triangulation_policy: " << triangulation_policy_str[triangulation_policy_] << endl;

        cout << "using_nonlinear_optimization: " << using_nonlinear_optimization << endl;
        cout << "try_predict_motion: " << try_predict_motion << endl;

        cout << "Bucketing amount features per cell: " << bucketing_amount << endl;
        cout << "Bucketing cell size: " << bucketing_size << endl;

        cout << "==========" << endl;
        if (output_images)
            cout << "save images in: " << output_dir << endl;
    }
};

struct input_parameters
{
    string sequence_path = "";
    string calib_filename = "";
    double focal = 0.;
    double cu =0.;
    double cv = 0.;
    double base = 0.; // meters

    int amount_frames = -1;

    int first_frame = 0;

    vector<double> parse_calib_params(string line) {
        line.erase(line.begin(), line.begin() + 4); /* "P0: " */

        std::istringstream iss(line);

        return std::vector<double>(
                    std::istream_iterator<double>(iss),
                    std::istream_iterator<double>()
                    );
    }

    input_parameters(string filename) {
        rapidxml::file<> xmlFile(filename.c_str());
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());

        sequence_path = doc.first_node("path")->value();
        rapidxml::xml_node<> *calibration = doc.first_node("calibration");
        if (calibration)
            calib_filename = calibration->value();
        else
            calib_filename = sequence_path + "/calib.txt";

        rapidxml::xml_node<> *amount_frames_n = doc.first_node("amount_frames");
        if (amount_frames_n)
            amount_frames = stoi(amount_frames_n->value());

        rapidxml::xml_node<> *ff = doc.first_node("first_frame");
        if (ff)
            first_frame = stoi(ff->value());

        /* parse calibration parameters */
        ifstream input;
        input.open(calib_filename);

        string line;;
        getline(input, line);
        vector<double> c1 = parse_calib_params(line);
        getline(input, line);
        vector<double> c2 = parse_calib_params(line);

        input.close();

        focal = c1[0];
        cu = c1[2];
        cv = c1[4 + 2];

        base = -c2[3] / focal; /* hmmm */
    }

    void print() {
        cout << "Sequence parameters: " << endl;
        cout << "path: " << sequence_path << endl;
        cout << "first frame: " << first_frame << endl;
        if (amount_frames > 0) cout << "first " << amount_frames << " frames" << endl;
        cout << "calibration file path: " << calib_filename << endl;
        cout << "Calibrations: " << endl;
        cout << "f = " << focal << endl;
        cout << "cu = " << cu << endl;
        cout << "cv = " << cv << endl;
        cout << "baseline = " << base << endl;
    }
};

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


/* something taken form libviso */
/* --parameters="../../something" --input="../../sequence" */

int main (int argc, char** argv) {
    cxxopts::Options options("Something", "try to use on kitti dataset");
    options.add_options()
            ("parameters", "Parameters such as ransac threshold, max iteretions etc", cxxopts::value<string>())
            ("input", "Input parameters such as calibration, path to sequence", cxxopts::value<string>())
            ;
    auto result = options.parse(argc, argv);

    parameters params(result["parameters"].as<string>());
    input_parameters input_params(result["input"].as<string>());

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

            continue;
        }

        RegularFrame &previous_frame = *tracker.previous;

        delta = ego.estimate_motion(current_frame, previous_frame);

        cout << "EgomotionEstimation: i = " << i_frame << "\tAmount features: " << current_frame.additionals.size() << "\t Amount inliers: " << ego.inliers.size() << "\ncorrespondences: " << ego.selection.size() << "\tRansac inliers: " << ego.ransac.inliers_.size() << endl;

        if (params.output_images) {
//            Mat res = somedrawing_A(img_l, img_r, ego, current_frame, previous_frame);
//            imwrite(params.output_dir+ string(image_name), res);

            Mat disp = somedrawing_B1disp(img_l, img_r, ego, current_frame, previous_frame);
            imwrite(params.output_dir+ "/residual" + string(image_name), disp);

            ego.determine_inliers_reprojection_error();
            Mat reproj = somedrawing_B0reproj(img_l, img_r, ego, current_frame, previous_frame);
            imwrite(params.output_dir+ "/reproj" + string(image_name), reproj);

            ego.determine_inliers();
            Mat angle = somedrawing_B2angle(img_l, img_r, ego, current_frame, previous_frame);
            imwrite(params.output_dir+ "/angle" + string(image_name), angle);
        }

        i_frame++;
    } while(img_l.data && i_frame != input_params.amount_frames + input_params.first_frame);

    ofstream output;
    output.open("result_pose");
    for (int i = 0; i < frames.size(); i++) {
        double *a = frames[i]->motion.matrix3x4().data();
        /* data now is [r00, .. r22 t1 t2 t3], so need to do this: */
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++)
                output << a[i * 3 + j] << " ";
            output << a[9 + i] << " ";
        }

        output << endl;

    }

    output.close();

    return 0;
}
