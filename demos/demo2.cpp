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


    double farthest_coeff = 40; // points that have debth more than it will not be used in egomotion estimation; надо посчитать через disparity и тд чему он должен быть равен

    EgomotionEstimation::configuration::selection_policy selection_policy_ = EgomotionEstimation::configuration::selection_policy::SELECT_ALL;
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

        xml_node<> *sp = doc.first_node("selection_policy");
        if (sp) {
            string val(sp->value());
            if (val == "SELECT_ALL" || val == "0") {
                selection_policy_ = EgomotionEstimation::configuration::selection_policy::SELECT_ALL;
            }
            else {
                if (val == "SELECT_CLOSE" || val == "1") {
                    selection_policy_ = EgomotionEstimation::configuration::selection_policy::SELECT_CLOSE;
                }
                else if (val == "SELECT_FAR" || val == "2")
                    selection_policy_ = EgomotionEstimation::configuration::selection_policy::SELECT_FAR;

            }
        }

        xml_node<> *fc = doc.first_node("farthest_coeff");
        if (fc)
            farthest_coeff = stod(fc->value());

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
        cout << "farthest_coeff: " << farthest_coeff << endl;
        string selection_policy_str [] = {"SELECT_ALL" , "SELECT_CLOSE", "SELECT_FAR"};
        cout << "selection_policy: " << selection_policy_str[selection_policy_] << endl;

        string inliers_determination_policy_str [] = {"ANGEL_BETWEEN_RAYS", "REPROJECTION_ERROR"};
        cout << "inliers_determination_policy: " << inliers_determination_policy_str[inliers_determination_policy_] << endl;

        string triangulation_policy_str [] = {"FROM_PREVIOUS_FRAME", "WITH_BEST_DISPARITY"};
        cout << "triangulation_policy: " << triangulation_policy_str[triangulation_policy_] << endl;

        cout << "using_nonlinear_optimization: " << using_nonlinear_optimization << endl;
        cout << "try_predict_motion: " << try_predict_motion << endl;

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
        else
            amount_frames = -1;

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
        if (amount_frames > 0) cout << "first " << amount_frames << " frames" << endl;
        cout << "calibration file path: " << calib_filename << endl;
        cout << "Calibrations: " << endl;
        cout << "f = " << focal << endl;
        cout << "cu = " << cu << endl;
        cout << "cv = " << cv << endl;
        cout << "baseline = " << base << endl;
    }
};

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
    params_matcher.refinement = 2;

    Tracker tracker(f, cu, cv, params_matcher); /* tracker does not need f, cu cv; only matcher need */

    EgomotionEstimation::configuration c(
                params.selection_policy_,
                params.inliers_determination_policy_,
                params.triangulation_policy_,
                params.using_nonlinear_optimization,
                params.ransac_threshold,
                params.ransac_max_iterations,
                params.final_threshold,
                params.farthest_coeff
                );
    EgomotionEstimation ego(f, cu, cv, base, c);


    vector<shared_ptr<RegularFrame>> frames;

    string left_img_dir = input_params.sequence_path + "/image_0";
    string right_img_dir = input_params.sequence_path + "/image_1";

    cv::Mat img_l;
    cv::Mat img_r;

    Sophus::SE3d delta;

    vector<Sophus::SE3d> poses;

    int i_frame = 0;
    do {
        char image_name[256]; sprintf(image_name,"/%06d.png",i_frame );
        string left_img_file_name  = left_img_dir + image_name;
        string right_img_file_name = right_img_dir + image_name;
        img_l = imread(left_img_file_name, CV_LOAD_IMAGE_GRAYSCALE);
        img_r = imread(right_img_file_name, CV_LOAD_IMAGE_GRAYSCALE);

        /* works strange..... */
        if (i_frame > 1 && params.try_predict_motion) /* could make motion prediction */
            tracker.push_back(img_l, img_r, &delta);
        else
            tracker.push_back(img_l, img_r);

        frames.push_back(tracker.current);

        RegularFrame &current_frame = *tracker.current;
        if (i_frame  == 0) {
            /* initialize first frame on origin */
            SE3d origin;
            current_frame.set_motion(origin);
            i_frame++;

            continue;
        }

        RegularFrame &previous_frame = *tracker.previous;

        ego.estimate_motion(current_frame, previous_frame);
        cout << "EgomotionEstimation: i = " << i_frame << "\t correspondences: " << ego.selection.size() << "\t Inliers: " << ego.inliers.size() << endl;
        if (params.output_images) {
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

            vconcat(res_l, res_r, res_l);

            imwrite(params.output_dir + string(image_name), res_l);
        }

        delta = current_frame.motion * previous_frame.motion.inverse();

        poses.push_back(delta);

        i_frame++;
    } while(img_l.data && i_frame != input_params.amount_frames);

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
