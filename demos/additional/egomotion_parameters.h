#pragma once

#include <string>
#include <sstream>
#include <fstream>
#include <iterator>

#include <rapidxml/rapidxml.hpp>
#include <rapidxml/rapidxml_utils.hpp>

#include <egomotionestimation.h>

using namespace std;
using namespace rapidxml;

struct egomotion_parameters
{
    int ransac_max_iterations = 200;
    double ransac_threshold = 0.0; // set it after calibration parameters parse
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

    egomotion_parameters(string filename) {
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
