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

    double final_threshold = 0.01; // set it after defenition of polity of final inliers detection

    bool try_predict_motion = false;

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

        xml_node<> *mp = doc.first_node("motion_prediction");
        if (mp)
            try_predict_motion = stoi(mp->value());
    }

    void print() {
        cout << "Motion estimation parameters: " << endl;
        cout << "ransac_max_iterations: " << ransac_max_iterations << endl;
        cout << "ransac_threshold: " << ransac_threshold << endl;
        cout << "final_threshold: " << final_threshold << endl;

        cout << "try_predict_motion: " << try_predict_motion << endl;
    }
};
