#pragma once

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <iterator>

#include <rapidxml/rapidxml.hpp>
#include <rapidxml/rapidxml_utils.hpp>

using namespace std;

/* parse sequence parameters from xml */
/* kitti sequences -- odometry */

struct sequence_parameters
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

    sequence_parameters(string filename) {
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
