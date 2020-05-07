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

struct io_parameters
{
    string sequence_path = "";
    string calib_filename = "";
    double focal = 0.;
    double cu =0.;
    double cv = 0.;
    double base = 0.; // meters

    int amount_frames = -1;

    int first_frame = 0;

    bool write_images = false;
    string output_dir = "newdir";

    vector<double> parse_calib_params(string line) {
        line.erase(line.begin(), line.begin() + 4); /* "P0: " */

        std::istringstream iss(line);

        return std::vector<double>(
                    std::istream_iterator<double>(iss),
                    std::istream_iterator<double>()
                    );
    }

    io_parameters(string filename) {
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

        rapidxml::xml_node<> *output_images_node = doc.first_node("save_images");
        if (output_images_node) {
            write_images = stoi(output_images_node->value());

            if (write_images) {
                rapidxml::xml_node<> *dir = doc.first_node("save_dir");
                if (dir)
                    output_dir = dir->value();
            }
        }

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

        cout << "======" << endl << "Output parameters: " << endl << "Write images: " << write_images << endl;
        if (write_images)
            cout << "Dir: " << output_dir << endl;
    }
};
