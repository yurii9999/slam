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
using namespace cv;

using namespace opengv;

struct parameters
{
    int ransac_max_iterations = 0;
    double ransac_threshold = 0.;

    int bucketing_something = 0;
    parameters(string filename) {
        rapidxml::file<> xmlFile(filename.c_str());
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());

        ransac_max_iterations = stoi(doc.first_node("ransac_max_iterations")->value());
        ransac_threshold = stod(doc.first_node("ransac_threshold")->value());
    }
};

struct input_parameters
{
    string sequence_path = "";
    double focal = 0.;
    double cu =0.;
    double cv = 0.;
    double base = 0.; // meters

    vector<double> parse_calib_params(string line) {
        line.erase(line.begin(), line.begin() + 4);

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
        string calib_filename;
        if (calibration)
            calib_filename = calibration->value();
        else
            calib_filename = sequence_path + "/calib.txt";

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

    double f  = input_params.focal;
    double cu = input_params.cu;
    double cv = input_params.cv;
    double base = input_params.base;

    Matcher::parameters params_matcher;
    params_matcher.cu = cu;
    params_matcher.cv = cv;
    params_matcher.base = base;
    params_matcher.f = f;

    Tracker tracker(f, cu, cv, params_matcher);

    EgomotionEstimation ego(f, cu, cv, base);

    vector<shared_ptr<RegularFrame>> frames;

    string left_img_dir = input_params.sequence_path + "/image_0";
    string right_img_dir = input_params.sequence_path + "/image_1";

    cv::Mat img_l;
    cv::Mat img_r;
    for (int32_t i_frame =0; i_frame < 500; i_frame ++) {
       char image_name[256]; sprintf(image_name,"/%06d.png",i_frame );
       string left_img_file_name  = left_img_dir + image_name;
       string right_img_file_name = right_img_dir + image_name;
       img_l = imread(left_img_file_name);
       img_r = imread(right_img_file_name);

       tracker.push_back(img_l, img_r);
       frames.push_back(tracker.current);

       RegularFrame &current_frame = *tracker.current;
       if (i_frame  == 0) {
           /* initialize first frame on origin */
           SE3d origin;
           current_frame.set_motion(origin);

           continue;
       }

       RegularFrame &previous_frame = *tracker.previous;

       ego.estimate_motion(current_frame, previous_frame);
       cout << "i = " << i_frame << "\t correspondences: " << ego.amount_correspondences << "\t Inliers: " << ego.amount_inliers << endl;

    }

    ofstream output;
    output.open("result_scene");

    for (int i = 0; i < ego.scene.size(); i++) {
        output << ego.scene[i]->landmark.transpose() << endl;
    }


    output.close();

    output.open("result_pose");
    for (int i = 0; i < frames.size(); i++) {
        output << frames[i]->motion.translation().transpose() << endl;
    }

    output.close();

    return 0;
}
