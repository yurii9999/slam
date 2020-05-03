#include "additional/sequence_parameters.h"
#include "additional/segmentation_parameters.h"
#include "additional/fabric.h"

#include <cxxopts.hpp>

#include "viso2/matcher.h"
#include "tracker.h"
#include "segmentation.h"

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace cv;
using namespace std;

void draw(Mat &img, Vector2d feature, Scalar color) {
    Point a(feature[0], feature[1]);
    circle(img, a, 3, color, 2);
}

Scalar generate_color(int num) {
    int r = (127 + num * 7 % 255);
    int g = (63 + num * 11 % 255);
    int b = (87 + num * 13 % 255);
    return Scalar(b, g, r);
}

void draw_graph(Mat &img, Segmentation segmentation, RegularFrame &current_frame) {
    int num = 213312;
    for (auto component : segmentation.graph.components) {
        if (component.size() > 1) {
            Scalar color = generate_color(num);
            for (auto idx : component) {
                Point a(current_frame.image_points_left[idx][0], current_frame.image_points_left[idx][1]);
//                circle(img, a, 3, color, 2);

                for (auto idx2 : component) {
                    Point b(current_frame.image_points_left[idx2][0], current_frame.image_points_left[idx2][1]);
                    line(img, a, b, color, 1);
                }
            }

            num += 157;
        }
        cout << component.size() << endl;
    }
    num = 213312;
    for (auto component : segmentation.graph.components) {
        if (component.size() > 5) {
            Scalar color = generate_color(num);
            for (auto idx : component) {
                Point a(current_frame.image_points_left[idx][0], current_frame.image_points_left[idx][1]);
                circle(img, a, 3, color, 2);
            }

            num += 157;
        }
    }
//    for (auto e : segmentation.graph) {
//        Point a(current_frame.image_points_left[e.a_index][0], current_frame.image_points_left[e.a_index][1]);
//        Point b(current_frame.image_points_left[e.b_index][0], current_frame.image_points_left[e.b_index][1]);

//        // BGR
//        Scalar color(0, 0, 255);
//        if (e.difference < th)
//            color = Scalar(0, 255, 0);

//        line(img, a, b, color, 2);
//    }

}

int main (int argc, char** argv) {
    cxxopts::Options options("Something", "try to use on kitti dataset");
    options.add_options()
            ("segmentation", "xml with segmentation parameters", cxxopts::value<string>())
            ("input", "Input parameters such as calibration, path to sequence", cxxopts::value<string>());
    auto result = options.parse(argc, argv);

    segmentation_parameters segmentation_params(result["segmentation"].as<string>());
    sequence_parameters sequence_params(result["input"].as<string>());

    sequence_params.print();
    segmentation_params.print();

    Tracker tracker = Fabric::get_with_params(sequence_params);
    Segmentation segmentation = Fabric::get_with_params(segmentation_params, sequence_params);

    vector<shared_ptr<RegularFrame>> frames;

    int i_frame = sequence_params.first_frame;
    Mat img_l, img_r;
    do {
        char image_name[256]; sprintf(image_name,"/%06d.png",i_frame );
        string left_img_file_name  = sequence_params.sequence_path + "/image_0" + image_name;
        string right_img_file_name = sequence_params.sequence_path + "/image_1" + image_name;
        img_l = imread(left_img_file_name, CV_LOAD_IMAGE_GRAYSCALE);
        img_r = imread(right_img_file_name, CV_LOAD_IMAGE_GRAYSCALE);

        tracker.push_back(img_l, img_r);

        frames.push_back(tracker.current);

        RegularFrame &current_frame = *tracker.current;
        if (i_frame  == sequence_params.first_frame) {
            i_frame++;
            continue;
        }
        RegularFrame &previous_frame = *tracker.previous;

        segmentation.exec(current_frame);

        // Draw points according to the derivatives:
        // smoth gradient from green(no motion) to red(big motion)

        Scalar red(0, 0, 255);

        Mat res;
        cv::cvtColor(img_l, res, cv::COLOR_GRAY2BGR);


        draw_graph(res, segmentation, current_frame);

        imwrite("destt"+ string(image_name), res);


        i_frame++;
    } while(img_l.data && i_frame != sequence_params.amount_frames + sequence_params.first_frame);
}
