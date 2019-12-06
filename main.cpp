#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"

#include "Tracker.h"

#include "opencv2/imgproc.hpp"

#include "RegularFrame.h"
#include "FancyDrawer.h"

#include "FrameKeeper.h"

using namespace std;
using namespace cv;

/* Добавить изменение дескрипторов в трекере и обновление дополнительной информации */

int main(int argc, char** argv)
{
    string dir = argv[1];
    string output_dir = "out/";
    Ptr<Feature2D> detector = AKAZE::create();
    Ptr<DescriptorMatcher> matcher = BFMatcher::create(NORM_HAMMING);
    Tracker t(detector, matcher);

    FrameKeeper fk;

    for (int32_t i=0; i<20; i++) {
        // st is taken from libviso's demo.cpp
        // input file names
        char base_name[256]; sprintf(base_name,"%06d.png",i);
        string left_img_file_name  = dir + "/I1_" + base_name;
        string right_img_file_name = dir + "/I2_" + base_name;

        // catch image read/write errors here
        Mat img_l = imread(left_img_file_name);
        Mat img_r = imread(right_img_file_name);

        t.push_back(img_l, img_r);
        fk.push_back(t.prev, t.corresopondences);
        string outname = output_dir + (string) base_name;
    }

    cout << "Hello World!" << endl;
    return 0;
}
