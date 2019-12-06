#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"

#include "../Tracker.h"

#include "opencv2/imgproc.hpp"

#include "../RegularFrame.h"
#include "../FancyDrawer.h"

using namespace std;
using namespace cv;

/* demo2.cpp gets first 2 stereo pairs from 2010_03_09_drive_0019, drawing correspondences on all 4 images same color */

int main(int argc, char** argv)
{
    string dir = argv[1];
    string output_dir = "out/";
    Ptr<Feature2D> detector = AKAZE::create();
    Ptr<DescriptorMatcher> matcher = BFMatcher::create(NORM_HAMMING);
    Tracker t(detector, matcher);

    Mat img1_l = imread(dir + "I1_000000.png");
    Mat img1_r = imread(dir + "I2_000000.png");
    Mat img2_l = imread(dir + "I1_000001.png");
    Mat img2_r = imread(dir + "I2_000001.png");

    t.push_back(img1_l, img1_r);
    Ptr<RegularFrame> frame1 = t.prev;
    t.push_back(img2_l, img2_r);
    Ptr<RegularFrame> frame2 = t.prev;

    vector<int> correspondeces = t.corresopondences;
    for (int i = 0; i < correspondeces.size(); i++) {
        cout << "\t" << correspondeces[i];
    }
    cout << endl << correspondeces.size();

    const int Radius = 5;
    for (int i = 0; i < frame2.get()->stable.size(); i++) {
        int stable_frame2 = frame2.get()->stable[i];
        int correspondence_frame1 = correspondeces[i];

        KeyPoint a = frame1.get()->kpts_l[correspondence_frame1];
        KeyPoint b = frame1.get()->kpts_r[correspondence_frame1];
        KeyPoint c = frame2.get()->kpts_l[stable_frame2];
        KeyPoint d = frame2.get()->kpts_r[stable_frame2];

        int range = 255;
        int blue = (range / 1) * (i % 2) ;
        int green = (range / 2) * (i % 3);
        int red = (range / 4) * (i % 5);

        Scalar color(blue, green, red);
        circle(img1_l, a.pt, Radius, color);
        circle(img1_r, b.pt, Radius, color);

        circle(img2_l, c.pt, Radius, color);
        circle(img2_r, d.pt, Radius, color);
    }

    Mat res1 = FancyDrawer::stick(img1_l, img1_r);
    Mat res2 = FancyDrawer::stick(img2_l, img2_r);
    /* drawing */

    imwrite("res1.png", res1);
    imwrite("res2.png", res2);

    frame1.release();
    frame2.release();

    detector.release();
    matcher.release();

    cout << "Done!" << endl;
    return 0;
}
