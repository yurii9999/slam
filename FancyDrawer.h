//
// Created by yurii on 04/12/2019.
//

#ifndef TRACKING_FANCYDRAWER_H
#define TRACKING_FANCYDRAWER_H

#include <opencv2/core.hpp>

#include "RegularFrame.h"

using namespace cv;

class FancyDrawer {
public:
    static Mat stick(Mat img1, Mat img2);
    static Mat drawFrame(Mat img_l, Mat img_r, RegularFrame frame);
    static Mat drawStable(Mat img_l, Mat img_r, RegularFrame frame);
};


#endif //TRACKING_FANCYDRAWER_H
