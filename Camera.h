//
// Created by yurii on 30/11/2019.
//

#ifndef TRACKING_CAMERA_H
#define TRACKING_CAMERA_H

#include <opencv2/core.hpp>
#include "Projection.h"

using namespace cv;

/* camera = frame + R,t */

class Camera {
public:
    Mat R;
    Point3d t;

    /* fabric method */
//    static getCameraFromStereopair(Mat left, Mat right);
//private:
    /* each frame represented as the set of features we found */
//    vector<*Projection> features;

    /* draw features which describe stereopair on [corresponded] stereopair and stick images */
    Mat drawFeatures(Mat img_left, Mat img_right);
    /* move implemenation from Tracker, and actually "sticking" and "drawing" are not camera's responsobility, so dont do anything*/
    static Mat stick(Mat img1, Mat img2);
};


#endif //TRACKING_CAMERA_H
