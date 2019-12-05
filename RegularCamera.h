//
// Created by yurii on 30/11/2019.
//

#ifndef TRACKING_REGULARCAMERA_H
#define TRACKING_REGULARCAMERA_H

#include "Camera.h"
#include "opencv2/core.hpp"

/* move all to regular frame and remove it */

using namespace cv;

/* class describes regular camera (that we use for all processed frames)
 * if frame is chosen to be key-frame, we can create Camera object from here
 * main defference with Camera is having more information about features (for tracking), these classes (Camera and RegularCamera) are not suppose to have common interface */

class RegularCamera {
public:
    Camera getKeyFrameCamera();

private:
    /* dont sure we need R & t */
    /* maybe create abstraction "set of features", that can be upgraded to RegularCamera??????? */
    /* or maybe just remove this class and hold information about features and theirs descriptors in Tracking class????? But we have to keep information about each Projection and assotiate it with real point
     * because we want to observe it in more than two frames */
    Mat R;
    Point3d t;

    vector<KeyPoint> from_left;
    vector<KeyPoint> from_right;
    Mat left_descriptors;
    Mat right_descriptors;



};


#endif //TRACKING_REGULARCAMERA_H
