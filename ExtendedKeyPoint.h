//
// Created by yurii on 01/12/2019.
//

#include <opencv2/core.hpp>

#ifndef TRACKING_EXTENDEDKEYPOINT_H
#define TRACKING_EXTENDEDKEYPOINT_H

using namespace cv;

class ExtendedKeyPoint: public KeyPoint{
public:
    void markAsKnown() {
        this->isKnown = true;
    }
//private:
    /* exist keyFrame that contains this feature */
    bool isKnown = false;
    int age = 0;
};


#endif //TRACKING_EXTENDEDKEYPOINT_H
