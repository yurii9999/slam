//
// Created by yurii on 03/12/2019.
//

#ifndef TRACKING_REGULARFRAME_H
#define TRACKING_REGULARFRAME_H

#include <opencv2/core.hpp>
#include <vector>

using namespace cv;
using namespace std;

class RegularFrame {
public:
    struct KpAdditional {
        int age;
        bool isKnown; /* hz zachem eto tut */
    };
    /* sticks two images into one
     * maybe its better to move it in separated header fancyDrawer or st, cause stick is not frame's responsobility, and actually darwing is not too */
    static Mat stick(Mat img1, Mat img2);
    Mat drawStable(Mat img1, Mat img2);

    RegularFrame(vector<KeyPoint> kpts_l, vector<KeyPoint> kpts_r, Mat desc_l, Mat desc_r, vector<KpAdditional> addi_l, vector<KpAdditional> addi_r, vector<int> stable) {
        this->kpts_l = kpts_l;
        this->kpts_r = kpts_r;
        this->descriptors_l = desc_l;
        this->descriptors_r = desc_r;
        this->additional_l = addi_l;
        this->additional_r = addi_r;
        this->stable = stable;
//        this->indexes = ????
    }

//private:
    vector<KeyPoint> kpts_l, kpts_r;
    Mat descriptors_l, descriptors_r;
    vector<KpAdditional> additional_l, additional_r;

    vector<int> stable; /* stable points that have found in chain provided by tracker.h */

    vector<int> indexes; /* indexs[i] is correspondence for i'th point in previous fram */
    /* it s not supported now */
};


#endif //TRACKING_REGULARFRAME_H
