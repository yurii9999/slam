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
        bool isKnown;
        int id; /* while key point exist it has same id; */
                /* it does not help in cases when kp disappeared and observd again, it will be another point in this defenition */

        KpAdditional(int age, bool isKnown, int id):
            age(age), isKnown(isKnown), id(id) {}
    };

    RegularFrame(vector<KeyPoint> kpts_l, vector<KeyPoint> kpts_r, Mat desc_l, Mat desc_r, vector<KpAdditional> additional, vector<int> stable) {
        this->kpts_l = kpts_l;
        this->kpts_r = kpts_r;
        this->descriptors_l = desc_l;
        this->descriptors_r = desc_r;
        this->additional = additional;
        this->stable = stable;
    }

//private:
    vector<KeyPoint> kpts_l, kpts_r;
    Mat descriptors_l, descriptors_r; /* probably we can reduce it to descriptors_l but not now */

    vector<KpAdditional> additional; /* these information interesting for us only about stable features */
    vector<int> stable; /* stable points that have found in chain provided by tracker.h */
};


#endif //TRACKING_REGULARFRAME_H
