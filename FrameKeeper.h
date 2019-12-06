//
// Created by yurii on 03/12/2019.
//

#ifndef TRACKING_FRAMEKEEPER_H
#define TRACKING_FRAMEKEEPER_H

#include "Tracker.h"
#include "RegularFrame.h"
#include <vector>

using namespace std;
/* class keeps n(=2) last RegularFrames and correspondences between them */
/* МОжно сделать декоратор над треккингом который логгирует точки */

class FrameKeeper {
public:
    /* now it keeps two last frames that we gave it, but in the future there will be need to keep more than two */
    void push_back(Ptr<RegularFrame> frame, vector<int> correspondences); /* to initialize system, better just push back empty vector, than create special method for this */
private:
    Ptr<RegularFrame> prev;
    vector<int> correspondences;
    Ptr<RegularFrame> cur;
};


#endif //TRACKING_FRAMEKEEPER_H
