//
// Created by yurii on 03/12/2019.
//

#include "FrameKeeper.h"

void FrameKeeper::push_back(Ptr<RegularFrame> frame, vector<int> correspondences)
{
    if (this->cur == NULL) {
        this->cur = frame;
        vector<int> empty;
        this->correspondences = empty;
        return;
    }

    this->prev.release();
    this->prev = this->cur;
    this->cur = frame;
    this->correspondences = correspondences;
}
