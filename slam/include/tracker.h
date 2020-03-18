#pragma once

#include "regularframe.h"

#include "matcher.h"
#include <memory>
#include <opencv2/imgproc.hpp>

using std::shared_ptr;
using cv::Mat;

class Tracker
{
public:
    Tracker(Matcher::parameters param) {
        matcher = new Matcher(param);
    }

    ~Tracker() {
        delete matcher;
    }

    void push_back(const Mat &img_l, const Mat &img_r);
    void push_back(uint8_t *I1,uint8_t* I2,int32_t* dims);

//private:
    Matcher *matcher;
    shared_ptr<RegularFrame> previous;
    shared_ptr<RegularFrame> current;
};
