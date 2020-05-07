#pragma once

#include "regularframe.h"
#include "viso2/matcher.h"

#include <memory>
#include <opencv2/imgproc.hpp>

using std::shared_ptr;
using cv::Mat;

class Tracker
{
public:
    Tracker(double focal, double cu, double cv, Matcher::parameters param) {
        this->focal = focal;
        this->cu = cu;
        this->cv = cv;

        param.f = focal;
        param.cu = cu;
        param.cv = cv;
        matcher = new Matcher(param);
    }

    ~Tracker() {
        delete matcher;
    }

    void push_back(const Mat &img_l, const Mat &img_r, SE3d *prediction = nullptr);
    void push_back(uint8_t *I1,uint8_t* I2,int32_t* dims, SE3d *prediction = nullptr);

//private:

    double focal;
    double cv;
    double cu;

    Matcher *matcher;
    shared_ptr<RegularFrame> previous;
    shared_ptr<RegularFrame> current;
};
