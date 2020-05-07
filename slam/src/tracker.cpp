#include "tracker.h"
#include <opencv2/imgproc/imgproc.hpp>

#include <sophus/se3.hpp>

using std::vector;

//Eigen::Vector3d Tracker::normolize(Eigen::Vector2d a) {

//    Eigen::Vector3d res((a[0] - cu) / focal, (a[1] - cv) / focal, 1);
//    res /= res.norm();

//    return res;
//}

void Tracker::push_back(const cv::Mat &img_l, const cv::Mat &img_r, Sophus::SE3d *prediction)
{
    int height = img_l.size[0];
    int width = img_l.size[1];

    cv::Mat img_l_gray, img_r_gray;
    if (img_l.type() != CV_8UC1) {
        cv::cvtColor(img_l, img_l_gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(img_r, img_r_gray, cv::COLOR_BGR2GRAY);
    }
    else {
        img_l_gray = img_l;
        img_r_gray = img_r;
    }

    uint8_t* data_l = (uint8_t*)img_l_gray.data;
    uint8_t* data_r = (uint8_t*)img_r_gray.data;

    int32_t dims[] = {width,height,width};

    push_back(data_l, data_r, dims, prediction);
}

void Tracker::push_back(uint8_t *I1,uint8_t* I2,int32_t* dims, SE3d *prediction) {
    if (!current) {
        matcher->pushBack(I1, I2, dims, false);
        current = shared_ptr<RegularFrame>(new RegularFrame);
        return;
    }

    /* previous.snapshot = lalalalalalal; -- save snapshot on previous */
    previous = current;
    current = shared_ptr<RegularFrame>(new RegularFrame);

    matcher->pushBack(I1, I2, dims, false);
    if (prediction) {
        SE3d m = prediction->inverse();
        Eigen::Matrix4d temp = m.matrix();
        temp.transposeInPlace();

        Matrix tr_delta(4, 4, temp.data());
        matcher->matchFeatures(2, &tr_delta);
    }
    else
        matcher->matchFeatures(2);

    vector<int> previous_indexs(matcher->n1p2); /* correspondences from libviso's indexes to regularframe::points indexes */
    for (int i = 0; i < previous_indexs.size(); i++) {
        previous_indexs[i] = -1;
    }
    for (int i = 0; i < previous->amount_points(); i++) {
        RegularFrame::feature_additional &a = previous->additionals[i];
        previous_indexs[a.index_left] = i;
    }

    vector<Matcher::p_match> matches = matcher->getMatches();
    for (int i = 0; i < matches.size(); i++) {
        Matcher::p_match match = matches[i];

        int prev_index = previous_indexs[match.i1p];
        if (prev_index == -1) { /* if this landmark was not observed in the past */

            previous->push_back(match.i1p, match.i2p,
                                Eigen::Vector2d(match.u1p, match.v1p),
                                Eigen::Vector2d(match.u2p, match.v2p));
            prev_index = previous->amount_points() - 1;
        }

        /* complete */
        current->push_back(match.i1c, match.i2c,
                           Eigen::Vector2d(match.u1c, match.v1c),
                           Eigen::Vector2d(match.u2c, match.v2c),
                           RegularFrame::point_reference(previous.get(), prev_index));
    }
}
