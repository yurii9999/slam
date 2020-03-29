#include "tracker.h"
#include <opencv2/imgproc/imgproc.hpp>

using std::vector;

Eigen::Vector3d Tracker::normolize(Eigen::Vector2d a) {

    Eigen::Vector3d res((a[0] - cu) / focal, (a[1] - cv) / focal, 1);
    res /= res.norm();

    return res;
}

void Tracker::push_back(const cv::Mat &img_l, const cv::Mat &img_r)
{
    int height = img_l.size[0];
    int width = img_l.size[1];

    cv::Mat img_l_gray, img_r_gray;
    cv::cvtColor(img_l, img_l_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img_r, img_r_gray, cv::COLOR_BGR2GRAY);

    uint8_t* data_l = (uint8_t*)img_l_gray.data;
    uint8_t* data_r = (uint8_t*)img_r_gray.data;

    int32_t dims[] = {width,height,width};

    push_back(data_l, data_r, dims);
}

void Tracker::push_back(uint8_t *I1,uint8_t* I2,int32_t* dims) {
    if (!current) {
        matcher->pushBack(I1, I2, dims, false);
        current = shared_ptr<RegularFrame>(new RegularFrame);
        return;
    }

    /* previous.snapshot = lalalalalalal; -- save snapshot on previous */
    previous = current;
    current = shared_ptr<RegularFrame>(new RegularFrame);


    matcher->pushBack(I1, I2, dims, false);
    matcher->matchFeatures(2); /* trDelta */

    vector<int> previous_indexs(matcher->n1p2); /* correspondences from libviso's indexes to regularframe::points indexes */
    for (int i = 0; i < previous_indexs.size(); i++) {
        previous_indexs[i] = -1;
    }
    for (int i = 0; i < previous->points.size(); i++) {
        RegularFrame::StablePoint a = previous->points[i];
        previous_indexs[a.index1] = i;
    }

    vector<Matcher::p_match> matches = matcher->getMatches();
    for (int i = 0; i < matches.size(); i++) {
        Matcher::p_match match = matches[i];

        int prev_index = previous_indexs[match.i1p];
        if (prev_index == -1) { /* if this landmark was not observed in the past */

            previous->push_back(match.i1p, match.i2p,
                                normolize(Eigen::Vector2d(match.u1p, match.v1p)),
                                normolize(Eigen::Vector2d(match.u2p, match.v2p)));
            prev_index = previous->points.size() - 1;
        }

        /* complete */
        current->push_back(match.i1c, match.i2c,
                           normolize(Eigen::Vector2d(match.u1c, match.v1c)),
                           normolize(Eigen::Vector2d(match.u2c, match.v2c)),
                           previous->points[prev_index]);
    }
}
