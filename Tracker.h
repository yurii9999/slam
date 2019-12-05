#ifndef TRACKING_TRACKER_H
#define TRACKING_TRACKER_H

#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <opencv2/features2d.hpp>
#include "Camera.h"

#include "FrameKeeper.h"

#include "RegularFrame.h"

using namespace std;
using namespace cv;

class Tracker {
public:
    /* I'd considered two ways to hold information about KeyPoint:
     * 1. Extend KeyPoint (ExtendedKeyPoint) with additional fields (age, ...)
     * 2. Add additionval vector of struct information (age, ...) associated with feature that has same index (for left and right frames separated)
     *
     * vector<KeyPoints> we get from opencv's method, and I dont know how to extend it to vector<ExtendedKeyPoints> without a lot of copying,
     * and I dont know how to use clever vector<ExtendedKeyPoint> instead of vector<KeyPoint>
     * And we already hold descriptors and keyPoints separated, and now we just hold separated descriptors, keypoints and addition informations */


    /* КпАддишнл и РегуларФрэйм дожны описываться в отдельном файле, трекер их хорошо знает, но при этом является отдеьной сущностью */
//    struct KpAdditional {
//        int age;
//        int prevIndex; /* it s actually is not kp's property, it s more frames relation's. */
//        bool isKnown; /* is the feature observed in at least one key frame */
//        /* это поле (isKnown) заполняется, при вызове функции, которая будет формировать из фрейма камеру (когда мы решиили, что щас вот у нас будет новый ключевой кадр */
//        /* ну и понятное дело эти поля передаются и совершенствуются с течением времени от фич к их соответсвиям на новых кадрах */
//    };


    /* camera = frame + R,t */
    /* maybe move to separated class but later */
//    struct RegularFrame {
//        vector<KeyPoint> kpts_l, kpts_r; /* ya ponyal: kpts = KeyPoinTS */
//        Mat desc_l, desc_r;
//        vector<KpAdditional> additional_l, additional_r;
////        vector<KpAdditional> info_l, info_r; /* uncomment it later */
//        /* I dont completly understand, compute we correspondences between stereopair once (when it is a new frame(L -> R)) or twice (when it is a new frame(L -> R) and when it is previous frame(R -> L))
//         * and is there essential difference between (L -> R) and (R -> L) */

//        RegularFrame(vector<KeyPoint> kpts_l, vector<KeyPoint> kpts_r, Mat desc_l, Mat desc_r):
//            kpts_l(kpts_l), kpts_r(kpts_r), desc_l(desc_l), desc_r(desc_r) {}
//    };

    Tracker(Ptr<Feature2D> detector, Ptr<DescriptorMatcher> matcher) {
        this->m_detector = detector;
        this->m_matcher = matcher;

        this->initialized = false;
    }

    void push_back(cv::Mat left, cv::Mat right);

    vector<DMatch> matchCorrespondencesStereoPair(Mat desc1, Mat desc2);
    vector<DMatch> matchCorrespondences(Mat desc1, Mat desc2);

    /* strategy pattern */
    Ptr<Feature2D> m_detector; /* why m_...???? */
    Ptr<DescriptorMatcher> m_matcher;

//private:
    /* maybe rename it to 'lastProcessed' */
    Ptr<RegularFrame> prev; /* frame that was processed recently */
    bool initialized; /* <=> prev != NULL */
};


#endif //TRACKING_TRACKER_H
