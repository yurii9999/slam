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
    Tracker(Ptr<Feature2D> detector, Ptr<DescriptorMatcher> matcher) {
        this->m_detector = detector;
        this->m_matcher = matcher;

        this->initialized = false;
    }

    void push_back(cv::Mat left, cv::Mat right);

    vector<DMatch> matchCorrespondencesStereoPair(Mat desc1, Mat desc2);
    vector<DMatch> matchCorrespondences(Mat desc1, Mat desc2);

    /* is it strategy pattern???? */
    Ptr<Feature2D> m_detector; /* why m_...???? */
    Ptr<DescriptorMatcher> m_matcher;

//private:
    /* maybe rename it to 'lastProcessed' */
    Ptr<RegularFrame> prev; /* frame that was processed recently */
    vector<int> corresopondences; /* each RegularFrame is pair of set of correspondend features on stereopair. This vector<int> correspondences is correspondence between current and previous stereopairs */
                                  /* correspondences[i] -- is state[i]'h feature index on previous pair */
    bool initialized; /* <=> prev != NULL */
};


#endif //TRACKING_TRACKER_H
