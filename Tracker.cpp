//
// Created by yurii on 29/11/2019.
//
#include <stdio.h>
#include <iostream>
#include "RegularFrame.h"
#include "Tracker.h"

using namespace std;

using namespace cv;


/*  написать:
 * замену дескрипторов
 * использование информации которая дополнительная в отдельной структуре (увеличение взраста с течением времени, и остальную информацию, чтобы *хз, забыл дописать а сейчас забыл и лень перечитывать чтобы вспоминать сори(((( *
 * нарисовать метод который рисует точки одним цветом на четырех кадрах */

/* methods search first element of list, where queiryIndx = index and returns trainIndx or -1 if index does not founded*/
int getNextIndex(int index, vector<DMatch> list) {
    for (int i = 0; i < list.size(); i++) {
        DMatch c = list[i];
        if (c.queryIdx == index) {
            return c.trainIdx;
        }

        if (c.queryIdx > index) {
            return -1;
        };
    }

    return -1;
}

/* returns vector of indexs of points in vector<DMatch> begin, that was found through all nodes */
/* and vector on indexs -- correspondence: correspondence[i] -- corresponding to stable[i]'th feature on previous stereopair */
pair<vector<int>, vector<int>> getStable(vector<DMatch> cur_left, vector<DMatch> cur_right, vector<DMatch> prev_right, vector<DMatch> prev_left) {
    vector<int> stable;
    vector<int> correspondence;

    for (int i = 0; i < cur_left.size(); i++) {
        DMatch c = cur_left[i];
        int i1 = c.queryIdx;
        int i2 = c.trainIdx;
        int i3 = getNextIndex(i2, cur_right); /* i3 -- index of feature on prev_right */
        int i4 = getNextIndex(i3, prev_right); /* i4 -- index of feature on prev_left */
        int i5 = getNextIndex(i4, prev_left);

        if (i1 == i5) {
            stable.push_back(i);
            correspondence.push_back(i4);
        }
    }

    pair<vector<int>, vector<int>> result(stable, correspondence);
    return result;
}

void Tracker::push_back(cv::Mat img_left, cv::Mat img_right) {
    vector<KeyPoint> kpts_l, kpts_r;
    Mat desc_l, desc_r;
    this->m_detector->detectAndCompute(img_left, Mat(), kpts_l, desc_l);
    this->m_detector->detectAndCompute(img_right, Mat(), kpts_r, desc_r);

    vector<DMatch> correspondences = matchCorrespondencesStereoPair(desc_l, desc_r);
    /* hold points that found in both left and right frames (<- is it true???)
     * future process only stable (that found on previous and current) */

    vector<KeyPoint> matchers_l, matchers_r; // -- matched correspondences matchers_l[i] correspond matchers_r[i]
    Mat newDesc_l, newDesc_r; // -- new descriptors
    newDesc_l = Mat(correspondences.size(), desc_l.cols, desc_l.type());
    newDesc_r = Mat(correspondences.size(), desc_l.cols, desc_l.type());

    int amount = 0;

    for (int i = 0; i < correspondences.size(); i++) {
        DMatch concreteMatch = correspondences[i];
        matchers_l.push_back(kpts_l[concreteMatch.queryIdx]);
        matchers_r.push_back(kpts_r[concreteMatch.trainIdx]);

        Mat row1 = desc_l.row(concreteMatch.queryIdx);
        row1.copyTo(newDesc_l.row(amount));

        Mat row2 = desc_r.row(concreteMatch.trainIdx);
        row2.copyTo(newDesc_r.row(amount));

        amount++;
    }

    /* above that was new stereopair processing */

    /* initialization */
    if (!initialized) {
        vector<RegularFrame::KpAdditional> kkk;
        vector<int> empty;
        vector<int> emptyCorrespondences;
        this->corresopondences = emptyCorrespondences;
        prev = new RegularFrame(matchers_l, matchers_r, newDesc_l, newDesc_r, kkk, kkk, empty);
        initialized = true;
        return;
    }

    /* matching in the chain: c_l -> c_r -> p_r -> p_l -> c_l and prosess only these */
    vector<DMatch> cl_cr;
    for (int i = 0; i < matchers_l.size(); i++) {
        cl_cr.push_back(DMatch(i, i, 0)); /* 0 is a 'disstance'; for our purposes(track features through 4 frames) it does not matter */
    }
    vector<DMatch> cr_pr = matchCorrespondences(newDesc_r, prev.get()->descriptors_r);
    vector<DMatch> pr_pl = matchCorrespondencesStereoPair(prev.get()->descriptors_r, prev.get()->descriptors_l); /* or form it by the knowlege, that desc_l[i] correspond desc_r[i] */
    vector<DMatch> pl_cl = matchCorrespondences(prev.get()->descriptors_l, newDesc_l);
    /* or we can remove features that already are not stable, before trying to match them in next frame */


    pair<vector<int>, vector<int>> stableAndCorrespondece = getStable(cl_cr, cr_pr, pr_pl, pl_cl);
    vector<int> stableFeatures = stableAndCorrespondece.first; /* indexes of features in matchers_l */
    vector<int> correspondencesBetweenFrames = stableAndCorrespondece.second; /* curFeatures[stable[i]] correspond to prevFeatures[correspondenceBetweenFrames[i]] */
    /* do something with names of vars */

    this->corresopondences = correspondencesBetweenFrames;

    vector<RegularFrame::KpAdditional> kkk;
    this->prev.release();
    this->prev = new RegularFrame(matchers_l, matchers_r, newDesc_l, newDesc_r, kkk, kkk, stableFeatures);;

    /* is there essential difference beetwen left -> right and right -> left ??????????? */
    return;
}

/* find correspondences between two frames (frames represented as a set of features with corresponded descriptors) */
vector<DMatch> Tracker::matchCorrespondences(Mat desc1, Mat desc2) {
    vector<vector<DMatch>> knn_matches;
    this->m_matcher->knnMatch( desc1, desc2, knn_matches, 2);
    const float ratio_thresh = 0.7f;

    vector<DMatch> good_matches;
    for (size_t i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
        {
            good_matches.push_back(knn_matches[i][0]);
        }
    }

    return good_matches;
}

/* it could be more effective in a future, if use here epipolar constraint */
vector <DMatch> Tracker::matchCorrespondencesStereoPair(Mat desc1, Mat desc2) {
    return this->matchCorrespondences(desc1, desc2);
}
