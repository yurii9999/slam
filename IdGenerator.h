//
// Created by yurii on 06/12/2019.
//

#ifndef SLAM_IDGENERATOR_H
#define SLAM_IDGENERATOR_H

#include <vector>

using namespace std;


/* class generates and control id's
 * -- need for tracking, features' field id: if feature was recognized as stable in two consecutive frames, then it has same id in both of regularframes' additional_l and additional_r */

/* There's situation: we have two chosen regular frames to be keyFrames, and probably they have some common features
 * if now we triangulate them both directly, we get two images of these common features, but if we recognize these common feature(by checking if they having same id), we build
 * only one 3D point */

/* if features in two regular frame have same id and does not assuming be projection of same 3D point, then flag isKnown be set false on this feature on last regularFrame */
class IdGenerator {
public:
    int getId();
    void release(int id);
//private:
    vector<bool> isFree;
};


#endif //SLAM_IDGENERATOR_H
