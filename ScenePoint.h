//
// Created by yurii on 30/11/2019.
//

#ifndef TRACKING_SCENEPOINT_H
#define TRACKING_SCENEPOINT_H

#include "Projection.h"
#include <vector>

using namespace std;

class Projection;

class ScenePoint {
public:
    double x, y, z;

//private:
    /* all projections of this point */
    vector<Projection*> projections;
    // are these projection from key-frames only???? or from each frame that we processed in odometry thread??????
};


#endif //TRACKING_SCENEPOINT_H
