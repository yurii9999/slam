//
// Created by yurii on 30/11/2019.
//

#ifndef TRACKING_SCENE_H
#define TRACKING_SCENE_H

#include "Camera.h"
#include "ScenePoint.h"

class Scene {
public:
private:
    vector<ScenePoint*> points;
    vector<Camera*> cameras; /* only keyframe cameras are included */
};


#endif //TRACKING_SCENE_H
