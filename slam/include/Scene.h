//
// Created by yurii on 30/11/2019.
//
#pragma once

#include "Camera.h"
#include "ScenePoint.h"

class Scene {
public:
private:
    vector<ScenePoint*> points;
    vector<Camera*> cameras; /* only keyframe cameras are included */
};
