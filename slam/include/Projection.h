//
// Created by yurii on 30/11/2019.
//

#pragma once

#include "Camera.h"
#include "ScenePoint.h"

class Camera;
class ScenePoint;

class Projection {
public:
    double l_u, lv;
    double r_u, r_v;

//private:
    ScenePoint *point;
    Camera *camera;
};
