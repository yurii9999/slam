#pragma once

#include "regularframe.h"

class Segmentation {
public:
    Segmentation(double focal, double cu, double cv, double base);

    void exec(RegularFrame &current);

    vector<Vector3d> derivatives;

private:
    Vector3d triangulate(Vector2d left, Vector2d right);

    void estimate_derivatives();
    void estimate_derivative();

    double base, cu, cv, focal;
};
