#pragma once

#include "regularframe.h"

using Eigen::Matrix3d;

class Segmentation {
public:
    struct edge {
        double difference;
        int a_index;
        int b_index;

        edge (int a_idx, int b_idx, double diff) :
            difference(diff), a_index(a_idx), b_index(b_idx)
        {}
    };

    Segmentation(double focal, double cu, double cv, double base);

    void exec(RegularFrame &current);

    vector<Vector3d> derivatives;

    vector<Vector3d> derivatives_ul;
    vector<Vector3d> derivatives_ur;
    vector<Vector3d> derivatives_v;

    vector<Matrix3d> Jacobians;

    vector<edge> graph;

    RegularFrame *current_frame;

private:
    Vector3d getX(Vector2d left, Vector2d right);

    Vector3d getdXduL(Vector2d left, Vector2d right);
    Vector3d getdXduR(Vector2d left, Vector2d right);
    Vector3d getdXdv(Vector2d left, Vector2d right);

    void estimate_derivatives();
    void estimate_additional_derivatives();

    void build_jacobians();

    void build_graph();

    Vector3d estimate_derivative(
            int data_idx,
            Vector3d(Segmentation::*function) (Vector2d left, Vector2d right));

    double base, cu, cv, focal;
};
