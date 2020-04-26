#include "segmentation.h"

#include <vector>
#include <eigen3/Eigen/Core>

#include "dt/delaunay.h"
#include "dt/edge.h"
#include "dt/vector2.h"

using Eigen::Vector3d;
using std::vector;

using namespace std;


Segmentation::Segmentation(double focal, double cu, double cv, double base)
{
    this->focal = focal;
    this->cu = cu;
    this->cv = cv;
    this->base = base;
}

void Segmentation::exec(RegularFrame &current) {
    current_frame = &current;

    // estimate via finite differences
    estimate_derivatives();

    // estimate additional derivatives to build Jacobian of velocity
    estimate_additional_derivatives();

    // build jacobians from partial derivatives that were compute on previous step
    build_jacobians();
    // TODO: fuse two previos steps

    // build graph:
    // delaunay triangulation & weighted each edge (weight = || a.derivative - b.derivative || )
    build_graph();

}

void Segmentation::estimate_derivatives() {
    derivatives.clear();
    derivatives.resize(current_frame->additionals.size());

    for (auto p : current_frame->additionals)
        derivatives[p.index] = estimate_derivative(p.index, &Segmentation::getX);
}

void Segmentation::estimate_additional_derivatives() {
    derivatives_ul.clear();
    derivatives_ur.clear();
    derivatives_v.clear();

    derivatives_ul.resize(current_frame->additionals.size());
    derivatives_ur.resize(current_frame->additionals.size());
    derivatives_v.resize(current_frame->additionals.size());

    for (auto p : current_frame->additionals) {
        derivatives_ul[p.index] = estimate_derivative(p.index, &Segmentation::getdXduL);
        derivatives_ur[p.index] = estimate_derivative(p.index, &Segmentation::getdXduR);
        derivatives_v[p.index] = estimate_derivative(p.index, &Segmentation::getdXdv);
    }
}

void Segmentation::build_jacobians() {
    Jacobians.clear();
    Jacobians.resize(current_frame->additionals.size());

    for (auto p : current_frame->additionals) {
        Matrix3d a;
        a.col(0) = derivatives_ul[p.index];
        a.col(1) = derivatives_ur[p.index];
        a.col(2) = derivatives_v[p.index];

        Jacobians[p.index] = a;
    }
}

void Segmentation::build_graph() {
    /* wrap for https://github.com/Bl4ckb0ne/delaunay-triangulation with little correction (with Vector2 associatet id) */
    vector<dt::Vector2<double>> points;
    for (auto p : current_frame->additionals)
        points.push_back(dt::Vector2<double>(
                            current_frame->image_points_left[p.index][0],
                            current_frame->image_points_left[p.index][1],
                            p.index));

    dt::Delaunay<double> triangulation;
    triangulation.triangulate(points);


    vector<dt::Edge<double>> edges = triangulation.getEdges();

    graph.clear();
    graph.reserve(current_frame->additionals.size());
    for (auto e : edges) {
        int idx_a = e.v->p_index;
        int idx_b = e.w->p_index;

//        double difference = (derivatives[idx_a] - derivatives[idx_b]).norm();

        Matrix3d J_ij = Jacobians[idx_a] - Jacobians[idx_b];
        Vector3d delta = derivatives[idx_a] - derivatives[idx_b];
        Matrix3d covariace = J_ij * J_ij.transpose();
        double difference = delta.transpose() * covariace * delta;
        graph.push_back(edge(idx_a, idx_b, difference));

//        cout << "====================================" << endl;
//        cout << "New edge: " << endl;
//        cout << "Idx A: " << idx_a << "\tIdx B: " << idx_b << endl;
//        cout << "Image point A: " << current_frame->image_points_left[idx_a].transpose() << endl;
//        cout << "Image point B: " << current_frame->image_points_left[idx_b].transpose() << endl;
////        Vector3d A = getX(current_frame->image_points_left[idx_a], current_frame->image_points_right[idx_a]);
////        Vector3d B = getX(current_frame->image_points_left[idx_b], current_frame->image_points_right[idx_b]);
//        cout << "VelA: " << derivatives[idx_a].transpose() << endl;
//        cout << "VelB: " << derivatives[idx_b].transpose() << endl;

//        cout << "JacobianA: \n" << Jacobians[idx_a] << endl;
//        cout << "JacobianB: \n" << Jacobians[idx_b] << endl;


        cout << "Covariance: \n" << covariace << endl;

//        cout << "Covariance inversed: \n" << covariace.inverse() << endl;

        cout << "Result: " << difference << endl;

//        --input=../../sequence --seg_th=0.01


    }
}

/* estimate derivateve of function with pair of image-point coordinates as argument of data that stores on current_frame under data_idx index */
Eigen::Vector3d Segmentation::estimate_derivative(
        int data_idx,
        Eigen::Vector3d (Segmentation::*function)(Eigen::Vector2d, Eigen::Vector2d)) {

        RegularFrame::point_reference prev_observation = current_frame->additionals[data_idx].get_it_on_previous();

    Vector3d cur = (this->*function) (current_frame->image_points_left[data_idx], current_frame->image_points_right[data_idx]);
    Vector3d prev = (this->*function) (prev_observation.get_image_point_left(), prev_observation.get_image_point_right());

    /* switch age : case 1: case 2 etc; now only for age = 1(2 times observed) */
    return cur - prev;
}

Eigen::Vector3d Segmentation::getX(Eigen::Vector2d left, Eigen::Vector2d right) {
    double disparity = fmax(left[0] - right[0], 0.001);

    return Vector3d(
            (left[0] - cu) * base / disparity,
            (left[1] - cv) * base / disparity,
            base * focal / disparity);
}

Eigen::Vector3d Segmentation::getdXduL(Eigen::Vector2d left, Eigen::Vector2d right) {
    double d = fmax(left[0] - right[0], 0.001);

    return Vector3d(
            (cu - right[0]) * base / (d * d),
            (left[1] - cv) * base / (d * d),
            base * focal / (d * d));
}

Eigen::Vector3d Segmentation::getdXduR(Eigen::Vector2d left, Eigen::Vector2d right) {
    double d = fmax(left[0] - right[0], 0.001);

    return Vector3d(
            - (left[0] - cu) * base / (d * d),
            - (left[1] - cv) * base / (d * d),
            - base * focal / (d * d));
}

Eigen::Vector3d Segmentation::getdXdv(Eigen::Vector2d left, Eigen::Vector2d right) {
    double d = fmax(left[0] - right[0], 0.001);

    return Vector3d(
            0,
            base / d,
            0);
}
