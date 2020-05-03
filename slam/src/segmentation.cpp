#include "segmentation.h"

#include <vector>
#include <eigen3/Eigen/Core>

#include "dt/delaunay.h"
#include "dt/edge.h"
#include "dt/vector2.h"

using Eigen::Vector3d;
using std::vector;

using namespace std;


Segmentation::Segmentation(double focal, double cu, double cv, double base, double threshold)
{
    this->focal = focal;
    this->cu = cu;
    this->cv = cv;
    this->base = base;
    this->threshold = threshold;
}

void Segmentation::exec(RegularFrame &current) {
    current_frame = &current;

    // estimate via finite differences
    estimate_derivatives();

    // build jacobians from partial derivatives that were compute on previous step
    build_jacobians();

    // build graph:
    // delaunay triangulation & weighted each edge (weight = || a.derivative - b.derivative || )
    build_graph();


    // get connected components
    graph.get_components();
}

void Segmentation::estimate_derivatives() {
    derivatives.clear();
    derivatives.resize(current_frame->additionals.size());

    for (auto p : current_frame->additionals)
        derivatives[p.index] = estimate_derivative(p.index, &Segmentation::getX);
}


void Segmentation::build_jacobians() {
    Jacobians.clear();
    Jacobians.resize(current_frame->additionals.size());

    for (auto p : current_frame->additionals) {
        Matrix3d a;
        a.col(0) = estimate_derivative(p.index, &Segmentation::getdXduL);
        a.col(1) = estimate_derivative(p.index, &Segmentation::getdXduR);
        a.col(2) = estimate_derivative(p.index, &Segmentation::getdXdv);

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


    vector<dt::Edge<double>> edges_dt = triangulation.getEdges();

    vector<edge> edges;
    edges.clear();
    edges.reserve(current_frame->additionals.size());
    for (auto e : edges_dt) {
        int idx_a = e.v->p_index;
        int idx_b = e.w->p_index;


        Matrix3d J_ij = Jacobians[idx_a] - Jacobians[idx_b];
        Vector3d delta = derivatives[idx_a] - derivatives[idx_b];
        Matrix3d covariace_inv = J_ij * J_ij.transpose(); // hmmm
//        double difference = sqrt(delta.transpose() * covariace_inv * delta); // hmmm transpose \ netranspose
        double difference = (derivatives[idx_a] - derivatives[idx_b]).norm();

        if (difference < threshold)
            edges.push_back(edge(idx_a, idx_b, difference));

        graph.update(edges, current_frame->additionals.size());



//        cout << "====================================" << endl;
//        cout << "New edge: " << endl;
//        cout << "Idx A: " << idx_a << "\tIdx B: " << idx_b << endl;
//        cout << "Image point A: " << current_frame->image_points_left[idx_a].transpose() << "\t" << current_frame->image_points_right[idx_a].transpose() << endl;
//        cout << "Image point B: " << current_frame->image_points_left[idx_b].transpose() << "\t" << current_frame->image_points_right[idx_b].transpose() << endl;
//        Vector3d A = getX(current_frame->image_points_left[idx_a], current_frame->image_points_right[idx_a]);
//        Vector3d B = getX(current_frame->image_points_left[idx_b], current_frame->image_points_right[idx_b]);
//        cout << "VelA: " << derivatives[idx_a].transpose() << endl;
//        cout << "VelB: " << derivatives[idx_b].transpose() << endl;

//        cout << "JacobianA: \n" << Jacobians[idx_a] << endl;
//        cout << "JacobianB: \n" << Jacobians[idx_b] << endl;
//        cout << "Covariance: \n" << covariace << endl;
//        cout << "Covariance inversed: \n" << covariace.inverse() << endl;
//        cout << "Result: " << difference << endl;


//        --input=../../sequence --seg_th=0.01


    }
}

/* estimate derivateve of function with pair of image-point coordinates as argument of data that stores on current_frame under data_idx index */
Eigen::Vector3d Segmentation::estimate_derivative(
        int data_idx,
        Eigen::Vector3d (Segmentation::*function)(Eigen::Vector2d, Eigen::Vector2d)) {
    int order = min(current_frame->additionals[data_idx].age + 1, 6); // order of finite difference
    vector<Vector3d> o(order); // observations

    o[0] = (this->*function) (current_frame->image_points_left[data_idx], current_frame->image_points_right[data_idx]);

    for (int i = 1; i < order; i++) {
        RegularFrame::point_reference prev_observation = current_frame->additionals[data_idx].get_it_on(i);
        o[i] = (this->*function) (prev_observation.get_image_point_left(), prev_observation.get_image_point_right());
    }

    Vector3d res;

    switch (order) {
    case 2:
        res = o[0] - o[1];
        break;
    case 3:
        res = 3./2 * o[0] - 2 * o[1] + 1./2 * o[2];
        break;
    case 4:
        res = 11./6 * o[0] - 3 * o[1] + 3./2 * o[2] - 1./3 * o[3];
        break;
    case 5:
        res = 25./12 * o[0] - 4 * o[1] + 3 * o[2] - 4./3 * o[3] + 1./4 * o[4];
        break;
    case 6:
        res = 137./60 * o[0] - 5 * o[1] + 5 * o[2] - 10./3 * o[3] + 5./4 * o[4] - 1./5 * o[5];
        break;
    }

    return res;
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

// Graph
void Segmentation::Graph::update(vector<Segmentation::edge> edges, int amount_vertexs) {
    data.clear();
    data.resize(amount_vertexs);

    for (auto e : edges) {
        data[e.a_index].push_back(e.b_index);
        data[e.b_index].push_back(e.a_index);
    }
}

void Segmentation::Graph::get_components() {
    c_vertex.clear();
    c_vertex.resize(data.size());

    for (int i = 0; i < data.size(); i++)
        c_vertex[i] = i;

    remains_vertex = data.size();
    components.clear();

    for (int i = 0; i < data.size(); i++) {
        if (c_vertex[i] != -1) {
            c_component.clear();
            dfs(i);
            components.push_back(c_component);
        }
    }
}

void Segmentation::Graph::dfs(int idx) {
    if (c_vertex[idx] == -1)
        return;

    c_vertex[idx] = -1;
    --remains_vertex;
    c_component.push_back(idx);

    for (int next_v : data[idx])
        dfs(next_v);
}
