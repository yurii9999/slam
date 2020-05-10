#include "segmentation.h"

#include <vector>
#include <stack>
#include <eigen3/Eigen/Core>

#include "dt/delaunay.h"
#include "dt/edge.h"
#include "dt/vector2.h"

using Eigen::Vector3d;
using std::vector;
using std::stack;

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
    vector<int> indices;

    for (auto p : current.additionals)
        indices.push_back(p.index);

    exec(current, indices);
}

void Segmentation::exec(RegularFrame &current, vector<int> &indices) {
    if (indices.size() < 3)
        return;
    current_frame = &current;
    active_points = indices;

    // estimate via finite differences
    estimate_derivatives();

    // build jacobians
    build_jacobians();

    // build graph:
    // delaunay triangulation & weighted each edge (weight = || a.derivative - b.derivative || )
    build_graph();

    // compute connected components
    graph.get_components();
    components = graph.components;

    /* in graph used pseudonyms (name = index of name in active_points) */
    for (auto &component : components)
        for (auto &element : component)
            element = active_points[element];

}

void Segmentation::estimate_derivatives() {
    derivatives.clear();
    derivatives.resize(active_points.size());

    Covariances.clear();
    Covariances.resize(active_points.size());

    for (int i = 0; i < active_points.size(); ++i) {
        derivatives[i] = estimate_derivative(active_points[i], &Segmentation::getX);
    }
//    for (int i = 0; i < active_points.size(); ++i) {
//        Vector2d l = current_frame->image_points_left[active_points[i]];
//        Vector2d r = current_frame->image_points_right[active_points[i]];

//        RegularFrame::point_reference prev_observation = current_frame->additionals[active_points[i]].get_it_on(1);
//        Vector2d l_p = prev_observation.get_image_point_left();
//        Vector2d r_p = prev_observation.get_image_point_right();

//        Matrix3d Jacobian;
//        Jacobian.col(0) = getdXduL(l, r);
//        Jacobian.col(1) = getdXduR(l, r);
//        Jacobian.col(2) = getdXdv(l, r);

//        Vector3d dudt(l[0] - l_p[0], r[0] - r_p[0], l[1] - l_p[1]);

//        derivatives[i] = Jacobian * dudt;
//    }
}


void Segmentation::build_jacobians() {
    Covariances.clear();
    Covariances.resize(active_points.size());
    for (int i = 0 ; i < active_points.size(); i++) {
        int order = min(current_frame->additionals[active_points[i]].age + 1, 2);

        vector<Matrix3d> covs(order);

        for (int j = 0; j < order; j++) {
            Matrix3d jacobianX1;
            RegularFrame::point_reference prev = current_frame->additionals[active_points[i]].get_it_on(j);
            jacobianX1.col(0) = getdXduL(prev.get_image_point_left(), prev.get_image_point_right());
            jacobianX1.col(1) = getdXduR(prev.get_image_point_left(), prev.get_image_point_right());
            jacobianX1.col(2) = getdXdv(prev.get_image_point_left(), prev.get_image_point_right());

            Matrix3d covX1 = jacobianX1 * jacobianX1.transpose();
            covs[j] = covX1;
        }

        switch (order) {
        case 6:
//            Covariances[i] = pow(137./60, 2) * covs[0] + pow(5, 2) * covs[1] + pow(5, 2) * covs[2] + pow(10./3, 2) * covs[3] + pow(5./4, 2) * covs[4] + pow(1./5, 2) * covs[5];
            Covariances[i] = (covs[0] + covs[5]) / 25;
            break;
        case 5:
//            Covariances[i] = pow(25./12, 2) * covs[0] + pow(4, 2) * covs[1] + pow(3, 2) * covs[2] + pow(4./3, 2) * covs[3] + pow(1./4, 2) * covs[4];
            Covariances[i] = (covs[0] + covs[4]) / 16;
            break;
        case 4:
//            Covariances[i] = pow(11./6, 2) * covs[0] + pow(3, 2) * covs[1] + pow(3./2, 2) * covs[2] + pow(1./3, 2) * covs[3];
            Covariances[i] = (covs[0] + covs[3]) / 9;
            break;
        case 3:
//            Covariances[i] = pow(3./2, 2) * covs[0] + pow(2, 2) * covs[1] + pow(1./2, 2) * covs[2];
            Covariances[i] = (covs[0] + covs[2]) / 4;
            break;
        case 2:
            Covariances[i] = covs[0] + covs[1];
            break;
        }

//        Matrix3d covX0 = jacobianX0 * jacobianX0.transpose();

//        Covariances[i] = covX0 + covX1;
    }
}

void Segmentation::build_graph() {
    /* wrap for https://github.com/Bl4ckb0ne/delaunay-triangulation with little correction (with Vector2 associatet id) */
    vector<dt::Vector2<double>> points;
    for (int i = 0 ; i < active_points.size(); i++) {
        points.push_back(dt::Vector2<double>(
                            current_frame->image_points_left[active_points[i]][0],
                            current_frame->image_points_left[active_points[i]][1],
                            i));
    }

    dt::Delaunay<double> triangulation;
    triangulation.triangulate(points);


    vector<dt::Edge<double>> edges_dt = triangulation.getEdges();


    vector<edge> edges;
    for (auto &e : edges_dt) {
        int idx_a = e.v->p_index;
        int idx_b = e.w->p_index;

        Matrix3d cov = Covariances[idx_a] + Covariances[idx_b];
//        cout << "covariance: " << endl << cov << endl;
//        cout << "covariance inverse: " << endl << cov.inverse() << endl;

        Vector3d delta = derivatives[idx_a] - derivatives[idx_b];
        double difference = sqrt(delta.transpose() * cov.inverse() * delta);
//        difference = delta.norm();



        if (difference < threshold) {
            edges.push_back(edge(idx_a, idx_b, difference));
        }
    }

    graph.update(edges, active_points.size());
}


/* estimate derivateve of function with pair of image-point coordinates as argument of data that stores on current_frame under data_idx index */
Eigen::Vector3d Segmentation::estimate_derivative(
        int data_idx,
        Eigen::Vector3d (Segmentation::*function)(Eigen::Vector2d, Eigen::Vector2d)) {
    int order = min(current_frame->additionals[data_idx].age + 1, 2); // order of finite difference
    vector<Vector3d> o(order); // observations

    o[0] = (this->*function) (current_frame->image_points_left[data_idx], current_frame->image_points_right[data_idx]);
//    cout << "=====" << endl;
//    cout << "Order: " << order << endl;
//    cout << current_frame->image_points_left[data_idx].transpose() << "\t";
//    cout << current_frame->image_points_right[data_idx].transpose() << endl;
//    cout << (this->*function) (current_frame->image_points_left[data_idx], current_frame->image_points_right[data_idx]).transpose() << endl;

    for (int i = 1; i < order; i++) {
        RegularFrame::point_reference prev_observation = current_frame->additionals[data_idx].get_it_on(i);
//        cout << prev_observation.get_image_point_left().transpose() << "\t";
//        cout << prev_observation.get_image_point_right().transpose() << endl;
        o[i] = (this->*function) (prev_observation.get_image_point_left(), prev_observation.get_image_point_right());
//        cout << (this->*function) (prev_observation.get_image_point_left(), prev_observation.get_image_point_right()).transpose() << endl;
    }

    Vector3d res;



    switch (order) {
    case 6:
//        res = 137./60 * o[0] - 5 * o[1] + 5 * o[2] - 10./3 * o[3] + 5./4 * o[4] - 1./5 * o[5];
        res = (o[0] - o[5]) / 5;
        break;
    case 5:
//        res = 25./12 * o[0] - 4 * o[1] + 3 * o[2] - 4./3 * o[3] + 1./4 * o[4];
        res = (o[0] - o[4]) / 4;
        break;
    case 4:
//        res = 11./6 * o[0] - 3 * o[1] + 3./2 * o[2] - 1./3 * o[3];
        res = (o[0] - o[3]) / 3;
        break;
    case 3:
//        res = 3./2 * o[0] - 2 * o[1] + 1./2 * o[2];
        res = (o[0] - o[2]) / 2;
        break;
    case 2:
        res = o[0] - o[1];
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
                (cv - left[1]) * base / (d * d),
                - base * focal / (d * d));
}

Eigen::Vector3d Segmentation::getdXduR(Eigen::Vector2d left, Eigen::Vector2d right) {
    double d = fmax(left[0] - right[0], 0.001);

    return Vector3d(
            (left[0] - cu) * base / (d * d),
            (left[1] - cv) * base / (d * d),
            base * focal / (d * d));
}

Eigen::Vector3d Segmentation::getdXdv(Eigen::Vector2d left, Eigen::Vector2d right) {
    double d = fmax(left[0] - right[0], 0.001);

    return Vector3d(
            0,
            base / d,
            0);
}

// Graph
void Segmentation::Graph::update(vector<Segmentation::edge> &edges, int amount_vertexs) {
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
    stack<int> to_visit;

    to_visit.push(idx);
    c_vertex[idx] = -1;
    --remains_vertex;
    c_component.push_back(idx);

    while (!to_visit.empty()) {
        int cur_vertex = to_visit.top();
        to_visit.pop();

        for (auto next_v : data[cur_vertex])
            if (c_vertex[next_v] != -1) {
                c_component.push_back(next_v);
                to_visit.push(next_v);
                c_vertex[next_v] = -1;
                --remains_vertex;
            }
    }
}
