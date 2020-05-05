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

    // I will try some graph representations
    // Now it is vector of vertex with each corresponded vector of neighbors
    class Graph {
    public:
        // Output of delaunay-triangulation is vector of edges that holds ids of A and B
        // In this constructor Graph transforms edges to its representation
        // Assumed, that edges with width more than threshold already removed
        void update(vector<edge> &edges, int amount_vertexs);

        // returns connected components
        void get_components();
        vector<vector<int>> components;

    private:
        void dfs(int idx);

        vector<vector<int>> data;

        vector<int> c_vertex;
        int remains_vertex;

        vector<int> c_component;
    };

    Segmentation(double focal, double cu, double cv, double base, double threshold);

    void exec(RegularFrame &current);

    double threshold;
    double second_th;

    vector<Vector3d> derivatives;

    vector<Vector3d> derivatives_ul;
    vector<Vector3d> derivatives_ur;
    vector<Vector3d> derivatives_v;

    vector<Matrix3d> Jacobians;

    RegularFrame *current_frame;

    Graph graph;

    vector<vector<int>> components;


private:
    Vector3d getX(Vector2d left, Vector2d right);

    Vector3d getdXduL(Vector2d left, Vector2d right);
    Vector3d getdXduR(Vector2d left, Vector2d right);
    Vector3d getdXdv(Vector2d left, Vector2d right);

    void estimate_derivatives();
    void estimate_additional_derivatives();

    void build_jacobians();

    void build_graph();

    /* fuse components to one point with average values through all elements of each component */
    void second_refiment();

    Vector3d estimate_derivative(
            int data_idx,
            Vector3d(Segmentation::*function) (Vector2d left, Vector2d right));

    double base, cu, cv, focal;
};
