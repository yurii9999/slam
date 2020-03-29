#pragma once

#include <vector>
#include <boost/circular_buffer.hpp>
#include <eigen3/Eigen/Core>
#include <memory>

using std::vector;
using boost::circular_buffer;
using std::shared_ptr;
using Eigen::Vector3d;

class RegularFrame {
public:
    struct StablePoint {
        int index1;
        int index2;

        Vector3d p_left;
        Vector3d p_right;

        int age;
        int reliability;

        shared_ptr<circular_buffer<int>> buffer;
        int buffer_index;

        /* other additional information, as class, value etc */

        StablePoint(
                int i1, int i2,
                Vector3d p_left,
                Vector3d p_right,
                int age,
                int reliability,
                shared_ptr<circular_buffer<int>> buffer,
                int buffer_index /* is it same thing as age? */
                ):
            index1(i1), index2(i2),
            p_left(p_left), p_right(p_right),
            age(age),
            reliability(reliability),
            buffer(buffer),
            buffer_index(buffer_index)
        {}

        void increaseReliability() {
            reliability++;
        }
    };

    RegularFrame();

//private:
    // const Match::getFeatures ();
    struct Snapshot {
    };

    /* complete stable point from previous */
    void push_back(int i1, int i2,
                   Eigen::Vector3d bearing_vector_left,
                   Eigen::Vector3d bearing_vector_right,
                   const RegularFrame::StablePoint &previous);
    /* initializate stable point */
    void push_back(int i1, int i2,
                   Eigen::Vector3d bearing_vector_left,
                   Eigen::Vector3d bearing_vector_right);

    vector<StablePoint> points;
    Snapshot snapshot;
};
