#pragma once

#include <vector>
#include <boost/circular_buffer.hpp>
#include <eigen3/Eigen/Core>
#include <memory>

using std::vector;
using boost::circular_buffer;
using std::shared_ptr;
using Eigen::Vector2f;

class RegularFrame {
public:
    struct StablePoint {
        int index1;
        int index2;

        Vector2f p_left;
        Vector2f p_right;

        int age;
        int reliability;

        shared_ptr<circular_buffer<int>> buffer;
        int buffer_index;

        /* other additional information, as class, value etc */

        StablePoint(
                int i1, int i2,
                Vector2f p_left,
                Vector2f p_right,
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
    void push_back(int i1, int i2, float u1, float v1, float u2, float v2, const StablePoint &previous);
    /* initializate stable point */
    void push_back(int i1, int i2, float u1, float v1, float u2, float v2);

    vector<StablePoint> points;
    Snapshot snapshot;
};
