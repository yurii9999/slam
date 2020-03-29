#include "regularframe.h"

RegularFrame::RegularFrame()
{

}

void RegularFrame::push_back(int i1, int i2,
                             Eigen::Vector3d bearing_vector_left,
                             Eigen::Vector3d bearing_vector_right,
                             const RegularFrame::StablePoint &previous)
{
    int age = previous.age + 1;
    int buffer_index = previous.buffer->size(); /* немного подругому надо */
    int index = points.size();
    previous.buffer->push_back(index);
    points.push_back(
                StablePoint(
                    i1, i2,
                    bearing_vector_left,
                    bearing_vector_right,
                    age,
                    previous.reliability,
                    previous.buffer,
                    buffer_index)
                );

}

void RegularFrame::push_back(int i1, int i2,
                             Eigen::Vector3d bearing_vector_left,
                             Eigen::Vector3d bearing_vector_right) {
    int age = 0;
    int buffer_index = 0;
    shared_ptr<circular_buffer<int>> buff(new circular_buffer<int>(64));
    int index = points.size();
    buff->push_back(index);
    points.push_back(
                StablePoint(
                    i1, i2,
                    bearing_vector_left,
                    bearing_vector_right,
                    age,
                    0,
                    buff,
                    buffer_index)
                );
}
