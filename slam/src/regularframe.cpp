#include "regularframe.h"

RegularFrame::RegularFrame()
{

}

void RegularFrame::push_back(int i1, int i2, float u1, float v1, float u2, float v2, const RegularFrame::StablePoint &previous)
{
    int age = previous.age + 1;
    int buffer_index = previous.buffer->size(); /* немного подругому надо */
    int index = points.size();
    previous.buffer->push_back(index);
    points.push_back(
                StablePoint(
                    i1, i2,
                    Vector2f(u1, v1),
                    Vector2f(u2, v2),
                    age,
                    previous.reliability,
                    previous.buffer,
                    buffer_index)
                );

}

void RegularFrame::push_back(int i1, int i2, float u1, float v1, float u2, float v2) {
    int age = 0;
    int buffer_index = 0;
    shared_ptr<circular_buffer<int>> buff(new circular_buffer<int>(64));
    int index = points.size();
    buff->push_back(index);
    points.push_back(
                StablePoint(
                    i1, i2,
                    Vector2f(u1, v1),
                    Vector2f(u2, v2),
                    age,
                    0,
                    buff,
                    buffer_index)
                );
}
