#include "regularframe.h"

//RegularFrame::RegularFrame()
//{
//}

void RegularFrame::push_back(int i1, int i2,
                             Vector2d point_left,
                             Vector2d point_right,
                             const point_reference &previous)
{
    image_points_left.push_back(point_left);
    image_points_right.push_back(point_right);

    feature_additional curr_addi;
    feature_additional prev_addi = previous.getAdditional();

    curr_addi.index_left = i1;
    curr_addi.index_right = i2;
    curr_addi.index = additionals.size();
    curr_addi.age = prev_addi.age + 1;
    curr_addi.common = prev_addi.common;
    curr_addi.common->buffer.push_back(point_reference(this, curr_addi.index));

//    curr_addi.depth
//    curr_addi.disparity
//    curr_addi.reliability

    additionals.push_back(curr_addi);
}

void RegularFrame::push_back(int i1, int i2,
                             Vector2d point_left,
                             Vector2d point_right) {

    image_points_left.push_back(point_left);
    image_points_right.push_back(point_right);

    feature_additional curr_addi;

    curr_addi.index_left = i1;
    curr_addi.index_right = i2;
    curr_addi.index = additionals.size();
    curr_addi.age = 0;
    curr_addi.common = shared_ptr<PointCommon>(new PointCommon());
    curr_addi.common->buffer.push_back(point_reference(this, curr_addi.index));

    additionals.push_back(curr_addi);
}

void RegularFrame::compute_bearing_vectors(double focal, double cu, double cv)
{
    if (bearingVectors_left.size() == image_points_left.size())
        return;

    bearingVectors_left.reserve(image_points_left.size());
    bearingVectors_right.reserve(image_points_right.size());

    for (int i = bearingVectors_left.size(); i < image_points_left.size(); i++) {
        Vector2d &p_l = image_points_left[i];
        Vector2d &p_r = image_points_right[i];

        Vector3d b_l((p_l[0] - cu) / focal, (p_l[1] - cv) / focal, 1);
        Vector3d b_r((p_r[0] - cu) / focal, (p_r[1] - cv) / focal, 1);

        bearingVectors_left.push_back(b_l / b_l.norm());
        bearingVectors_right.push_back(b_r / b_r.norm());
    }
}
