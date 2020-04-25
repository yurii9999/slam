#include "segmentation.h"

#include <vector>
#include <eigen3/Eigen/Core>

using Eigen::Vector3d;
using std::vector;


Segmentation::Segmentation(double focal, double cu, double cv, double base)
{
    this->focal = focal;
    this->cu = cu;
    this->cv = cv;
    this->base = base;
}

void Segmentation::exec(RegularFrame &current) {
    derivatives.clear();
    derivatives.resize(current.additionals.size());

    for (auto p : current.additionals) {
        RegularFrame::point_reference prev_observation = p.get_it_on_previous();

        Vector3d cur = triangulate(current.image_points_left[p.index], current.image_points_right[p.index]);
        Vector3d prev = triangulate(prev_observation.get_image_point_left(), prev_observation.get_image_point_right());

        derivatives[p.index] = cur - prev;
    }
}

Eigen::Vector3d Segmentation::triangulate(Eigen::Vector2d left, Eigen::Vector2d right) {
    double disparity = fmax(left[0] - right[0], 0.001);

    return Vector3d(
            (left[0] - cu) * base / disparity,
            (left[1] - cv) * base / disparity,
            base * focal / disparity);
}
