#include "egomotionestimation.h"

#include <math.h>

#include <algorithm>

#include "opengv/absolute_pose/CentralAbsoluteAdapter.hpp"
#include "opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp"
#include "opengv/sac/Ransac.hpp"
#include "opengv/absolute_pose/NoncentralAbsoluteAdapter.hpp"
#include "opengv/absolute_pose/methods.hpp"

#include <opengv/sac/Lmeds.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>

#include <opengv/sac_problems/relative_pose/RotationOnlySacProblem.hpp>

#include <opengv/relative_pose/methods.hpp>

#include <opengv/triangulation/methods.hpp>

#include "loransac/sac_problem.h"

using namespace std;

void EgomotionEstimation::triangulate_current_frame() {
    temp_map.clear();
    temp_map.resize(active_indices.size());

    for (int i = 0; i < active_indices.size(); i++) {
        RegularFrame::point_reference prev = current_frame_->additionals[active_indices[i]].get_it_on_previous();
        Vector2d l = prev.get_image_point_left();
        Vector2d r = prev.get_image_point_right();

        double disparity = fmax(l[0] - r[0], 0.001);
        temp_map[i] = Vector3d(
                    (l[0] - cu) * base / disparity,
                    (l[1] - cv) * base / disparity,
                    base * focal / disparity
                );
    }
}

void EgomotionEstimation::compute_bearing_vectors() {
    temp_bearing.clear();
    temp_bearing.resize(active_indices.size());

    for (int i = 0; i < active_indices.size(); i++) {
        Vector2d p = current_frame_->image_points_left[active_indices[i]];
        Vector3d bearing((p[0] - cu) / focal, (p[1] - cv) / focal, 1);

        temp_bearing[i] = bearing / bearing.norm();
    }
}

Sophus::SE3d EgomotionEstimation::estimate_motion(RegularFrame &curr, RegularFrame &prev) {
    vector<int> indeces(curr.additionals.size());
    for (int i = 0; i < indeces.size(); i++)
        indeces[i] = i;

    return estimate_motion(curr, prev, indeces);
}

Sophus::SE3d EgomotionEstimation::estimate_motion(RegularFrame &current_frame, RegularFrame &previous_frame, vector<int> indices) {
    active_indices = indices;
    current_frame_ = &current_frame;
    previous_frame_ = &previous_frame;

    /* compute bearing vectors */
    compute_bearing_vectors();

    /* build temp map */
    triangulate_current_frame();

    delta = estimate_relative_motion();

    return delta;
}

Sophus::SE3d EgomotionEstimation::estimate_relative_motion() {
    opengv::absolute_pose::CentralAbsoluteAdapter adapter(temp_bearing, temp_map);

    std::shared_ptr<sac_problem> absposeproblem_ptr(new sac_problem(adapter));

    ransac.problem = absposeproblem_ptr;
    ransac.computeModel();

    opengv::transformation_t t = ransac.model_coefficients_;

    return SE3d(t.block<3,3>(0,0), t.col(3));
}

/* determine points that in both frames(L & R) fit in motion model */
void EgomotionEstimation::determine_inliers() {
    residual_l.clear();
    residual_l.resize(current_frame_->additionals.size());
    inliers.clear();

    Sophus::SE3d deltaInverse = delta.inverse();
    for (int i = 0; i < active_indices.size(); i++) {
        Vector3d estimated_bv_left = deltaInverse * temp_map[i];
        estimated_bv_left.normalize(); /* vector that should be close to corresponded bearing vector on the left camera */
        double res_l = 1 - (temp_bearing[i]).dot(estimated_bv_left); /* = 1 - cos */

        residual_l[active_indices[i]] = res_l;

        if (abs(res_l) < conf.final_th /*&& abs(res_r) < conf.final_th*/)
            inliers.push_back(active_indices[i]);
    }

}

void EgomotionEstimation::determine_inliers(vector<int> points) {
    active_indices = points;

    triangulate_current_frame();
    compute_bearing_vectors();

    determine_inliers();
}

void EgomotionEstimation::determine_inliers_reprojection_error() {
    residual_l.clear();
    residual_r.clear();

    residual_l.resize(current_frame_->additionals.size());
    residual_r.resize(current_frame_->additionals.size());

    inliers.clear();

    SE3d deltaInverse = delta.inverse();
    for (auto p : current_frame_->additionals) {
        Vector3d estimated_bv_left = deltaInverse * temp_map[p.index];
        Vector2d reprojection_left(
                    (estimated_bv_left[0] / estimated_bv_left[2] ) * focal + cu,
                    (estimated_bv_left[1] / estimated_bv_left[2] ) * focal + cv);

        Vector3d estimated_bv_right = deltaInverse * temp_map[p.index] - Vector3d(base, 0, 0);
        Vector2d reprojection_right(
                    (estimated_bv_right[0] / estimated_bv_right[2] ) * focal + cu,
                    (estimated_bv_right[1] / estimated_bv_right[2] ) * focal + cv);

        double res_l = abs((current_frame_->image_points_left[p.index] - reprojection_left).norm());
        double res_r = abs((current_frame_->image_points_right[p.index] - reprojection_right).norm());
        residual_l[p.index] = res_l;
        residual_r[p.index] = res_r;
        if (abs(res_l) < conf.final_th /*&& abs(res_r) < conf.final_th*/)
            inliers.push_back(p.index);
    }
}
