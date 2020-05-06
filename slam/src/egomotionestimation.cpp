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
    temp_map.resize(current_frame_->additionals.size());

    disparities.clear();
    disparities.resize(current_frame_->additionals.size());

    opengv::bearingVectors_t b1(current_frame_->additionals.size());
    opengv::bearingVectors_t b2(current_frame_->additionals.size());

    for (auto p : current_frame_->additionals) {
        RegularFrame::point_reference prev = p.get_it_on_previous();
//        b1[p.index] = prev.get_bearing_vector_left();
//        b2[p.index] = prev.get_bearing_vector_right();


        Vector2d l = prev.get_image_point_left();
        Vector2d r = prev.get_image_point_right();

        double disparity = fmax(l[0] - r[0], 0.001);

        disparities[p.index] = disparity;
        temp_map[p.index] = Vector3d(
                    (l[0] - cu) * base / disparity,
                    (l[1] - cv) * base / disparity,
                    base * focal / disparity
                );
    }

//    opengv::relative_pose::CentralRelativeAdapter adapter_triangulation(b1, b2, camOffsets[1], camRotations[1]);

//    for (auto p : current_frame_->additionals)
//        temp_map[p.index] = opengv::triangulation::triangulate(adapter_triangulation, p.index);

}

Sophus::SE3d EgomotionEstimation::estimate_motion(RegularFrame &curr, RegularFrame &prev) {
    vector<int> indeces(curr.additionals.size());
    for (int i = 0; i < indeces.size(); i++)
        indeces[i] = i;

    return estimate_motion(curr, prev, indeces);
}

Sophus::SE3d EgomotionEstimation::estimate_motion(RegularFrame &current_frame, RegularFrame &previous_frame, vector<int> indeces) {
    active_indeces = indeces;
    current_frame_ = &current_frame;
    previous_frame_ = &previous_frame;

    /* compute bearing vectors */
    previous_frame.compute_bearing_vectors(focal, cu, cv);
    current_frame.compute_bearing_vectors(focal, cu, cv);

    /* triangulation */
    triangulate_current_frame();

    /* point selection */
    select_points();

    delta = estimate_relative_motion();

    /* determine inliers */
    switch (conf.inliers_determination_policy_) {
    case configuration::REPROJECTION_ERROR:
        determine_inliers_reprojection_error();
        break;
    case configuration::ANGEL_BETWEEN_RAYS:
        determine_inliers();
        break;
    }

    return delta;
}

Sophus::SE3d EgomotionEstimation::estimate_relative_motion() {
    /* build opengv adapter */
    opengv::bearingVectors_t bearing_vectors(selection.size());
    opengv::points_t points(selection.size());

    int i = 0;
    for (auto p : selection) {
        bearing_vectors[i] = p.get_bearing_vector_left();

        points[i] = temp_map[p.index];
        i++;
    }

    opengv::absolute_pose::CentralAbsoluteAdapter adapter(bearing_vectors, points);

    std::shared_ptr<sac_problem> absposeproblem_ptr(new sac_problem(adapter));

    ransac.problem = absposeproblem_ptr;
    ransac.computeModel();

    opengv::transformation_t t = ransac.model_coefficients_;

    return SE3d(t.block<3,3>(0,0), t.col(3));
}

/* determine points that in both frames(L & R) fit in motion model */
void EgomotionEstimation::determine_inliers() {
    residual_l.clear();
    residual_r.clear();

    residual_l.resize(current_frame_->additionals.size());
    residual_r.resize(current_frame_->additionals.size());

    inliers.clear();

    Sophus::SE3d deltaInverse = delta.inverse();

    for (auto p : current_frame_->additionals) {
        Vector3d estimated_bv_left = deltaInverse * temp_map[p.index];
        estimated_bv_left.normalize(); /* vector that should be close to corresponded bearing vector on the left camera */
        Vector3d estimated_bv_right = deltaInverse * temp_map[p.index] - Vector3d(base, 0, 0);
        estimated_bv_right.normalize();

        double res_l = 1 - (current_frame_->bearingVectors_left[p.index]).dot(estimated_bv_left); /* = 1 - cos */
        double res_r = 1 - (current_frame_->bearingVectors_right[p.index]).dot(estimated_bv_right);

        residual_l[p.index] = res_l;
        residual_r[p.index] = res_r;

        if (abs(res_l) < conf.final_th /*&& abs(res_r) < conf.final_th*/)
            inliers.push_back(p.index);
    }
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

void EgomotionEstimation::select_points()
{
    selection.clear();
    selection.reserve(active_indeces.size());

    for (auto idx : active_indeces)
        selection.push_back(RegularFrame::point_reference(current_frame_, idx));
}
