#include "egomotionestimation.h"

#include <math.h>

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

using namespace std;

void EgomotionEstimation::triangulate_current_frame() {
    temp_map.clear();
    temp_map.resize(current_frame_->additionals.size());

    opengv::bearingVectors_t b1(current_frame_->additionals.size());
    opengv::bearingVectors_t b2(current_frame_->additionals.size());

    for (auto p : current_frame_->additionals) {
        RegularFrame::point_reference prev = p.get_it_on_previous();
        b1[p.index] = prev.get_bearing_vector_left();
        b2[p.index] = prev.get_bearing_vector_right();
    }

    opengv::relative_pose::CentralRelativeAdapter adapter_triangulation(b1, b2, camOffsets[1], camRotations[1]);

    for (int i = 0; i < current_frame_->additionals.size(); i++)
        temp_map[i] = opengv::triangulation::triangulate(adapter_triangulation, i);
}

void EgomotionEstimation::estimate_motion(RegularFrame &current_frame, RegularFrame &previous_frame) {
    current_frame_ = &current_frame;
    previous_frame_ = &previous_frame;

    /* compute bearing vectors */
    previous_frame.compute_bearing_vectors(focal, cu, cv);
    current_frame.compute_bearing_vectors(focal, cu, cv);

    /* triangulation */
    triangulate_current_frame();

    /* point selection */
    switch (conf.selection_policy_) {
    case configuration::SELECT_ALL:
        select_all_points();
        break;
    case configuration::SELECT_CLOSE:
        select_points_closer_than();
        break;
    case configuration::SELECT_FAR:
        select_points_farther_than();
        break;
    }

    delta = estimate_relative_motion_A_();

    /* determine inliers */
    switch (conf.inliers_determination_policy_) {
    case configuration::REPROJECTION_ERROR:
        determine_inliers_reprojection_error();
        break;
    case configuration::ANGEL_BETWEEN_RAYS:
        determine_inliers();
        break;
    }

    current_frame_->set_motion(previous_frame_->motion * delta);
}

Sophus::SE3d EgomotionEstimation::estimate_relative_motion_A_() {
    opengv::absolute_pose::NoncentralAbsoluteAdapter *adapter = build_adapter();

    std::shared_ptr<
        opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem> absposeproblem_ptr(
        new opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem(
        *adapter,
        opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem::GP3P));
    ransac.sac_model_ = absposeproblem_ptr;

    ransac.computeModel();

    amount_inliers = ransac.inliers_.size();

    opengv::transformations_t ts = opengv::absolute_pose::upnp(*adapter, ransac.inliers_);
    opengv::transformation_t t = ts[0];

    if (conf.using_nonlinear_optimization) {
        adapter->setR(t.block<3,3>(0,0));
        adapter->sett(t.col(3));
        t = opengv::absolute_pose::optimize_nonlinear(*adapter, ransac.inliers_);
    }

    delete adapter;

    return SE3d(t.block<3,3>(0,0), t.col(3));
}

/* building opengv ' s adapter */
opengv::absolute_pose::NoncentralAbsoluteAdapter *EgomotionEstimation::build_adapter()
{
    bearing_vectors.clear();
    bearing_vectors.resize(2 * selection.size());

    points.clear();
    points.resize(2 * selection.size());

    int i = 0;
    for (auto v : selection) {
        bearing_vectors[i] = v.get_bearing_vector_left();
        bearing_vectors[selection.size() + i] = v.get_bearing_vector_right();

        points[i] = temp_map[v.index];
        points[selection.size() + i] = temp_map[v.index];
        i++;
    }

    camCorrespondence.clear();
    camCorrespondence.resize(2 * selection.size());
    for (int i = 0; i < selection.size(); i++) {
        camCorrespondence[i] = 0;
        camCorrespondence[selection.size() + i] = 1;
    }

    return new opengv::absolute_pose::NoncentralAbsoluteAdapter(
                bearing_vectors,
                camCorrespondence,
                points,
                camOffsets,
                camRotations);
}

/* determine points that in both frames(L & R) fit in motion model */
void EgomotionEstimation::determine_inliers() {
    residual_l.clear();
    residual_r.clear();

    residual_l.resize(current_frame_->additionals.size());
    residual_r.resize(current_frame_->additionals.size());

    inliers.clear();
    for (auto p : current_frame_->additionals) {
        Vector3d estimated_bv_left = delta.inverse() * temp_map[p.index];
        estimated_bv_left.normalize(); /* vector that shold be close to corresponded bearing vector on the left camera */
        Vector3d estimated_bv_right = delta.inverse() * temp_map[p.index] - camOffsets[1]; /* now camRotations[1] = I */
        estimated_bv_right.normalize();

        double res_l = (current_frame_->bearingVectors_left[p.index].cross(estimated_bv_left)).norm(); /* = sin between bearing vector and estimatedbearing vector */
        double res_r = (current_frame_->bearingVectors_right[p.index].cross(estimated_bv_right)).norm();
        residual_l[p.index] = res_l;
        residual_r[p.index] = res_r;
        if (abs(res_l) < conf.final_th && abs(res_r) < conf.final_th)
            inliers.push_back(p.index);
    }
}

void EgomotionEstimation::determine_inliers_reprojection_error() {
    residual_l.clear();
    residual_r.clear();

    residual_l.resize(current_frame_->additionals.size());
    residual_r.resize(current_frame_->additionals.size());

    inliers.clear();

    for (auto p : current_frame_->additionals) {
        Vector3d estimated_bv_left = delta.inverse() * temp_map[p.index];
        Vector2d reprojection_left(
                    (estimated_bv_left[0] / estimated_bv_left[2] ) * focal + cu,
                    (estimated_bv_left[1] / estimated_bv_left[2] ) * focal + cv);

        Vector3d estimated_bv_right = delta.inverse() * temp_map[p.index] - camOffsets[1]; /* now camRotations[1] = I */
        Vector2d reprojection_right(
                    (estimated_bv_right[0] / estimated_bv_right[2] ) * focal + cu,
                    (estimated_bv_right[1] / estimated_bv_right[2] ) * focal + cv);

        double res_l = abs((current_frame_->image_points_left[p.index] - reprojection_left).norm());
        double res_r = abs((current_frame_->image_points_right[p.index] - reprojection_right).norm());
        residual_l[p.index] = res_l;
        residual_r[p.index] = res_r;
        if (abs(res_l) < conf.final_th && abs(res_r) < conf.final_th)
            inliers.push_back(p.index);
    }


}

void EgomotionEstimation::select_all_points()
{
    selection.clear();
    selection.reserve(current_frame_->additionals.size());

    for (auto p : current_frame_->additionals)
        selection.push_back(RegularFrame::point_reference(current_frame_, p.index));
}

void EgomotionEstimation::select_points_closer_than() {
    selection.clear();
    selection.reserve(current_frame_->additionals.size());

    for (auto p : current_frame_->additionals) {
        if (temp_map[p.index][2] < conf.farthest_coeff)
            selection.push_back(RegularFrame::point_reference(current_frame_, p.index));
    }
}

void EgomotionEstimation::select_points_farther_than() {
    selection.clear();
    selection.reserve(current_frame_->additionals.size());

    for (auto p : current_frame_->additionals) {
        if (temp_map[p.index][2] > conf.farthest_coeff)
            selection.push_back(RegularFrame::point_reference(current_frame_, p.index));
    }
}
