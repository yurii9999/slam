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
        b1[p.index] = prev.get_bearing_vector_left();
        b2[p.index] = prev.get_bearing_vector_right();

        double disparity = abs(prev.get_image_point_left()[0] - prev.get_image_point_right()[0]);
        disparities[p.index] = disparity;
    }

    opengv::relative_pose::CentralRelativeAdapter adapter_triangulation(b1, b2, camOffsets[1], camRotations[1]);

    for (int i = 0; i < current_frame_->additionals.size(); i++)
        temp_map[i] = opengv::triangulation::triangulate(adapter_triangulation, i);
}

Sophus::SE3d EgomotionEstimation::estimate_motion(RegularFrame &current_frame, RegularFrame &previous_frame) {
    current_frame_ = &current_frame;
    previous_frame_ = &previous_frame;

    /* compute bearing vectors */
    previous_frame.compute_bearing_vectors(focal, cu, cv);
    current_frame.compute_bearing_vectors(focal, cu, cv);

    /* triangulation */
    triangulate_current_frame();

    /* point selection */
    select_points();
    bucketing();

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

    return delta;
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

Sophus::SE3d EgomotionEstimation::estimate_relative_motion_B_() {
    opengv::absolute_pose::CentralAbsoluteAdapter *adapter = build_adapter_central();

    std::shared_ptr<
        opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem> absposeproblem_ptr(
        new opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem(
        *adapter,
        opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem::KNEIP));
    ransac.sac_model_ = absposeproblem_ptr;

    ransac.computeModel();

    opengv::transformations_t ts = opengv::absolute_pose::upnp(*adapter, ransac.inliers_);

    for (auto t : ts)
        std::cout << t.col(3).norm() << std::endl;

    opengv::transformation_t t = ts[0];

    if (conf.using_nonlinear_optimization) {
        adapter->setR(t.block<3,3>(0,0));
        adapter->sett(t.col(3));
        t = opengv::absolute_pose::optimize_nonlinear(*adapter, ransac.inliers_);
    }

    delete adapter;

    return SE3d(t.block<3,3>(0,0), t.col(3));
}


opengv::absolute_pose::CentralAbsoluteAdapter *EgomotionEstimation::build_adapter_central() {
    bearing_vectors.clear();
    bearing_vectors.resize(selection.size());

    points.clear();
    points.resize(selection.size());

    int i = 0;
    for (auto v : selection) {
        bearing_vectors[i] = v.get_bearing_vector_left();

        points[i] = temp_map[v.index];
        i++;
    }

    return new opengv::absolute_pose::CentralAbsoluteAdapter(
                bearing_vectors,
                points);
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
        estimated_bv_left.normalize(); /* vector that should be close to corresponded bearing vector on the left camera */
        Vector3d estimated_bv_right = delta.inverse() * temp_map[p.index] - camOffsets[1]; /* now camRotations[1] = I */
        estimated_bv_right.normalize();

        double res_l = 1 - (current_frame_->bearingVectors_left[p.index]).dot(estimated_bv_left); /* = 1 - cos */
        double res_r = 1 - (current_frame_->bearingVectors_right[p.index]).dot(estimated_bv_right);

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

void EgomotionEstimation::select_points()
{
    selection.clear();
    selection.reserve(current_frame_->additionals.size());

    for (auto p : current_frame_->additionals)
        if (disparities[p.index] < conf.far_coeff * base && disparities[p.index] > conf.close_coeff * base)
            selection.push_back(RegularFrame::point_reference(current_frame_, p.index));
}

/* reduce features from selection */
void EgomotionEstimation::bucketing() {
    int img_width = 0;
    int img_height = 0;

    for (auto p : current_frame_->image_points_left) {
        if (p[0] > img_width)
            img_width = (int) p[0];

        if (p[1] > img_height)
            img_height = (int) p[1];
    }

    std::sort(selection.begin(), selection.end(), [] (RegularFrame::point_reference a, RegularFrame::point_reference b) { return a.getAdditional().age > b.getAdditional().age; });

    int buckets_u = img_width / conf.bucketing_widht;
    int buckets_v = img_height / conf.bucketing_height;
    vector<vector<RegularFrame::point_reference>>buckets(buckets_u * buckets_v);

    for (auto p : selection) {
        int b_u = p.get_image_point_left()[0] / conf.bucketing_widht;
        int b_v = p.get_image_point_left()[1] / conf.bucketing_height;

        if (buckets[b_u + b_v * buckets_v].size() < conf.bucketing_amount)
            buckets[b_u + b_v * buckets_v].push_back(p);
    }

    selection.clear();
    selection.reserve(buckets_u * buckets_v * conf.bucketing_amount);

    for (auto bucket : buckets)
        for (auto p : bucket)
            selection.push_back(p);

}
