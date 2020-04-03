#include "egomotionestimation.h"

#include <math.h>

#include "opengv/absolute_pose/CentralAbsoluteAdapter.hpp"
#include "opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp"
#include "opengv/sac/Ransac.hpp"
#include "opengv/absolute_pose/NoncentralAbsoluteAdapter.hpp"
#include "opengv/absolute_pose/methods.hpp"

#include <opengv/sac/Lmeds.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>

using namespace std;


void EgomotionEstimation::estimate_motion(RegularFrame &current_frame, RegularFrame &previous_frame) {
    /* compute bearing vectors */
    previous_frame.compute_bearing_vectors(focal, cu, cv);
    current_frame.compute_bearing_vectors(focal, cu, cv);

    /* triangulation: in the future move it to separated methiod -- update scene */
    /* stage 0: taken from viso_stereo libviso */
    for (auto p : previous_frame.additionals) {
        Vector2d &p_l = previous_frame.image_points_left[p.index];
        Vector2d &p_r = previous_frame.image_points_right[p.index];

        double disparity = fmax(abs(p_l[0] - p_r[0]), 0.0001);

        /* now triangulate only close points */
        if (disparity < base)
            continue;

        if (disparity > p.common->disparity || !p.common->already_triangulated) {
            Vector3d X(
                    (p_l[0] - cu) * base / disparity,
                    (p_l[1] - cv) * base / disparity,
                    focal * base / disparity );

            p.common->set_point(previous_frame.motion * X, disparity);
        }
    }


    /* building opengv's adapter */
    /* use only point that is observed on current frame and is triangulated (now: <=> is not far) */
    int amount_correspondences = 0;
    for (auto v : current_frame.additionals)
        if (v.common->already_triangulated)
            amount_correspondences++;

    opengv::bearingVectors_t bearing_vectors(2 * amount_correspondences);
    opengv::points_t points(2 * amount_correspondences);

    vector<int> camCorrespondence(2 * amount_correspondences);
    for (int i = 0; i < amount_correspondences; i++) {
        camCorrespondence[i] = 0;
        camCorrespondence[amount_correspondences + i] = 1;
    }

    correspondences.clear();
    correspondences.resize(amount_correspondences);

    int i = 0;
    for (auto v : current_frame.additionals) {
        if (!v.common->already_triangulated)
            continue;

        correspondences[i] = RegularFrame::point_reference(&current_frame, v.index);

        bearing_vectors[i] = current_frame.bearingVectors_left[v.index];
        bearing_vectors[amount_correspondences + i] = current_frame.bearingVectors_right[v.index];

        points[i] = v.common->landmark;
        points[amount_correspondences + i] = v.common->landmark;

        i++;
    }

    this->amount_correspondences = amount_correspondences;


    /* motion estimation */
    opengv::absolute_pose::NoncentralAbsoluteAdapter adapter(
                bearing_vectors,
                camCorrespondence,
                points,
                camOffsets,
                camRotations);

    std::shared_ptr<
        opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem> absposeproblem_ptr(
        new opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem(
        adapter,
        opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem::GP3P));
    ransac.sac_model_ = absposeproblem_ptr;
    ransac.threshold_ = (1.0 - cos(atan(sqrt(2.0)*0.5/focal))) * 3; /* these hardcoded parameters */
    ransac.max_iterations_ = 100;

    ransac.computeModel();

    amount_inliers = ransac.inliers_.size();

    opengv::transformations_t ts = opengv::absolute_pose::upnp(adapter, ransac.inliers_);
    opengv::transformation_t t = ts[0];

    SE3d m(t.block<3,3>(0,0), t.col(3));

    /* correspondences is 3D global point - (left 2d, right 2d) feature corespondences;
       inliers -- 3D global - left(or right) 2d feature corespondences */

    vector<int> is_inlier(amount_correspondences, -1);
    for (size_t inl_idx : ransac.inliers_)
        is_inlier[inl_idx % amount_correspondences] = inl_idx % amount_correspondences;

    for (size_t idx : is_inlier) {
        if (idx == -1)
            continue;

        RegularFrame::point_reference &correspondence = correspondences[idx];
        shared_ptr<RegularFrame::PointCommon> p = correspondence.frame->additionals[correspondence.index].common;

        if (p->in_scene())
            continue;
        if (p->disparity < base * 40)
            continue;

        scene.push_back(p);
        p->add_to_scene();
    }

    current_frame.set_motion(m);
}
