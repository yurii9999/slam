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

        if (disparity > p.common->disparity) {
            Vector3d X(
                    (p_l[0] - cu) * base / disparity,
                    (p_l[1] - cv) * base / disparity,
                    focal * base / disparity );

            p.common->set_point(previous_frame.motion * X, disparity);
        }
    }

    /* now select all point that was stable triangulated (Triangulated with disparity less than 40 * baseline) */
    selection.clear();
    for (auto v : current_frame.additionals)
        if (v.common->disparity > 40 * base)
            selection.push_back(RegularFrame::point_reference(&current_frame, v.index));

    /* motion estimation */
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

    SE3d m(t.block<3,3>(0,0), t.col(3));
    current_frame.set_motion(m);

    delete adapter;
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

        points[i] = v.get_point_3d();
        points[selection.size() + i] = v.get_point_3d();
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

void EgomotionEstimation::select_points()
{

}
