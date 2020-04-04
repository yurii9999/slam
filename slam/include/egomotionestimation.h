#pragma once

#include "regularframe.h"
#include <vector>
#include <memory>

#include "opengv/types.hpp"
#include "opengv/sac/Ransac.hpp"
#include "opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp"

using std::vector;
using std::shared_ptr;

class EgomotionEstimation
{
public:
    vector<shared_ptr<RegularFrame::PointCommon>> scene; /* it will be external reference in the future */

    /* camera offset in opengv terms */
    opengv::translations_t camOffsets;
    opengv::rotations_t camRotations;

    /* opengv's ransac */
    opengv::sac::Ransac<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem> ransac;

    /* camera's intrinsics */
    double focal;
    double cu;
    double cv;
    double base;

    /* additional output */
    vector<RegularFrame::point_reference> correspondences;
    int amount_correspondences;
    int amount_inliers;

    EgomotionEstimation(double focal, double cu, double cv, double base) {
        this->focal = focal;
        this->cu = cu;
        this->cv = cv;
        this->base = base;

        ransac.threshold_ = (1.0 - cos(atan(sqrt(2.0)*0.5/focal)));
        ransac.max_iterations_ = 100;

        camOffsets.resize(2); camRotations.resize(2);
        camOffsets[0] = opengv::translation_t(0, 0, 0); camOffsets[1] = opengv::translation_t(0, base, 0);
        camRotations[0] = Eigen::Matrix3d::Identity(); camRotations[1] = Eigen::Matrix3d::Identity();
    }

    EgomotionEstimation(double focal, double cu, double cv, double base, double ransac_threshold, int ransac_max_iterations) : EgomotionEstimation(focal, cu, cv, base) {
        ransac.threshold_ = ransac_threshold;
        ransac.max_iterations_ = ransac_max_iterations;
    }

    void estimate_motion(RegularFrame &curr, RegularFrame &prev);
};
