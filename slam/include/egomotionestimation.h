#pragma once

#include "regularframe.h"
#include <vector>
#include <memory>

#include "opengv/types.hpp"
#include "opengv/sac/Ransac.hpp"
#include "opengv/sac/Lmeds.hpp"

#include "opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp"
#include "opengv/absolute_pose/NoncentralAbsoluteAdapter.hpp"
#include "opengv/absolute_pose/CentralAbsoluteAdapter.hpp"

#include "opengv/relative_pose/CentralRelativeAdapter.hpp"
#include "sophus/se3.hpp"

#include "loransac/loransac.h"

using std::vector;
using std::shared_ptr;

// Usage of pointers:
// 1) use std::shared_ptr, for classes that store, or can make desigion about removeng
// 2) use std::shared_ptr for classes that create entities for classes in 1)
// 3) use * for classes that temporary handle entities for any compytations and dont remove it, dont overwrite it

class EgomotionEstimation
{
public:
    struct configuration {
        double ransac_threshold = 0.001;
        int ransac_max_iterations = 100;

        double final_th = 1.0;
        configuration() {}
        configuration(
                double ransac_threshold,
                int ransac_max_iterations,
                double final_th
                ) {
            this->ransac_threshold = ransac_threshold;
            this->ransac_max_iterations = ransac_max_iterations;
            this->final_th = final_th;
        }

    };

    vector<shared_ptr<RegularFrame::PointCommon>> scene; /* it will be external reference in the future */

    loransac ransac;

    /* camera's intrinsics */
    double focal;
    double cu;
    double cv;
    double base;

    /* parameters */
    configuration conf;

    opengv::points_t temp_map;
    void triangulate_current_frame();

    opengv::bearingVectors_t temp_bearing;
    void compute_bearing_vectors();

    EgomotionEstimation(double focal, double cu, double cv, double base) {
        this->focal = focal;
        this->cu = cu;
        this->cv = cv;
        this->base = base;

        ransac.th = (1.0 - cos(atan(sqrt(2.0)*0.5/focal)));
        ransac.max_iterations = 100;
    }

    EgomotionEstimation(double focal, double cu, double cv, double base, configuration conf): EgomotionEstimation(focal, cu, cv, base) {
        this->conf = conf;
        ransac.th = conf.ransac_threshold;
        ransac.max_iterations = conf.ransac_max_iterations;
    }

    Sophus::SE3d estimate_motion(RegularFrame &curr, RegularFrame &prev, vector<int> indeces);

    Sophus::SE3d estimate_motion(RegularFrame &curr, RegularFrame &prev);

private:
    vector<int> active_indeces;
    RegularFrame *current_frame_;
    RegularFrame *previous_frame_;

    Sophus::SE3d estimate_relative_motion();

    void select_points();

    /* get from scene 3d point with index idx on current frame */
    /* now Triangulate point from previous frame with index idx on current frame */
    Vector3d get_scene_point(int idx);

public:
    Sophus::SE3d delta; /* from previos frame to current frame */

    vector<double> residual_l;
    vector<double> residual_r;
    vector<int> inliers;
    void determine_inliers(); /* Check all obervation in current_frame_ is it inlier by comparison angels between rays */
    void determine_inliers_reprojection_error(); /* by compute reprojection error */
};
