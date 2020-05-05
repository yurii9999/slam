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
        enum selection_policy { SELECT_ALL, SELECT_CLOSE, SELECT_FAR};
        enum inliers_determination_policy { ANGEL_BETWEEN_RAYS, REPROJECTION_ERROR };
//        enum egomotion_estimation_policy { NONCENTRAL_3D_2D, CENTRAL_3D_2D };
        enum triangulation_policy { FROM_PREVIOUS_FRAME, WITH_BEST_DISPARITY };

        selection_policy selection_policy_ = SELECT_ALL;
        inliers_determination_policy inliers_determination_policy_ = REPROJECTION_ERROR;
        triangulation_policy triangulation_policy_ = FROM_PREVIOUS_FRAME;

        bool using_nonlinear_optimization = true;

        double ransac_threshold = 0.001;
        int ransac_max_iterations = 100;

        double far_coeff = 100;
        double close_coeff = 40;
        /* select only points with disparity < far_c * base && > close_c * base */
//         "far_coeff" cause point is far enough to consider it

        int bucketing_amount = 2;
        int bucketing_widht = 50;
        int bucketing_height = 50;

        double final_th = 1.0;
        configuration() {}
        configuration(
                inliers_determination_policy idp,
                triangulation_policy tp,
                bool using_nonlinear_optimization,
                double ransac_threshold,
                int ransac_max_iterations,
                double final_th
                ) {
            inliers_determination_policy_ = idp;
            triangulation_policy_ = tp;
            this->using_nonlinear_optimization = using_nonlinear_optimization;
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

    /* additional output */
    vector<RegularFrame::point_reference> selection;

    vector<double>disparities; /* disparities with which was points triangulated */ /* not used now cause it could be gotten from temp_map[i] Z'th coordinate */
    vector<Vector3d> temp_map;
    void triangulate_current_frame();

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

//    vector<double> disparities;

    Sophus::SE3d estimate_relative_motion(); /* central */

    void select_points();
    void bucketing();

public:
    Sophus::SE3d delta; /* from previos frame to current frame */

    vector<double> residual_l;
    vector<double> residual_r;
    vector<int> inliers;
    void determine_inliers(); /* Check all obervation in current_frame_ is it inlier by comparison angels between rays */
    void determine_inliers_reprojection_error(); /* by compute reprojection error */
};
