#pragma once

#include "loransac/sac_problem.h"
#include "opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp"

#include <vector>
#include <memory>

using std::shared_ptr;
using std::vector;
using opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem;

/* implementation of LO RANSAC
 * (2) Simple and (4) Inner RANSAC
 * Works with "sac_problem.h", where minimal solver is P3P KNEIP, and non-minimal solver is UPnP
 * Non-linear optimization perfoms in the end */

/* Task : test and compare usage in inner ransac procedure (upnp with sample size N of inliers) and (nonlinear optimization through sample size N of inliers) */

class loransac {
public:
    loransac();
    bool computeModel();

    int max_iterations;
    double probability;
    double th;

    opengv::transformation_t model_coefficients_;
    vector<int> model_;
    int iterations_;
    vector<int> inliers_;
    shared_ptr<sac_problem> problem;

private:
    int n_best_inliers_count;
};
