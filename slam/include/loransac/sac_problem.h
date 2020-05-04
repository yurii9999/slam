#pragma once

#include "opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp"
#include <vector>

using opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem;
using std::vector;

class sac_problem : public AbsolutePoseSacProblem {
public:
    sac_problem(adapter_t & adapter);

    /* use new optimize procedure instead of defined in opengv cause there is one useless argument (i want to optimize via upnp) */
    bool optimize(vector<int> &sample, opengv::transformation_t &new_model);

    void get_samples_for_optimization(vector<int> &sample, vector<int> &inliers);

    void final_optimization(vector<int> &inliers, opengv::transformation_t &final_model);
};
