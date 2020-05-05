#include "loransac/sac_problem.h"

#include <algorithm>
#include <iostream>

#include "opengv/absolute_pose/methods.hpp"

using namespace std;

sac_problem::sac_problem(opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem::adapter_t &adapter) :
    AbsolutePoseSacProblem (adapter, AbsolutePoseSacProblem::KNEIP)
{ }


// something copied from opengv
bool sac_problem::optimize(vector<int> &sample, opengv::transformation_t &new_model) {
    opengv::transformations_t solutions = opengv::absolute_pose::upnp(_adapter, sample);

    if (solutions.size() == 1) {
        new_model = solutions[0];
        return true;
    }

    //now compute reprojection error of fourth point, in order to find the right one
    double minScore = 1000000.0;
    int minIndex = -1;
    for(size_t i = 0; i < solutions.size(); i++)
    {
      //compute inverse transformation
      model_t inverseSolution;
      inverseSolution.block<3,3>(0,0) = solutions[i].block<3,3>(0,0).transpose();
      inverseSolution.col(3) = -inverseSolution.block<3,3>(0,0)*solutions[i].col(3);

      //get the fourth point in homogeneous form
      Eigen::Matrix<double,4,1> p_hom;
      p_hom.block<3,1>(0,0) = _adapter.getPoint(sample[3]);
      p_hom[3] = 1.0;

      //compute the reprojection (this is working for both central and
      //non-central case)
      opengv::point_t bodyReprojection = inverseSolution * p_hom;
      opengv::point_t reprojection =
          _adapter.getCamRotation(sample[3]).transpose() *
          (bodyReprojection - _adapter.getCamOffset(sample[3]));
      reprojection = reprojection / reprojection.norm();

      //compute the score
      double score =
          1.0 - (reprojection.transpose() * _adapter.getBearingVector(sample[3]));

      //check for best solution
      if( score < minScore )
      {
        minScore = score;
        minIndex = i;
      }
    }

    if(minIndex == -1)
      return false;

    new_model = solutions[minIndex];
    return true;
}

void sac_problem::get_samples_for_optimization(vector<int> &sample, vector<int> &inliers) {
    int sample_size = std::min(static_cast<int>(inliers.size() / 2), 14);


    if (sample_size < 4 && sample_size > 2)
        sample_size = inliers.size();

    sample.resize(sample_size);

    int index_size = inliers.size();
    for( unsigned int i = 0; i < sample_size; ++i )
    {
        // The 1/(RAND_MAX+1.0) trick is when the random numbers are not uniformly
        // distributed and for small modulo elements, that does not matter
        // (and nowadays, random number generators are good)
        //std::swap (shuffled_indices_[i], shuffled_indices_[i + (rand () % (index_size - i))]);
        std::swap(
                    inliers[i],
                    inliers[i + (rnd() % (index_size - i))] );
    }

    std::copy(
                inliers.begin(),
                inliers.begin() + sample_size,
                sample.begin() );
}

void sac_problem::final_optimization(vector<int> &inliers, opengv::transformation_t &model) {
    // nonlinear optimization already implemented here
    optimizeModelCoefficients(inliers, model, model);
}
