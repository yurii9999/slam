#include "loransac/loransac.h"

#include <iostream>

using namespace std;

loransac::loransac() {
    probability = 0.999;
    th = 1.0;
    max_iterations = 1000;
}

// something copied from opengv

bool loransac::computeModel() {
    iterations_ = 0;
    n_best_inliers_count = -INT_MAX;
    double k = 1.0;

    std::vector<int> selection;
    opengv::transformation_t model_coefficients;

    int n_inliers_count = 0;
    unsigned skipped_count = 0;

    const unsigned max_skip = max_iterations * 10;

    // Iterate
    while( iterations_ < k && skipped_count < max_skip ) {
        problem->getSamples(iterations_, selection);

        if(selection.empty()) {
            fprintf(stderr,
                    "[sm::RandomSampleConsensus::computeModel] No samples could be selected!\n");
            break;
        }

        // Search for inliers in the point cloud for the current plane model M
        if(!problem->computeModelCoefficients( selection, model_coefficients ))
        {
            //++iterations_;
            ++ skipped_count;
            continue;
        }

        // Select the inliers that are within threshold_ from the model
        //sac_model_->selectWithinDistance( model_coefficients, threshold_, inliers );
        //if(inliers.empty() && k > 1.0)
        //  continue;

        n_inliers_count = problem->countWithinDistance(model_coefficients, th );

        // Better match ?
        if(n_inliers_count > n_best_inliers_count)
        {
            n_best_inliers_count = n_inliers_count;

            // Save the current model/inlier/coefficients selection as being the best so far
            model_              = selection;
            model_coefficients_ = model_coefficients;

            vector<int> inliers;
            problem->selectWithinDistance(model_coefficients, th, inliers);

            /* optimization of new best solution */
            selection.clear();

            problem->get_samples_for_optimization(selection, inliers);

            if ( problem->optimize(selection, model_coefficients) ) {
                n_inliers_count = problem->countWithinDistance(model_coefficients, th );
                if ( n_inliers_count > n_best_inliers_count ) {
                    model_coefficients_ = model_coefficients;
                    n_best_inliers_count = n_inliers_count;
                }
            }

            /* update k: */
            // Compute the k parameter (k=log(z)/log(1-w^n))
            double w = static_cast<double> (n_best_inliers_count) /
                    static_cast<double> (problem->getIndices()->size());

            double p_no_outliers = 1.0 - pow(w, static_cast<double> (problem->getSampleSize()));
            p_no_outliers =
                    (std::max) (std::numeric_limits<double>::epsilon(), p_no_outliers);
            // Avoid division by -Inf
            p_no_outliers =
                    (std::min) (1.0 - std::numeric_limits<double>::epsilon(), p_no_outliers);

            k = abs(log(1.0 - probability) / log(p_no_outliers));
            /* there was situation Iterations = 1 k = -2.14748e+09, and now i return abs(..) */
        }

        ++iterations_;

        if(iterations_ > max_iterations) {
            break;
        }

    }

    if(model_.empty()) {
        inliers_.clear();
        return (false);
    }

    cout << "Iterations: ";
    cout << iterations_ << endl;
//    cout << "k = ";
//    cout << k << endl;
//    cout << "skipped: " << skipped_count << endl;
    // Get the set of inliers that correspond to the best model found so far
    problem->selectWithinDistance( model_coefficients_, th, inliers_ );
    cout << "Before: " << inliers_.size() << endl;
    problem->final_optimization(inliers_, model_coefficients_);
    problem->selectWithinDistance( model_coefficients_, th, inliers_ );
    cout << "After: " << inliers_.size() << endl;

    return (true);
}
