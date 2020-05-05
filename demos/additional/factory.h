#pragma once

#include "egomotionestimation.h"
#include "segmentation.h"
#include "tracker.h"

#include "segmentation_parameters.h"
#include "sequence_parameters.h"
#include "egomotion_parameters.h"

struct Factory {
    static Tracker get_with_params(sequence_parameters sp) {
        double f  = sp.focal;
        double cu = sp.cu;
        double cv = sp.cv;
        double base = sp.base;

        Matcher::parameters params_matcher;
        params_matcher.cu = cu;
        params_matcher.cv = cv;
        params_matcher.base = base;
        params_matcher.f = f;

        Tracker tracker(f, cu, cv, params_matcher); /* tracker does not need f, cu cv; only matcher need */

        return tracker;
    }

    static EgomotionEstimation get_with_params(egomotion_parameters ep, sequence_parameters sp) {
        double f  = sp.focal;
        double cu = sp.cu;
        double cv = sp.cv;
        double base = sp.base;

        EgomotionEstimation::configuration c(
                    ep.inliers_determination_policy_,
                    ep.triangulation_policy_,
                    ep.using_nonlinear_optimization,
                    ep.ransac_threshold,
                    ep.ransac_max_iterations,
                    ep.final_threshold
                    );
        c.bucketing_height = ep.bucketing_size;
        c.bucketing_widht = ep.bucketing_size;
        c.bucketing_amount = ep.bucketing_amount;

        c.close_coeff = ep.close_coeff;
        c.far_coeff = ep.far_coeff;

        EgomotionEstimation ego(f, cu, cv, base, c);

        return ego;
    }

    static Segmentation get_with_params(segmentation_parameters segp, sequence_parameters seqp) {
        double f  = seqp.focal;
        double cu = seqp.cu;
        double cv = seqp.cv;
        double base = seqp.base;

        Segmentation segmentation(seqp.focal, seqp.cu, seqp.cv, seqp.base, segp.threshold);
        segmentation.second_th = segp.second_th;

        return segmentation;
    }
};
