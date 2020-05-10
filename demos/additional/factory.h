#pragma once

#include "egomotionestimation.h"
#include "egomotion_parameters.h"

#include "segmentation.h"
#include "segmentation_parameters.h"

#include "bucketing.h"
#include "bucketing_parameters.h"

#include "tracker.h"

#include "sequence_parameters.h"


struct Factory {
    static Tracker get_with_params(io_parameters sp) {
        double f  = sp.focal;
        double cu = sp.cu;
        double cv = sp.cv;
        double base = sp.base;

        Matcher::parameters params_matcher;
        params_matcher.cu = cu;
        params_matcher.cv = cv;
        params_matcher.base = base;
        params_matcher.f = f;
        params_matcher.refinement = 2;
        params_matcher.half_resolution = 0;

        Tracker tracker(f, cu, cv, params_matcher); /* tracker does not need f, cu cv; only matcher need */

        return tracker;
    }

    static EgomotionEstimation get_with_params(egomotion_parameters ep, io_parameters sp) {
        double f  = sp.focal;
        double cu = sp.cu;
        double cv = sp.cv;
        double base = sp.base;

        EgomotionEstimation::configuration c(
                    ep.ransac_threshold,
                    ep.ransac_max_iterations,
                    ep.final_threshold
                    );

        EgomotionEstimation ego(f, cu, cv, base, c);

        return ego;
    }

    static Segmentation get_with_params(segmentation_parameters segp, io_parameters seqp) {
        double f  = seqp.focal;
        double cu = seqp.cu;
        double cv = seqp.cv;
        double base = seqp.base;

        Segmentation segmentation(seqp.focal, seqp.cu, seqp.cv, seqp.base, segp.threshold);

        return segmentation;
    }

    static Bucketing get_with_params(bucketing_parameters bp) {
        return Bucketing(bp.amount_per_cell, bp.bucket_width, bp.bucket_height);
    }
};
