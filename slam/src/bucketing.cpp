#include "bucketing.h"

#include <iostream>

using namespace std;

Bucketing::Bucketing(int amount_features_per_cell, int bucket_width, int bucket_height, int width, int height) {
    amount_ = amount_features_per_cell;
    bucket_w_ = bucket_width;
    bucket_h_ = bucket_height;

    width_ = width;
    height_ = height;

    amount_buckets_u = width_ / bucket_w_;
    amount_buckets_v = height_ / bucket_h_;

    buckets.resize(amount_buckets_u * amount_buckets_v);
}

vector<int> Bucketing::apply_bucketing(RegularFrame const frame) {
    for (auto bucket : buckets)
        bucket.clear();

    for (auto p : frame.additionals) {
        int bucket_u = (int) (frame.image_points_left[p.index][0] / bucket_w_);
        int bucket_v = (int) (frame.image_points_left[p.index][1] / bucket_h_);

        vector<int> &bucket = buckets[bucket_u + bucket_v * amount_buckets_u];
        if (bucket.size() < amount_)
            bucket.push_back(p.index);
    }


    selection.clear();
    for (auto bucket : buckets)
        for (auto idx : bucket)
            selection.push_back(idx);

    return selection;
}
