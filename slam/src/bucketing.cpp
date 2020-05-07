#include "bucketing.h"

#include <iostream>
#include <algorithm>

using namespace std;

Bucketing::Bucketing(int amount_features_per_cell, int bucket_width, int bucket_height) {
    amount_ = amount_features_per_cell;
    bucket_w_ = bucket_width;
    bucket_h_ = bucket_height;
}

void Bucketing::apply_bucketing(RegularFrame const frame) {
    for (auto &bucket : buckets)
        bucket.clear();

    vector<int> ages(frame.additionals.size());

    // distribute all features for theirs buckets
    for (auto &p : frame.additionals) {
        ages[p.index] = p.age;
        int bucket_u = floor(frame.image_points_left[p.index][0] / bucket_w_);
        int bucket_v = floor(frame.image_points_left[p.index][1] / bucket_h_);

        buckets[bucket_u + bucket_v * amount_buckets_u].push_back(p.index);
    }

    // chose this->amoun_ best features per each bucket
    for (auto &bucket : buckets) {
        std::sort(bucket.begin(), bucket.end(), [&ages] (int i1, int i2) { return ages[i1] > ages[i2]; });

        if (bucket.size() > amount_)
            bucket.erase(bucket.begin() + amount_, bucket.end());

    }

    // write chosen features to vector
    selection.clear();
    for (auto bucket : buckets)
        for (int idx : bucket)
            selection.push_back(idx);

}

void Bucketing::set_frame_size(int height, int width) {
    width_ = width;
    height_ = height;

    amount_buckets_u = width_ / bucket_w_ + 1;
    amount_buckets_v = height_ / bucket_h_ + 1;

    buckets.resize(amount_buckets_u * amount_buckets_v);
}
