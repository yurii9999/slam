#pragma once

#include <vector>
#include "regularframe.h"

using std::vector;

class Bucketing
{
public:
    Bucketing(int amount_features_per_cell, int bucket_width, int bucket_height, int width, int height);

    vector<int> apply_bucketing(RegularFrame const frame);

    int amount_ = 2;

    int bucket_w_ = 50;
    int bucket_h_ = 50;

    int width_ = 1;
    int height_ = 1;

    vector<int> selection;

private:
    int amount_buckets_u;
    int amount_buckets_v;
    vector<vector<int>> buckets;
};
