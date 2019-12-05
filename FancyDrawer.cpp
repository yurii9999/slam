//
// Created by yurii on 04/12/2019.
//

#include "FancyDrawer.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

Mat FancyDrawer::stick(Mat left, Mat right) {
    int rows = left.rows;
    int cols = left.cols;

    Mat result(rows * 2, cols, left.type());
    left.copyTo(result(Rect(0, 0, cols, rows)));
    right.copyTo(result(Rect(0, rows, cols, rows)));

    return result;
}

/* draw matches between two frames not ugly(imho) */
Mat FancyDrawer::drawFrame(Mat img_l, Mat img_r, RegularFrame frame)
{
    Mat r1, r2;
    img_l.copyTo(r1); // deep copy
    img_r.copyTo(r2);

    const int Radius = 5;
    for (int i = 0; i < frame.kpts_l.size(); i++) {
        KeyPoint a = frame.kpts_l[i];
        KeyPoint b = frame.kpts_r[i];
        int range = 255;
        int blue = (range / 1) * (i % 2) ;
        int green = (range / 2) * (i % 3);
        int red = (range / 4) * (i % 5);

        Scalar color(blue, green, red);
        circle(r1, a.pt, Radius, color);
        circle(r2, b.pt, Radius, color);
    }

    return FancyDrawer::stick(r1, r2);
}

/* draw only point recognized as stable */
Mat FancyDrawer::drawStable(Mat img_l, Mat img_r, RegularFrame frame)
{
    Mat r1, r2;
    img_l.copyTo(r1); // deep copy
    img_r.copyTo(r2);

    const int Radius = 5;
    for (int i = 0; i < frame.stable.size(); i++) {
        KeyPoint a = frame.kpts_l[frame.stable[i]];
        KeyPoint b = frame.kpts_r[frame.stable[i]];
        int range = 255;
        int blue = (range / 1) * (i % 2) ;
        int green = (range / 2) * (i % 3);
        int red = (range / 4) * (i % 5);

        Scalar color(blue, green, red);
        circle(r1, a.pt, Radius, color);
        circle(r2, b.pt, Radius, color);
    }

    return FancyDrawer::stick(r1, r2);
}
