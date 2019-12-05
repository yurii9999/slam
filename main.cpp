#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"

#include "Tracker.h"

#include "opencv2/imgproc.hpp"

#include "RegularFrame.h"
#include "FancyDrawer.h"

using namespace std;
using namespace cv;

/* Создать отдельный класс RegularFrame, туда вынести из трекера KpAdditional,
 * И перенести туда все рисовалки */
/* Добавить рисовалки только тех точек, возраст которых больше двух */
/* Добавить изменение дескрипторов в трекере и обновление дополнительной информации */
/* Добавить поддержку поля "индекс на пердыдущем кадре" (Видимо, тоже в трекере, или сделать какой то класс внутри трекера -- матчер) */

/* ещё я подумал, что возможно отбор стабильных точек -- это тоже не обязанности трекера, а обязанности отборщика точек */
/* то есть трекер только трекает точки, а стабильные или нестабильные ему вообщем то пофек, если нашлось на предыдущем кадре что то, то он продолжит то что нашел, если нет, то начнет новую */

int main(int argc, char** argv)
{
    string dir = argv[1];
    string output_dir = "out/";
    Ptr<Feature2D> detector = AKAZE::create();
    Ptr<DescriptorMatcher> matcher = BFMatcher::create(NORM_HAMMING);
    Tracker t(detector, matcher);

    for (int32_t i=0; i<10; i++) {
        // st is taken from libviso's demo.cpp
        // input file names
        char base_name[256]; sprintf(base_name,"%06d.png",i);
        string left_img_file_name  = dir + "/I1_" + base_name;
        string right_img_file_name = dir + "/I2_" + base_name;

        // catch image read/write errors here
        Mat img_l = imread(left_img_file_name);
        Mat img_r = imread(right_img_file_name);

        t.push_back(img_l, img_r);

        Mat out = FancyDrawer::drawStable(img_l, img_r, *t.prev);
        string outname = output_dir + (string) base_name;
        imwrite(outname, out);
    }

    cout << "Hello World!" << endl;
    return 0;
}
