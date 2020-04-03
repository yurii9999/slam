#include <iostream>
#include <string>
#include <vector>
#include <stdint.h>

#include <fstream>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "tracker.h"
#include "viso_stereo.h"

#include <boost/format.hpp>

#include "egomotionestimation.h"

using namespace std;
using namespace cv;

using namespace opengv;

/* normalization test:
 * now we hold bearing vectors for StablePoint instead of image-point,
 * it is test that points normalized correctly and we can get image-coordinates for stabe points
*/

/* something taken form libviso */
int main (int argc, char** argv) {
    if (argc<2) {
        cerr << "Usage: ./viso2 path/to/sequence/2010_03_09_drive_0019" << endl;
        return 1;
    }

    string dir = argv[1];

    // calibration parameters for sequence 2010_03_09_drive_0019
    double f  = 645.24; // focal length in pixels
    double cu = 635.96; // principal point (u-coordinate) in pixels
    double cv = 194.13; // principal point (v-coordinate) in pixels
    double base     = 0.5707; // baseline in meters

    Matcher::parameters params;
    params.cu = cu;
    params.cv = cv;
    params.base = base;
    params.f = f;

    VisualOdometryStereo::parameters par;
    par.calib.cu = cu;
    par.calib.cv = cv;
    par.calib.f = f;

    par.match.cu = cu;
    par.match.cv = cv;
    par.match.f = f;
    par.match.base = base;

    VisualOdometryStereo viso(par);

    Tracker tracker(f, cu, cv, params);

    EgomotionEstimation ego(f, cu, cv, base);

    /* ---------------------------------------------------------------------------------------------------------- */

    /* Scene structure now is vector of 3d point in world reference frame */
    vector<RegularFrame::PointCommon> scene;

    vector<shared_ptr<RegularFrame>> frames;

    cv::Mat img_l;
    cv::Mat img_r;
    for (int32_t i_frame =0; i_frame < 370; i_frame ++) {
       char base_name[256]; sprintf(base_name,"%06d.png",i_frame );
       string left_img_file_name  = dir + "/I1_" + base_name;
       string right_img_file_name = dir + "/I2_" + base_name;
       img_l = imread(left_img_file_name);
       img_r = imread(right_img_file_name);

       tracker.push_back(img_l, img_r);
       frames.push_back(tracker.current);

       RegularFrame &current_frame = *tracker.current;
       if (i_frame  == 0) {
           /* initialize first frame on origin */
           SE3d origin;
           current_frame.set_motion(origin);

           continue;
       }

       RegularFrame &previous_frame = *tracker.previous;

       ego.estimate_motion(current_frame, previous_frame);
       cout << "i = " << i_frame << "\t correspondences: " << ego.amount_correspondences << "\t Inliers: " << ego.amount_inliers << endl;


//       if (!previous_frame.is_motion_associated) {
//           cout << "Unexpected: no motion associated on the previous frame" << endl;
//           return -1;
//       }

//       /* compute bearing vectors */
//       previous_frame.compute_bearing_vectors(f, cu, cv);
//       current_frame.compute_bearing_vectors(f, cu, cv);


//       /* triangulation: */
//       /* stage 0: taken from viso_stereo libviso */

//       for (auto p : previous_frame.additionals) {
////           if (p.common->already_triangulated)
////               continue;

//           Vector2d &p_l = previous_frame.image_points_left[p.index];
//           Vector2d &p_r = previous_frame.image_points_right[p.index];

//           double disparity = max(p_l[0] - p_r[0], 0.0001);

//           /* now triangulate only close points */
//           if (disparity > 40 * base && !p.common->already_triangulated) { /* use 20 instead of 40 for no reason */
//               Vector3d X(
//                       (p_l[0] - cu) * base / disparity,
//                       (p_l[1] - cv) * base / disparity,
//                       f * base / disparity );

//               p.common->set_point(previous_frame.motion.inverse() * X);
//               scene.push_back(*p.common);
//           }
//       }


//       /* building opengv's adapter */
//       /* use only point that is observed on current frame and is triangulated (now: <=> is not far) */
//       int amount_correspondences = 0;
//       for (auto v : current_frame.additionals)
//           if (v.common->already_triangulated)
//               amount_correspondences++;

//       cout << "Amount correspondences:: " << amount_correspondences << endl;

//       int amount_adapterov = 2 * amount_correspondences;
//       opengv::bearingVectors_t bearing_vectors(amount_adapterov);
//       opengv::points_t points(amount_adapterov);

//       vector<int> camCorrespondence(amount_correspondences * 2);
//       for (int i = 0; i < amount_correspondences; i++) {
//           camCorrespondence[i] = 0;
//           camCorrespondence[amount_correspondences + i] = 1;
//       }

//       int i = 0;
//       for (auto v : current_frame.additionals) {
//           if (!v.common->already_triangulated)
//               continue;

//           bearing_vectors[i] = current_frame.bearingVectors_left[v.index];
//           bearing_vectors[amount_correspondences + i] = current_frame.bearingVectors_right[v.index];

//           points[i] = v.common->landmark;
//           points[amount_correspondences + i] = v.common->landmark;

//           i++;
//       }

//       opengv::absolute_pose::NoncentralAbsoluteAdapter adapter(
//                   bearing_vectors,
//                   camCorrespondence,
//                   points,
//                   camOffsets,
//                   camRotations );

////       transformations_t ts = absolute_pose::upnp(adapter);
////       transformation_t t = absolute_pose::gpnp(adapter);
////       absolute_pose::CentralAbsoluteAdapter adapter(
////                   bearing_vectors,
////                   points
////                   );

//       sac::Ransac<sac_problems::absolute_pose::AbsolutePoseSacProblem> ransac;
//       std::shared_ptr<
//           sac_problems::absolute_pose::AbsolutePoseSacProblem> absposeproblem_ptr(
//           new sac_problems::absolute_pose::AbsolutePoseSacProblem(
//           adapter,
//           sac_problems::absolute_pose::AbsolutePoseSacProblem::KNEIP));
//       ransac.sac_model_ = absposeproblem_ptr;
//       ransac.threshold_ = 0.00005;
//       ransac.max_iterations_ = 200;

//       ransac.computeModel();

//       cout << ransac.inliers_.size() << endl;

//       transformation_t t = ransac.model_coefficients_;

////       transformation_t t = absolute_pose::gpnp(adapter, ransac.inliers_);

//       SE3d m(t.block<3,3>(0,0), t.col(3));

//       /* use libviso's viso_stereo */
//       vector<Matcher::p_match> p_matches;
//       i = 0;
//       for (auto v : current_frame.additionals) {
//           if (!v.common->already_triangulated){
//               continue;
//           }
//           int index_cur = v.index;
//           int index_prev = v.common->buffer[v.common->buffer.size() - 2].index;
//           Matcher::p_match new_match;
////           Vector2d im_p_l_c = current_frame.image_points_left[index_cur];
////           Vector2d im_p_r_c = current_frame.image_points_right[index_cur];
////           Vector2d im_p_l_p = previous_frame.image_points_left[index_prev];
////           Vector2d im_p_r_p = previous_frame.image_points_right[index_prev];
////           new_match.u1c = im_p_l_c[0];
////           new_match.v1c = im_p_l_c[1];

////           new_match.u2c = im_p_r_c[0];
////           new_match.v2c = im_p_r_c[1];

////           new_match.u1p = im_p_l_p[0];
////           new_match.v1p = im_p_l_p[1];

////           new_match.u2p = im_p_r_p[0];
////           new_match.v2p = im_p_r_p[1];

//           Vector3d ber_v_left_cur = current_frame.bearingVectors_left[index_cur];
//           Vector3d ber_v_right_cur = current_frame.bearingVectors_right[index_cur];
//           Vector3d ber_v_left_prev= previous_frame.bearingVectors_left[index_prev];
//           Vector3d ber_v_right_prev = previous_frame.bearingVectors_right[index_prev];
//           new_match.u1c = ber_v_left_cur[0] / ber_v_left_cur[2] * f + cu;
//           new_match.v1c = ber_v_left_cur[1] / ber_v_left_cur[2] * f + cv;

//           new_match.u2c = ber_v_right_cur[0] / ber_v_right_cur[2] * f + cu;
//           new_match.v2c = ber_v_right_cur[1] / ber_v_right_cur[2] * f + cv;

//           new_match.u1p = ber_v_left_prev[0] / ber_v_left_prev[2] * f + cu;
//           new_match.v1p = ber_v_left_prev[1] / ber_v_left_prev[2] * f + cv;

//           new_match.u2p = ber_v_right_prev[0] / ber_v_right_prev[2] * f + cu;
//           new_match.v2p = ber_v_right_prev[1] / ber_v_right_prev[2] * f + cv;

//           p_matches.push_back(new_match);

//           i++;
//       }


//       vector<double> tr = viso.estimateMotion(p_matches);
//       Matrix mat = viso.transformationVectorToMatrix(tr);
//       Eigen::Matrix3d rot;
//       for (int i = 0;i < 3; i++) {
//           rot.row(i) << mat.val[i][0], mat.val[i][1], mat.val[i][2];
//       }

//       Eigen::Vector3d tran;
//       for (int i= 0; i < 3; i++) {
//           tran[i] = mat.val[i][3];
//       }

//       SE3d m(rot, tran);
//       current_frame.set_motion(m);
    }

    ofstream output;
    output.open("result_scene");

    for (int i = 0; i < ego.scene.size(); i++) {
        output << ego.scene[i]->landmark.transpose() << endl;
    }


    output.close();

    output.open("result_pose");
    for (int i = 0; i < frames.size(); i++) {
        output << frames[i]->motion.translation().transpose() << endl;
    }

    output.close();

//    cout << frames.size() << endl;

//    for (auto f : frames) {
//        cout << "Absolute position: " << endl;
//        cout << f->motion.matrix() << endl;
//    }

    return 0;
}
