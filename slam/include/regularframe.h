#pragma once

#include <vector>
#include <boost/circular_buffer.hpp>
#include <eigen3/Eigen/Core>
#include <memory>

#include "opengv/types.hpp"

#include "sophus/se3.hpp"

using std::vector;
using boost::circular_buffer;
using std::shared_ptr;
using Eigen::Vector3d;
using Eigen::Vector2d;

using Sophus::SE3d;

class RegularFrame {
public:
    struct Snapshot;
    struct PointCommon;
    struct point_reference;
    struct feature_additional;

    vector<Vector3d> bearingVectors_left;
    vector<Vector3d> bearingVectors_right;

    vector<Vector2d> image_points_left;
    vector<Vector2d> image_points_right;

    vector<feature_additional> additionals;

    SE3d motion;
    bool is_motion_associated;

    RegularFrame() {
        is_motion_associated = false;
    }

    void set_motion(SE3d motion) {
        this->motion = motion;
        is_motion_associated = true;
    }

//private:

    /* complete stable point from previous */
    void push_back(int i1, int i2,
                   Vector2d point_left,
                   Vector2d point_right,
                   const point_reference &previous);
    /* initializate stable point */
    void push_back(int i1, int i2,
                   Vector2d bearing_vector_left,
                   Vector2d bearing_vector_right);

    bool isBearingVectorsAvalible() {
        return bearingVectors_left.size() == additionals.size();
    }

    void release_image_points() {
        image_points_left.clear();
        image_points_right.clear();
    }

    int amount_points() {
        return additionals.size();
    }

    /* regular frame firstly holds only points' coordinates on image plane (measured in pixels)
     * also it can keep vearing vectors for each points
     * this method computes bearing vectors from points that already found on frame
     */
    void compute_bearing_vectors(double focal, double cu, double cv);

//    Snapshot snapshot;


    struct feature_additional {
        /* libviso's indeces' */
        int index_left;
        int index_right;

        int age;
        int reliability;

        int index; /* index of this point in regular frame's vector: its needed to sort features (ie for bucketing) */

        double depth;
        double disparity;

        shared_ptr<PointCommon> common; /* common information about feature for each point */
    };

    // TODO: create const Match::getFeatures (); in libviso to get snapshot of current frame
    struct Snapshot {
    };

    struct point_reference {
        /* Now i try to dont let any entity to get access to feature(2d) directly, only via frame observed it */
        RegularFrame *frame;
        int index;

        point_reference(RegularFrame *frame, int index) {
            this->frame = frame;
            this->index = index;
        }

        point_reference() {
            frame = nullptr;
            index = -1;
        }

        RegularFrame::feature_additional &getAdditional() const { return frame->additionals[index]; }

        opengv::bearingVector_t get_bearing_vector_left() { return frame->bearingVectors_left[index]; }
        opengv::bearingVector_t get_bearing_vector_right() { return frame->bearingVectors_right[index]; }

        opengv::point_t get_point_3d() { return frame->additionals[index].common->landmark; }


        point_reference &operator =(const point_reference &another) {
            this->frame = another.frame;
            this->index = another.index;

            return *this;
        }
    };


    /* Information about 2d feature that is common for it at all frames
     * i e 3D landmark that it corresponded,
     * RingBuffer(or another sructure) for quick search through frames it observed,
     * Stability of the point
     * Cto to eshe
    */
    struct PointCommon
    {
        Vector3d landmark; /* it's strange to make ScenePoint *landmark */
        circular_buffer<point_reference> buffer;
        bool already_triangulated;
        double disparity; /* disparity of the point when it was triangulated */ /* the best triangulation achives when disparity is max (in most cases) */

        double get_disparity() { return disparity; }

        bool is_in_scene;
        bool in_scene() { return is_in_scene; }
        void add_to_scene() { is_in_scene = true; }

        int additional_fields;
    //    int flags;
    //    const int flag_triangulated = 1;

        PointCommon() {
            already_triangulated = false;
            is_in_scene = false;
            disparity = 0;

            buffer.set_capacity(64);
        }

        PointCommon(point_reference point) : PointCommon() {
            buffer.push_back(point);
        }

        void push_back(point_reference point) { buffer.push_back(point); }
        void set_point(Vector3d point, double disparity) {
            this->landmark = point;
            already_triangulated = true;
            this->disparity = disparity;
        }
    };
};
