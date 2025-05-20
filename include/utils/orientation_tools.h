/*! @file orientation_tools.h
 *  @brief Utility functions for 3D rotations
 *
 *  This file contains rotation utilities.  We generally use "coordinate
 * transformations" as opposed to the displacement transformations that are
 * commonly found in graphics.  To describe the orientation of a body, we use a
 * rotation matrix which transforms from world to body coordinates. This is the
 * transpose of the matrix which would rotate the body itself into the correct
 * orientation.
 *
 *  This follows the convention of Roy Featherstone's excellent book, Rigid Body
 * Dynamics Algorithms and the spatial_v2 MATLAB library that comes with it.
 * Note that we don't use the spatial_v2 convention for quaternions!
 */
#ifndef ORIENTATION_TOOLS_H
#define ORIENTATION_TOOLS_H

#include <cmath>
#include <iostream>
#include <type_traits>

#include "cpp_types.h"

namespace ori {
    enum class CoordinateAxis {
        X, Y, Z
    };

    /*!
     * Square a number
     */
    template<typename T>
    T square(T a) {
        return a * a;
    }


    /*!
     * Compute rotation matrix for coordinate transformation. Note that
     * coordinate_rotation(CoordinateAxis:X, .1) * v will rotate v by -.1 radians -
     * this transforms into a frame rotated by .1 radians!.
     */
    template<typename T>
    Mat3<T> coordinate_rotation(CoordinateAxis axis, T theta) {
        static_assert(std::is_floating_point<T>::value,
                      "must use floating point value");
        T s = std::sin(theta);
        T c = std::cos(theta);

        Mat3<T> R;

        if (axis == CoordinateAxis::X) {
            R << 1, 0, 0, 0, c, s, 0, -s, c;
        } else if (axis == CoordinateAxis::Y) {
            R << c, 0, -s, 0, 1, 0, s, 0, c;
        } else if (axis == CoordinateAxis::Z) {
            R << c, s, 0, -s, c, 0, 0, 0, 1;
        }

        return R;
    }

    /*!
     * Go from rpy to rotation matrix.
     */
    template<typename T>
    Mat3<typename T::Scalar> rpy_to_rotMat(const Eigen::MatrixBase<T> &v) {
        static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3,
                      "must have 3x1 vector");
        Mat3<typename T::Scalar> m = coordinate_rotation(CoordinateAxis::X, v[0]) *
                                     coordinate_rotation(CoordinateAxis::Y, v[1]) *
                                     coordinate_rotation(CoordinateAxis::Z, v[2]);
        return m;
    }


    /*!
     * Convert a coordinate transformation matrix to an orientation quaternion.
     */
    template<typename T>
    Quat<typename T::Scalar> rotMat_to_quat(
        const Eigen::MatrixBase<T> &r1) {
        static_assert(T::ColsAtCompileTime == 3 && T::RowsAtCompileTime == 3,
                      "Must have 3x3 matrix");
        Quat<typename T::Scalar> q;
        Mat3<typename T::Scalar> r = r1.transpose();
        typename T::Scalar tr = r.trace();
        if (tr > 0.0) {
            typename T::Scalar S = sqrt(tr + 1.0) * 2.0;
            q(0) = 0.25 * S;
            q(1) = (r(2, 1) - r(1, 2)) / S;
            q(2) = (r(0, 2) - r(2, 0)) / S;
            q(3) = (r(1, 0) - r(0, 1)) / S;
        } else if ((r(0, 0) > r(1, 1)) && (r(0, 0) > r(2, 2))) {
            typename T::Scalar S = sqrt(1.0 + r(0, 0) - r(1, 1) - r(2, 2)) * 2.0;
            q(0) = (r(2, 1) - r(1, 2)) / S;
            q(1) = 0.25 * S;
            q(2) = (r(0, 1) + r(1, 0)) / S;
            q(3) = (r(0, 2) + r(2, 0)) / S;
        } else if (r(1, 1) > r(2, 2)) {
            typename T::Scalar S = sqrt(1.0 + r(1, 1) - r(0, 0) - r(2, 2)) * 2.0;
            q(0) = (r(0, 2) - r(2, 0)) / S;
            q(1) = (r(0, 1) + r(1, 0)) / S;
            q(2) = 0.25 * S;
            q(3) = (r(1, 2) + r(2, 1)) / S;
        } else {
            typename T::Scalar S = sqrt(1.0 + r(2, 2) - r(0, 0) - r(1, 1)) * 2.0;
            q(0) = (r(1, 0) - r(0, 1)) / S;
            q(1) = (r(0, 2) + r(2, 0)) / S;
            q(2) = (r(1, 2) + r(2, 1)) / S;
            q(3) = 0.25 * S;
        }
        return q;
    }

    /*!
     * Convert a quaternion to a rotation matrix.  This matrix represents a
     * coordinate transformation into the frame which has the orientation specified
     * by the quaternion
     */
    template<typename T>
    Mat3<typename T::Scalar> quat_to_rotMat(
        const Eigen::MatrixBase<T> &q) {
        static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4,
                      "Must have 4x1 quat");
        typename T::Scalar e0 = q(0);
        typename T::Scalar e1 = q(1);
        typename T::Scalar e2 = q(2);
        typename T::Scalar e3 = q(3);

        Mat3<typename T::Scalar> R;

        R << 1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e0 * e3),
                2 * (e1 * e3 + e0 * e2), 2 * (e1 * e2 + e0 * e3),
                1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e0 * e1),
                2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1),
                1 - 2 * (e1 * e1 + e2 * e2);
        //  std::cout<<"R"<<std::endl<<R<<std::endl;
        R.transposeInPlace();
        //std::cout<<"RT: "<<std::endl<<R<<std::endl;
        return R;
    }

    /*!
     * Convert a quaternion to RPY.  Uses ZYX order (yaw-pitch-roll), but returns
     * angles in (roll, pitch, yaw).
     */
    template<typename T>
    Vec3<typename T::Scalar> quat_to_rpy(const Eigen::MatrixBase<T> &q) {
        static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4,
                      "Must have 4x1 quat");
        Vec3<typename T::Scalar> rpy;
        Vec4<typename T::Scalar> quat;
        quat[0] = q[3]; //w
        quat[1] = q[0];
        quat[2] = q[1];
        quat[3] = q[2];
        typename T::Scalar as = std::min(-2. * (quat[1] * quat[3] - quat[0] * quat[2]), .99999);
        rpy(2) =
                std::atan2(2 * (quat[1] * quat[2] + quat[0] * quat[3]),
                           square(quat[0]) + square(quat[1]) - square(quat[2]) - square(quat[3]));
        rpy(1) = std::asin(as);
        rpy(0) =
                std::atan2(2 * (quat[2] * quat[3] + quat[0] * quat[1]),
                           square(quat[0]) - square(quat[1]) - square(quat[2]) + square(quat[3]));
        return rpy;
    }

    //    /*!
    // * Convert a quaternion to RPY.  Uses ZYX order (yaw-pitch-roll), but returns
    // * angles in (roll, pitch, yaw).
    // */
    //    template<typename T>
    //    Vec3<typename T::Scalar> quat_to_rpy(const Eigen::MatrixBase<T> &q) {
    //        static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4,
    //                      "Must have 4x1 quat");
    //        Vec3<typename T::Scalar> rpy;
    //        typename T::Scalar as = std::min(-2. * (q[1] * q[3] - q[0] * q[2]), .99999);
    //        rpy(2) =
    //                std::atan2(2 * (q[1] * q[2] + q[0] * q[3]),
    //                           square(q[0]) + square(q[1]) - square(q[2]) - square(q[3]));
    //        rpy(1) = std::asin(as);
    //        rpy(0) =
    //                std::atan2(2 * (q[2] * q[3] + q[0] * q[1]),
    //                           square(q[0]) - square(q[1]) - square(q[2]) + square(q[3]));
    //        return rpy;
    //    }
    //    template<typename T>
    //    Quat<typename T::Scalar> rpy_to_quat(const Eigen::MatrixBase<T> &rpy) {
    //        static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3,
    //                      "Must have 3x1 vec");
    //        Mat3<typename T::Scalar> R = rpy_to_rotMat(rpy);
    //        Quat<typename T::Scalar> q = rotMat_to_quat(R);
    //        return q;
    //    }

    template<typename T>
    Vec3<typename T::Scalar> rotMat_to_rpy(const Eigen::MatrixBase<T> &R) {
        static_assert(T::ColsAtCompileTime == 3 && T::RowsAtCompileTime == 3,
                      "Must have 3x3 matrix");
        Quat<typename T::Scalar> q = rotMat_to_quat(R);
        Vec3<typename T::Scalar> rpy = quat_to_rpy(q);
        return rpy;
    }


    /*!
     * Take the product of two quaternions
     */
    template<typename T>
    Quat<typename T::Scalar> quat_product(const Eigen::MatrixBase<T> &q1,
                                          const Eigen::MatrixBase<T> &q2) {
        typename T::Scalar r1 = q1[0];
        typename T::Scalar r2 = q2[0];
        Vec3<typename T::Scalar> v1(q1[1], q1[2], q1[3]);
        Vec3<typename T::Scalar> v2(q2[1], q2[2], q2[3]);

        typename T::Scalar r = r1 * r2 - v1.dot(v2);
        Vec3<typename T::Scalar> v = r1 * v2 + r2 * v1 + v1.cross(v2);
        Quat<typename T::Scalar> q(r, v[0], v[1], v[2]);
        return q;
    }


    /*!
  * Take the multiply of two quaternions
  */
    template<typename T>
    Quat<typename T::Scalar> quat_multiply(const Eigen::MatrixBase<T> &q1,
                                           const Eigen::MatrixBase<T> &q0) {
        Quat<typename T::Scalar> q, qout; //x,y,z,w
        auto x0 = q0[0], y0 = q0[1], z0 = q0[2], w0 = q0[3];
        auto x1 = q1[0], y1 = q1[1], z1 = q1[2], w1 = q1[3];
        q << x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0,
                -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0;
        return q;
    }

    /*!
 * quat conjugate
 */
    template<typename T>
    Quat<typename T::Scalar> quat_conjugate(const Eigen::MatrixBase<T> &q) {
        Quat<typename T::Scalar> quat_conjugate; //w,x,y,z
        quat_conjugate << q[0], -q[1], -q[2], -q[3];
        return quat_conjugate;
    }

    //    template<typename T>
    //    T get_base_flip(const Eigen::MatrixBase<T> &quat) {
    //        Quat<float> q_point, quat_inverse;//w,x,y,z
    //        q_point << 0, 0, 0, 1;
    //        quat_inverse << quat[0], -quat[1], -quat[2], -quat[3];
    //        auto value = quat_product((quat, q_point), quat_inverse)[3];
    //        return value;
    //    }
} // namespace ori


#endif
