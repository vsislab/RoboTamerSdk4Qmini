/*! @file cTypes.h
 *  @brief Common types that are only valid in C++
 *
 *  This file contains types which are only used in C++ code.  This includes
 * Eigen types, template types, aliases, ...
 */

#ifndef PROJECT_CPPTYPES_H
#define PROJECT_CPPTYPES_H

#include <vector>
#include <eigen3/Eigen/Dense>


// Rotation Matrix
template<typename T>
using RotMat = typename Eigen::Matrix<T, 3, 3>;

// 2x1 Vector
template<typename T>
using Vec2 = typename Eigen::Matrix<T, 2, 1>;

// 3x1 Vector
template<typename T>
using Vec3 = typename Eigen::Matrix<T, 3, 1>;

// 4x1 Vector
template<typename T>
using Vec4 = typename Eigen::Matrix<T, 4, 1>;

// 6x1 Vector
template<typename T>
using Vec6 = typename Eigen::Matrix<T, 6, 1>;

// 8x1 Vector
template<typename T>
using Vec8 = typename Eigen::Matrix<T, 8, 1>;

// 12x1 Vector
template<typename T>
using Vec12 = Eigen::Matrix<T, 12, 1>;

// 4x1 Vector
template<typename T>
using Vec27 = typename Eigen::Matrix<T, 27, 1>;

// 20x1 Vector
template<typename T>
using Vec20 = Eigen::Matrix<T, 20, 1>;

// 20x1 Vector
template<typename T>
using Vec29 = Eigen::Matrix<T, 29, 1>;

// 20x1 Vector
template<typename T>
using Vec10 = Eigen::Matrix<T, 10, 1>;

// 32x1 Vector
template<typename T>
using Vec32 = Eigen::Matrix<T, 32, 1>;

// 3x3 Matrix
template<typename T>
using Mat3 = typename Eigen::Matrix<T, 3, 3>;

// 4x1 Vector
template<typename T>
using Quat = typename Eigen::Matrix<T, 4, 1>;

// 4x3 Matrix
template<typename T>
using Mat43 = typename Eigen::Matrix<T, 4, 3>;

// 4x4 Matrix
template<typename T>
using Mat44 = typename Eigen::Matrix<T, 4, 4>;
#endif  // PROJECT_CPPTYPES_H
