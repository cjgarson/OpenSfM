// geometry/src/absolute_pose.cc
#include <geometry/absolute_pose.h>
#include <foundation/types.h>

#include <vector>
#include <utility>
#include <algorithm>

Eigen::Matrix3d RotationMatrixAroundAxis(const double cos_theta,
                                         const double sin_theta,
                                         const Eigen::Vector3d &v) {
  Eigen::Matrix3d R;
  const auto one_minus_cos_theta = 1.0 - cos_theta;
  R(0, 0) = cos_theta + v[0] * v[0] * one_minus_cos_theta;
  R(1, 0) = -v[2] * sin_theta + v[0] * v[1] * one_minus_cos_theta;
  R(2, 0) = v[1] * sin_theta + v[0] * v[2] * one_minus_cos_theta;
  R(0, 1) = v[2] * sin_theta + v[0] * v[1] * one_minus_cos_theta;
  R(1, 1) = cos_theta + v[1] * v[1] * one_minus_cos_theta;
  R(2, 1) = -v[0] * sin_theta + v[1] * v[2] * one_minus_cos_theta;
  R(0, 2) = -v[1] * sin_theta + v[0] * v[2] * one_minus_cos_theta;
  R(1, 2) = v[0] * sin_theta + v[1] * v[2] * one_minus_cos_theta;
  R(2, 2) = cos_theta + v[2] * v[2] * one_minus_cos_theta;
  return R;
}

namespace geometry {

AlignedVector<Mat34d> AbsolutePoseThreePoints(
    const Eigen::Matrix<double, -1, 3> &bearings,
    const Eigen::Matrix<double, -1, 3> &points) {
  // Pack inputs as (bearing, point) pairs for the templated solver.
  const auto n = std::min<Eigen::Index>(bearings.rows(), points.rows());
  AlignedVector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> samples;
  samples.reserve(static_cast<size_t>(n));
  for (Eigen::Index i = 0; i < n; ++i) {
    samples.emplace_back(bearings.row(i).normalized(), points.row(i));
  }

  // Templated implementation returns AlignedVector<Mat34d>
  return ::AbsolutePoseThreePoints(samples.begin(), samples.end());
}

Mat34d AbsolutePoseNPoints(
    const Eigen::Matrix<double, -1, 3> &bearings,
    const Eigen::Matrix<double, -1, 3> &points) {
  const auto n = std::min<Eigen::Index>(bearings.rows(), points.rows());
  AlignedVector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> samples;
  samples.reserve(static_cast<size_t>(n));
  for (Eigen::Index i = 0; i < n; ++i) {
    samples.emplace_back(bearings.row(i).normalized(), points.row(i));
  }

  return ::AbsolutePoseNPoints(samples.begin(), samples.end());
}

Eigen::Vector3d AbsolutePoseNPointsKnownRotation(
    const Eigen::Matrix<double, -1, 3> &bearings,
    const Eigen::Matrix<double, -1, 3> &points) {
  const auto n = std::min<Eigen::Index>(bearings.rows(), points.rows());
  AlignedVector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> samples;
  samples.reserve(static_cast<size_t>(n));
  for (Eigen::Index i = 0; i < n; ++i) {
    samples.emplace_back(bearings.row(i).normalized(), points.row(i));
  }

  return ::AbsolutePoseNPointsKnownRotation(samples.begin(), samples.end());
}

}  // namespace geometry
