#ifndef _LASER_ODOMETRY_CORE_LASER_ODOMETRY_TRANSFORM_H_
#define _LASER_ODOMETRY_CORE_LASER_ODOMETRY_TRANSFORM_H_

// Eigen header
#include <Eigen/Geometry>

namespace laser_odometry
{

using Scalar = double;

constexpr Scalar eps = 1e-6;

constexpr Scalar default_cov_diag_val = 1e-6;

template <typename T, int Dim>
using Isometry   = Eigen::Transform<T, Dim, Eigen::Isometry>;

template <typename T>
using Isometry3  = Isometry<T, 3>;

using Isometry3s = Isometry3<Scalar>;

// The transform type.
using Transform  = Isometry3s;

// The covariance type associated
// to the above Transform.
using Covariance = Eigen::Matrix<Scalar, 6, 6>;

struct TransformWithCovariance
{
  TransformWithCovariance() = default;

  explicit TransformWithCovariance(const Transform& transform,
                                   const Covariance& covariance) :
    transform_(transform),
    covariance_(covariance)
  {
    //
  }

  Transform  transform_  = Transform::Identity();
  Covariance covariance_ = Covariance::Zero();
};

} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_CORE_LASER_ODOMETRY_TRANSFORM_H_ */
