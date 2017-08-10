#ifndef _LASER_ODOMETRY_CORE_LASER_ODOMETRY_TRANSFORM_H_
#define _LASER_ODOMETRY_CORE_LASER_ODOMETRY_TRANSFORM_H_

// Eigen header
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

namespace laser_odometry
{

using Scalar = double;

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
  TransformWithCovariance(const Transform& transform,
                          const Covariance& covariance) :
    transform_(transform),
    covariance_(covariance)
  {
    //
  }

  Transform  transform_;
  Covariance covariance_;
};

/// Some helper functions ///

template<typename T>
inline T anyabs(const T& v)
{
  return (v < T(0))? -v : v;
}

template <typename Derived>
typename Eigen::MatrixBase<Derived>::Scalar
getYaw(const Eigen::MatrixBase<Derived>& r)
{
  return std::atan2( r(1, 0), r(0, 0) );
}

template <typename T, int Dim>
inline bool isOthogonal(const Isometry<T, Dim>& t,
                        const T epsilon = 1e-5)
{
  return (anyabs(T(1) - t.rotation().determinant()) > epsilon) ? false : true;
}

template <typename T, int N>
bool isSymmetric(const Eigen::Matrix<T, N, N>& M,
                 const T eps = 1e-5)
{
  return M.isApprox(M.transpose(), eps);
}

template <typename T, int N>
bool isPositiveSemiDefinite(const Eigen::Matrix<T, N, N>& M)
{
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T, N, N> > eigensolver(M);

  if (eigensolver.info() == Eigen::Success)
  {
    // All eigenvalues must be >= 0:
    return (eigensolver.eigenvalues().array() >= T(0)).all();
  }

  return false;
}

template <typename T, int N>
bool isCovariance(const Eigen::Matrix<T, N, N>& M)
{
  return isSymmetric(M) && isPositiveSemiDefinite(M);
}

} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_CORE_LASER_ODOMETRY_TRANSFORM_H_ */
