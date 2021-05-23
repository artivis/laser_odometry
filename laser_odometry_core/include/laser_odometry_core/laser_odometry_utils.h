#ifndef _LASER_ODOMETRY_CORE_LASER_ODOMETRY_UTILS_H_
#define _LASER_ODOMETRY_CORE_LASER_ODOMETRY_UTILS_H_

#include "laser_odometry_core/laser_odometry_transform.h"

#include <vector>
#include <Eigen/Eigenvalues>

namespace laser_odometry
{
namespace utils
{

void tfFromXYTheta(const double x, const double y,
                   const double theta, Transform& t);

template <typename T>
bool all_positive(const std::vector<T> vec)
{
  for (const auto v : vec)
  {
    if (v < 0) return false;
  }
  return true;
}

template<typename T>
inline Eigen::Matrix<T, 3, 3> matrixRollPitchYaw(const T roll,
                                                 const T pitch,
                                                 const T yaw)
{
  const Eigen::AngleAxis<T> ax = Eigen::AngleAxis<T>(roll,  Eigen::Matrix<T, 3, 1>::UnitX());
  const Eigen::AngleAxis<T> ay = Eigen::AngleAxis<T>(pitch, Eigen::Matrix<T, 3, 1>::UnitY());
  const Eigen::AngleAxis<T> az = Eigen::AngleAxis<T>(yaw,   Eigen::Matrix<T, 3, 1>::UnitZ());

  return (az * ay * ax).toRotationMatrix().matrix();
}

template<typename T>
inline Eigen::Matrix<T, 3, 3> matrixYaw(const T yaw)
{
  return matrixRollPitchYaw<T>(0, 0, yaw);
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 6, 6>
covariance2dTo3d(const Eigen::MatrixBase<Derived>& cov_2d)
{
  static_assert(Eigen::MatrixBase<Derived>::RowsAtCompileTime == 3, "Input arg must be of size 3*3");
  static_assert(Eigen::MatrixBase<Derived>::ColsAtCompileTime == 3, "Input arg must be of size 3*3");

  using T = typename Derived::Scalar;

  Eigen::Matrix<T, 6, 6> cov_3d = Eigen::Matrix<T, 6, 6>::Zero();

  cov_3d.block(0,0,2,2) = cov_2d.block(0,0,2,2);
  cov_3d.block(0,5,2,1) = cov_2d.block(0,2,2,1);
  cov_3d.block(5,0,1,2) = cov_2d.block(2,0,1,2);

  cov_3d(5,5) = cov_2d(2,2);

  return cov_3d;
}

/// Some helper functions ///

template<typename T>
inline T anyabs(const T& v)
{
  return (v < T(0))? -v : v;
}

template <typename Derived>
inline typename Eigen::MatrixBase<Derived>::Scalar
getYaw(const Eigen::MatrixBase<Derived>& r)
{
  return std::atan2( r(1, 0), r(0, 0) );
}

template <typename T, int Dim>
inline bool isIdentity(const Isometry<T, Dim>& t,
                       const T epsilon = eps)
{
  return t.isApprox(Isometry<T, Dim>::Identity(), epsilon);
}

template <typename T, int Dim>
inline bool isOthogonal(const Isometry<T, Dim>& t,
                        const T epsilon = eps)
{
  // Calling t.rotation() normalizes the rotation already
  // so that T(1) - R.rotation().determinant() = 0
  // is always true.

  return (anyabs(T(1) - anyabs(t.linear().determinant())) > epsilon) ? false : true;
}

template <typename T, int Dim>
inline bool isRotationProper(const Isometry<T, Dim>& t,
                             const T epsilon = eps)
{
  // Calling t.rotation() normalizes the rotation already
  // so that T(1) - R.rotation().determinant() = 0
  // is always true.

  return (anyabs(T(1) - t.linear().determinant()) > epsilon) ? false : true;
}

template <typename T, int N>
inline bool isSymmetric(const Eigen::Matrix<T, N, N>& M,
                        const T epsilon = eps)
{
  return M.isApprox(M.transpose(), epsilon);
}

template <typename T, int N>
inline bool isPositiveSemiDefinite(const Eigen::Matrix<T, N, N>& M)
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
inline bool isCovariance(const Eigen::Matrix<T, N, N>& M)
{
  return isSymmetric(M) && isPositiveSemiDefinite(M);
}

template <typename T, int Dim>
inline void makeOrthogonal(Isometry<T, Dim>& t)
{
  const Isometry<T, Dim> tmp = t;

  t = tmp.rotation();
  t.translation() = tmp.translation();
}

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3>
skew(const Eigen::MatrixBase<Derived>& v)
{
  static_assert(v.ColsAtCompileTime == 1, "Function 'skew' expect a vector.");
  static_assert(v.RowsAtCompileTime == 3, "Function 'skew' expect a vector of size 3.");

  typedef typename Derived::Scalar T;

  return (Eigen::Matrix<T, 3, 3>() <<
          0.0,   -v(2), +v(1),
          +v(2), 0.0,   -v(0),
          -v(1), +v(0), 0.0   ).finished();
}

} /* namespace utils */
} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_CORE_LASER_ODOMETRY_UTILS_H_ */
