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
inline bool isIdentity(const Isometry<T, Dim>& t,
                        const T epsilon = 1e-5)
{
  return t.isApprox(Isometry<T, Dim>::Identity(), epsilon);
}

template <typename T, int Dim>
inline bool isOthogonal(const Isometry<T, Dim>& t,
                        const T epsilon = 1e-5)
{
  // Calling t.rotation() normalizes the rotation already
  // so that T(1) - R.rotation().determinant() = 0
  // is always true.

  const Eigen::Matrix<T, Dim-1, Dim-1> R(t.matrix().topLeftCorner(Dim-1, Dim-1));

  return (anyabs(T(1) - anyabs(R.determinant())) > epsilon) ? false : true;

//  return (anyabs(T(1) - t.matrix().topLeftCorner(Dim-1, Dim-1).determinant()) > epsilon) ? false : true;
}

template <typename T, int Dim>
inline bool isRotationProper(const Isometry<T, Dim>& t,
                             const T epsilon = 1e-6)
{
  // Calling t.rotation() normalizes the rotation already
  // so that T(1) - R.rotation().determinant() = 0
  // is always true.

  return (anyabs(T(1) - t.linear().determinant()) > epsilon) ? false : true;
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

template <typename T, int Dim>
inline void makeOrthogonal(Isometry<T, Dim>& t)
{
  const Isometry<T, Dim> tmp = t;

  t = tmp.rotation();
  t.translation() = tmp.translation();
}

} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_CORE_LASER_ODOMETRY_TRANSFORM_H_ */
