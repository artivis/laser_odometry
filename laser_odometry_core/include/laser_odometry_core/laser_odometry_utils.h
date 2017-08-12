#ifndef _LASER_ODOMETRY_CORE_LASER_ODOMETRY_UTILS_H_
#define _LASER_ODOMETRY_CORE_LASER_ODOMETRY_UTILS_H_

#include "laser_odometry_core/laser_odometry_transform.h"

#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace laser_odometry
{
namespace utils
{

void tfFromXYTheta(const double x, const double y,
                   const double theta, tf::Transform& t);

void tfFromXYTheta(const double x, const double y,
                   const double theta, Transform& t);

bool getTf(const std::string& source_frame,
           const std::string& target_frame,
           tf::StampedTransform& tf,
           const ros::Time& t = ros::Time(0),
           const ros::Duration& d = ros::Duration(1.5));

bool getTf(const std::string& source_frame,
           const std::string& target_frame,
           tf::Transform& tf,
           const ros::Time& t = ros::Time(0),
           const ros::Duration& d = ros::Duration(1.5));

bool getTf(const tf::tfMessagePtr tf_msg,
           const std::string& source_frame,
           const std::string& target_frame,
           tf::Transform& tf);

bool isIdentity(const tf::Transform& tf, const double eps = 1e-8);

std::string format(const tf::Transform& tf);

void print(const tf::Transform& tf, const std::string& h = "");

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

  Eigen::Matrix<T, 6, 6> cov_3d = Eigen::Matrix<T, 6, 6>::Identity();

  cov_3d.block(2,2,0,0) = cov_2d.block(2,2,0,0);
  cov_3d.block(2,1,0,5) = cov_2d.block(2,1,0,2);
  cov_3d.block(1,2,5,0) = cov_2d.block(1,2,2,0);

  cov_3d(5,5) = cov_2d(2,2);

  return cov_3d;
}

} /* namespace utils */
} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_CORE_LASER_ODOMETRY_UTILS_H_ */
