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

} /* namespace utils */
} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_CORE_LASER_ODOMETRY_UTILS_H_ */
