#ifndef _LASER_ODOMETRY_CORE_LASER_ODOMETRY_UTILS_H_
#define _LASER_ODOMETRY_CORE_LASER_ODOMETRY_UTILS_H_

#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace laser_odometry
{
namespace utils
{

void tfFromXYTheta(const double x, const double y,
                   const double theta, tf::Transform& t);

bool getTf(const std::string& source_frame,
           const std::string& target_frame,
           tf::StampedTransform& tf);

bool getTf(const std::string& source_frame,
           const std::string& target_frame,
           tf::Transform& tf);

bool getTf(const tf::tfMessagePtr tf_msg,
           const std::string& source_frame,
           const std::string& target_frame,
           tf::Transform& tf);

void print(const tf::Transform& tf, const std::string& h = "");

} /* namespace utils */
} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_CORE_LASER_ODOMETRY_UTILS_H_ */
