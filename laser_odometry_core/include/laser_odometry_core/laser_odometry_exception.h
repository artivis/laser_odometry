#ifndef _LASER_ODOMETRY_CORE_LASER_ODOMETRY_EXCEPTION_H_
#define _LASER_ODOMETRY_CORE_LASER_ODOMETRY_EXCEPTION_H_

#include

namespace laser_odometry
{

struct LaserOdometryException : public std::exception
{
  LaserOdometryException(const std::string &message);

  const char* what() const throw();

  const std::string message_;
};

} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_CORE_LASER_ODOMETRY_EXCEPTION_H_ */
