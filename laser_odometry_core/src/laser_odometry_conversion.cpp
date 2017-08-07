#include "laser_odometry_core/laser_odometry_conversion.h"

#include "laser_odometry_core/laser_odometry_transform.h"

#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>

namespace laser_odometry {
namespace conversion {

template <>
void toRos(const Transform& t,
           geometry_msgs::Pose& pose_msg)
{
  Eigen::Affine3d aff(t);
  tf::poseEigenToMsg(aff, pose_msg);
}

template <>
void toRos(const Covariance& covariance,
           geometry_msgs::PoseWithCovariance::_covariance_type& covariance_msg)
{
  covariance_msg.at(0)  = covariance(0,0);
  covariance_msg.at(1)  = covariance(0,1);
  covariance_msg.at(2)  = covariance(0,2);
  covariance_msg.at(3)  = covariance(0,3);
  covariance_msg.at(4)  = covariance(0,4);
  covariance_msg.at(5)  = covariance(0,5);

  covariance_msg.at(6)  = covariance(1,0);
  covariance_msg.at(7)  = covariance(1,1);
  covariance_msg.at(8)  = covariance(1,2);
  covariance_msg.at(9)  = covariance(1,3);
  covariance_msg.at(10) = covariance(1,4);
  covariance_msg.at(11) = covariance(1,5);

  covariance_msg.at(12) = covariance(2,0);
  covariance_msg.at(13) = covariance(2,1);
  covariance_msg.at(14) = covariance(2,2);
  covariance_msg.at(15) = covariance(2,3);
  covariance_msg.at(16) = covariance(2,4);
  covariance_msg.at(17) = covariance(2,5);

  covariance_msg.at(18) = covariance(3,0);
  covariance_msg.at(19) = covariance(3,1);
  covariance_msg.at(20) = covariance(3,2);
  covariance_msg.at(21) = covariance(3,3);
  covariance_msg.at(22) = covariance(3,4);
  covariance_msg.at(23) = covariance(3,5);

  covariance_msg.at(24) = covariance(4,0);
  covariance_msg.at(25) = covariance(4,1);
  covariance_msg.at(26) = covariance(4,2);
  covariance_msg.at(27) = covariance(4,3);
  covariance_msg.at(28) = covariance(4,4);
  covariance_msg.at(29) = covariance(4,5);

  covariance_msg.at(30) = covariance(5,0);
  covariance_msg.at(31) = covariance(5,1);
  covariance_msg.at(32) = covariance(5,2);
  covariance_msg.at(33) = covariance(5,3);
  covariance_msg.at(34) = covariance(5,4);
  covariance_msg.at(35) = covariance(5,5);
}

} /* namespace conversion */
} /* namespace laser_odometry */
