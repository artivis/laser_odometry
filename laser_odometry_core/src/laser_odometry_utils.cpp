#include <laser_odometry_core/laser_odometry_utils.h>

namespace laser_odometry
{
namespace utils
{

void tfFromXYTheta(const double x, const double y, const double theta, tf::Transform& t)
{
  t.setOrigin(tf::Vector3(x, y, 0.0));
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  t.setRotation(q);
}

bool getTf(const std::string& source_frame,
           const std::string& target_frame,
           tf::StampedTransform& tf)
{
  tf::TransformListener tf_listener;

  ros::Time t = ros::Time::now();
  try
  {
    tf_listener.waitForTransform(
      target_frame, source_frame, t, ros::Duration(1.0));
    tf_listener.lookupTransform (
      target_frame, source_frame, t, tf);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("Could not get initial transform from base to laser frame, %s", ex.what());
    return false;
  }

  return true;
}

bool getTf(const std::string& source_frame,
           const std::string& target_frame,
           tf::Transform& tf)
{
  tf::StampedTransform stamped_tf;

  bool ok = getTf(source_frame, target_frame, stamped_tf);

  if (ok) tf = stamped_tf;

  return ok;
}

} /* namespace utils */
} /* namespace laser_odometry */
