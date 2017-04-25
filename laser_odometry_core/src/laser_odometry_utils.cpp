#include <laser_odometry_core/laser_odometry_utils.h>

namespace laser_odometry
{
namespace utils
{

void tfFromXYTheta(const double x, const double y, const double theta, tf::Transform& t)
{
  t = tf::Transform(tf::createQuaternionFromYaw(theta),
                    {x, y, 0});
}

bool getTf(const std::string& source_frame,
           const std::string& target_frame,
           tf::StampedTransform& tf,
           const ros::Time& t,
           const ros::Duration& d)
{
  tf::TransformListener tf_listener;

  try
  {
    tf_listener.waitForTransform(
      target_frame, source_frame, t, d);
    tf_listener.lookupTransform (
      target_frame, source_frame, t, tf);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("Could not get transform from %s to %s : %s",
             source_frame.c_str(), target_frame.c_str(), ex.what());
    tf.setIdentity();
    return false;
  }

  return true;
}

bool getTf(const std::string& source_frame,
           const std::string& target_frame,
           tf::Transform& tf,
           const ros::Time& t,
           const ros::Duration& d)
{
  tf::StampedTransform stamped_tf;

  bool ok = getTf(source_frame, target_frame, stamped_tf, t, d);

  tf = stamped_tf;

  return ok;
}

bool getTf(const tf::tfMessagePtr tf_msg,
           const std::string& source_frame,
           const std::string& target_frame,
           tf::Transform& tf)
{
  for (const geometry_msgs::TransformStamped& tft : tf_msg->transforms)
  {
    if (tft.header.frame_id.compare(source_frame) == 0)
      if (tft.child_frame_id.compare(target_frame) == 0)
      {
        tf::transformMsgToTF(tft.transform, tf);
        return true;
      }
  }

  return false;
}

void print(const tf::Transform& tf, const std::string& h)
{
  std::cout << h
            << tf.getOrigin().getX()
            << " " << tf.getOrigin().getX()
            << " " << tf::getYaw(tf.getRotation()) << std::endl;
}

} /* namespace utils */
} /* namespace laser_odometry */
