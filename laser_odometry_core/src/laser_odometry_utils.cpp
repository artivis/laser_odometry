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

void tfFromXYTheta(const double x, const double y, const double theta, Transform& t)
{
  Eigen::AngleAxis<Scalar> rollAngle(0,    Eigen::Vector3d::UnitX());
  Eigen::AngleAxis<Scalar> pitchAngle(0,   Eigen::Vector3d::UnitY());
  Eigen::AngleAxis<Scalar> yawAngle(theta, Eigen::Vector3d::UnitZ());

  Eigen::Quaternion<Scalar> q = rollAngle * pitchAngle * yawAngle;

  t = q;
  t.translation() = Eigen::Matrix<Scalar, 3, 1>(x, y, 0);
}

bool getTf(const std::string& source_frame,
           const std::string& target_frame,
           tf::StampedTransform& tf,
           const ros::Time& t,
           const ros::Duration& d)
{
  tf::TransformListener tf_listener;
  tf::StampedTransform tf_tmp;

  std::string error;
  if (tf_listener.waitForTransform(target_frame, source_frame, t, d, ros::Duration(0.01), &error))
  {
    try
    {
      tf_listener.lookupTransform (
        target_frame, source_frame, t, tf_tmp);
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN("Could not get transform from %s to %s at %f after %f :\n %s",
               source_frame.c_str(), target_frame.c_str(),
               t.toSec(), d.toSec(), ex.what());

      return false;
    }
  }
  else
  {
    ROS_WARN("Could not find transform from %s to %s at %f after %f :\n %s",
             source_frame.c_str(), target_frame.c_str(),
             t.toSec(), d.toSec(), error.c_str());
    return false;
  }

  tf = tf_tmp;
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

  if (ok) tf = stamped_tf;

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

bool isIdentity(const tf::Transform& tf, const double eps)
{
  const tf::Vector3& o = tf.getOrigin();

  if (o.x() > eps) return false;
  if (o.y() > eps) return false;
  if (o.z() > eps) return false;

  if (tf.getRotation().angleShortestPath(
        tf::Quaternion::getIdentity()) > eps) return false;

  return true;
}

std::string format(const tf::Transform& tf)
{
  std::stringstream ss;

  ss << tf.getOrigin().getX()
     << " " << tf.getOrigin().getY()
     << " " << tf::getYaw(tf.getRotation());

  return ss.str();
}

void print(const tf::Transform& tf, const std::string& h)
{
  std::cout << h << format(tf) << std::endl;
}

template <>
inline bool all_positive<float>(const std::vector<float> vec);

template <>
inline bool all_positive<double>(const std::vector<double> vec);

} /* namespace utils */
} /* namespace laser_odometry */
