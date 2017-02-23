#include <laser_odometry_core/laser_odometry_core.h>

#include <tf/tf.h>

namespace laser_odometry
{

LaserOdometryBase::process(const sensor_msgs::LaserScan& scan,
                           geometry_msgs::PoseWithCovarianceStampedPtr pose)
{
  geometry_msgs::Pose2DPtr pose_2d_ptr = boost::make_shared<geometry_msgs::Pose2D>();

  if (pose == nullptr)
    pose = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();

  bool processed = process(scan, pose_2d_ptr);

  pose->pose.pose.position.x = pose_2d_ptr->x;
  pose->pose.pose.position.y = pose_2d_ptr->y;
  pose->pose.pose.position.z = 0;

  tf::Quaternion q; q.setRPY(0, 0, pose_2d_ptr->theta);

  pose->pose.pose.orientation = q;

  // @todo
  //pose->pose.covariance;

  //pose->header

  return processed;
}

} /* namespace laser_odometry */
