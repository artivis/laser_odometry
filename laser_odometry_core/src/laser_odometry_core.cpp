#include <laser_odometry_core/laser_odometry_core.h>

#include <laser_odometry_core/laser_odometry_utils.h>

namespace laser_odometry
{

bool LaserOdometryBase::configure(const property_bag::PropertyBag &parameters)
{
  parameters.getPropertyValue<std::string>("base_frame",
                                           base_frame_, "base_frame");

  parameters.getPropertyValue<std::string>("laser_frame",
                                           laser_frame_, "base_laser_frame");

  parameters.getPropertyValue<std::string>("world_frame",
                                           world_frame_, "world_frame");

  base_to_laser_.setIdentity();
  utils::getTf(laser_frame_, base_frame_, base_to_laser_);
  laser_to_base_ = base_to_laser_.inverse();

  world_to_base_.setIdentity();

  // If set here, derived classes may struggle
  //configured_ = true;

  return true;
}

bool LaserOdometryBase::getOrigin()
{
  world_origin_.setIdentity();

  return true;
}

bool LaserOdometryBase::process(const sensor_msgs::LaserScan& scan,
                                geometry_msgs::PoseWithCovarianceStampedPtr pose)
{
  current_time_ = scan.header.stamp;

  geometry_msgs::Pose2DPtr pose_2d_ptr = boost::make_shared<geometry_msgs::Pose2D>();

  if (pose == nullptr)
    pose = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();

  bool processed = process(scan, pose_2d_ptr);

  pose->pose.pose.position.x = pose_2d_ptr->x;
  pose->pose.pose.position.y = pose_2d_ptr->y;
  pose->pose.pose.position.z = 0;

  tf::Quaternion q; q.setRPY(0, 0, pose_2d_ptr->theta);
  tf::quaternionTFToMsg(q, pose->pose.pose.orientation);

  // @todo
  //pose->pose.covariance;

  //pose->header

  relative_tf_.setIdentity();

  return processed;
}

tf::Transform LaserOdometryBase::predict(const tf::Transform& /*tf*/)
{
  tf::Transform tmp_tf; tmp_tf.setIdentity();
  return tmp_tf;
}

void LaserOdometryBase::broadcastTf()
{
  tf::StampedTransform transform_msg(world_to_base_, current_time_,
                                     world_frame_, base_frame_);

  tf_broadcaster_.sendTransform(transform_msg);
}

} /* namespace laser_odometry */
