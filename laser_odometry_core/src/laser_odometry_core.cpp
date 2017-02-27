#include <laser_odometry_core/laser_odometry_core.h>

#include <laser_odometry_core/laser_odometry_utils.h>

#include <boost/assign/list_of.hpp>

namespace laser_odometry
{

bool LaserOdometryBase::configure()
{
  /// @todo use rosparam_handler
  private_nh_.param("laser_frame", laser_frame_, std::string("base_laser_link"));
  private_nh_.param("base_frame",  base_frame_,  std::string("base_link"));
  private_nh_.param("world_frame", world_frame_, std::string("world"));
  private_nh_.param("laser_odom_frame", laser_odom_frame_, std::string("laser_odom"));

  // Init all tf to I
  base_to_laser_.setIdentity();
  laser_to_base_.setIdentity();
  relative_tf_.setIdentity();
  world_origin_.setIdentity();
  world_to_base_.setIdentity();

  // Default covariance diag :
  std::vector<double> default_covariance;
  private_nh_.param("covariance_diag", default_covariance, default_covariance);

  if (default_covariance.size() == 6)
    default_covariance_ = default_covariance;
  else
  {
    ROS_WARN_STREAM("Retrieved " << default_covariance_.size()
                    << " covariance coeff. Should be 6. Setting default.");

    default_covariance_.resize(6);
    std::fill_n(default_covariance_.begin(), 6, 1e-9);
  }

  utils::getTf(laser_frame_, base_frame_, base_to_laser_);
  laser_to_base_ = base_to_laser_.inverse();

  // Configure derived class
  configured_ = configureImpl();

  return configured_;
}

tf::Transform& LaserOdometryBase::getOrigin()
{
  return world_origin_;
}

const tf::Transform& LaserOdometryBase::getOrigin() const
{
  return world_origin_;
}

void LaserOdometryBase::setOrigin(const tf::Transform& origin)
{
  world_origin_ = origin;
}

tf::Transform& LaserOdometryBase::getInitialGuess()
{
  return guess_relative_tf_;
}

const tf::Transform& LaserOdometryBase::getInitialGuess() const
{
  return guess_relative_tf_;
}

void LaserOdometryBase::setInitialGuess(const tf::Transform& guess)
{
  guess_relative_tf_ = guess;
}

bool LaserOdometryBase::process(const sensor_msgs::LaserScanPtr scan_ptr,
                                geometry_msgs::PosePtr pose_ptr,
                                geometry_msgs::PosePtr /*relative_pose_ptr*/)
{
  //if (scan_ptr == nullptr || pose_ptr == nullptr) return false;
  assert(scan_ptr != nullptr);
  assert(pose_ptr != nullptr);

  geometry_msgs::Pose2DPtr pose_2d_ptr = boost::make_shared<geometry_msgs::Pose2D>();

  bool processed = process(scan_ptr, pose_2d_ptr);

  pose_ptr->position.x = pose_2d_ptr->x;
  pose_ptr->position.y = pose_2d_ptr->y;
  pose_ptr->position.z = 0;
  tf::quaternionTFToMsg(tf::createQuaternionFromYaw(pose_2d_ptr->theta),
                        pose_ptr->orientation);
  return processed;
}

bool LaserOdometryBase::process(const sensor_msgs::LaserScanPtr scan_ptr,
                                geometry_msgs::PoseWithCovariancePtr pose_ptr,
                                geometry_msgs::PoseWithCovariancePtr /*relative_pose_ptr*/)
{
  //if (scan_ptr == nullptr || pose_ptr == nullptr) return false;
  assert(scan_ptr != nullptr);
  assert(pose_ptr != nullptr);

  geometry_msgs::Pose2DPtr pose_2d_ptr = boost::make_shared<geometry_msgs::Pose2D>();

  bool processed = process(scan_ptr, pose_2d_ptr);

  pose_ptr->pose.position.x = pose_2d_ptr->x;
  pose_ptr->pose.position.y = pose_2d_ptr->y;
  pose_ptr->pose.position.z = 0;
  tf::quaternionTFToMsg(tf::createQuaternionFromYaw(pose_2d_ptr->theta),
                        pose_ptr->pose.orientation);

  fillCovariance(pose_ptr->covariance);

  return processed;
}

bool LaserOdometryBase::process(const sensor_msgs::LaserScanPtr scan_ptr,
                                geometry_msgs::PoseWithCovarianceStampedPtr pose_ptr,
                                geometry_msgs::PoseWithCovarianceStampedPtr /*relative_pose_ptr*/)
{
  //if (scan_ptr == nullptr || pose_ptr == nullptr) return false;
  assert(scan_ptr != nullptr);
  assert(pose_ptr != nullptr);

  current_time_ = scan_ptr->header.stamp;

  geometry_msgs::PoseWithCovariancePtr tmp_pose_ptr =
      boost::make_shared<geometry_msgs::PoseWithCovariance>();

  bool processed = process(scan_ptr, tmp_pose_ptr);

  pose_ptr->pose.pose = tmp_pose_ptr->pose;
  pose_ptr->pose.covariance = tmp_pose_ptr->covariance;

  pose_ptr->header.frame_id = world_frame_;
  pose_ptr->header.stamp = scan_ptr->header.stamp;

  return processed;
}

bool LaserOdometryBase::configured() const noexcept
{
  return configured_;
}

tf::Transform LaserOdometryBase::predict(const tf::Transform& /*tf*/)
{
  return tf::Transform::getIdentity();
}

void LaserOdometryBase::fillCovariance(Covariance& covariance)
{
  covariance = boost::assign::list_of
               (static_cast<double>(default_covariance_[0]))  (0)  (0)  (0)  (0) (0)
               (0)  (static_cast<double>(default_covariance_[1]))  (0)  (0)  (0) (0)
               (0)  (0)  (static_cast<double>(default_covariance_[2]))  (0)  (0) (0)
               (0)  (0)  (0)  (static_cast<double>(default_covariance_[3]))  (0) (0)
               (0)  (0)  (0)  (0)  (static_cast<double>(default_covariance_[4])) (0)
               (0)  (0)  (0)  (0)  (0)  (static_cast<double>(default_covariance_[5]));
}

} /* namespace laser_odometry */
