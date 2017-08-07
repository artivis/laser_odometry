#include <laser_odometry_core/laser_odometry_base.h>
#include <laser_odometry_core/laser_odometry_report.h>
#include <laser_odometry_core/laser_odometry_conversion.h>

#include <laser_odometry_core/laser_odometry_utils.h>

#include <boost/assign/list_of.hpp>

#define assert_not_null(x) \
  assert(x != nullptr);

namespace laser_odometry
{

// Some Specialization
template <>
inline void LaserOdometryBase::fillMsg<geometry_msgs::Pose2DPtr>(geometry_msgs::Pose2DPtr& msg_ptr)
{
  if (msg_ptr == nullptr) return;

  msg_ptr->x = fixed_origin_to_base_.translation()(0);
  msg_ptr->y = fixed_origin_to_base_.translation()(1);
  msg_ptr->theta = getYaw(fixed_origin_to_base_.rotation());
}

template <>
inline void LaserOdometryBase::fillMsg<nav_msgs::OdometryPtr>(nav_msgs::OdometryPtr& msg_ptr)
{
  if (msg_ptr == nullptr) return;

  msg_ptr->header.stamp    = current_time_;
  msg_ptr->header.frame_id = laser_odom_frame_;
  msg_ptr->child_frame_id  = base_frame_;

  conversion::toRos(fixed_origin_to_base_, msg_ptr->pose.pose);

  conversion::toRos(pose_covariance_, msg_ptr->pose.covariance);

  //msg_ptr->pose.covariance  = pose_covariance_;
  //msg_ptr->twist.covariance = pose_twist_covariance_;
}

template <>
inline void LaserOdometryBase::fillIncrementMsg<geometry_msgs::Pose2DPtr>(geometry_msgs::Pose2DPtr& msg_ptr)
{
  if (msg_ptr == nullptr) return;

  msg_ptr->x = increment_.translation()(0);
  msg_ptr->y = increment_.translation()(1);
  msg_ptr->theta = getYaw(increment_.rotation());
}

template <>
inline void LaserOdometryBase::fillIncrementMsg<nav_msgs::OdometryPtr>(nav_msgs::OdometryPtr& msg_ptr)
{
  if (msg_ptr == nullptr) return;

  msg_ptr->header.stamp    = current_time_;
  msg_ptr->header.frame_id = "last_key_frame"; /// @todo this frame does not exist. Should it?
  msg_ptr->child_frame_id  = base_frame_;

  conversion::toRos(increment_, msg_ptr->pose.pose);

  conversion::toRos(increment_covariance_, msg_ptr->pose.covariance);
  //msg_ptr->twist.covariance = increment_twist_covariance_;
}

// Class functions definition

bool LaserOdometryBase::configure()
{
  reset();

  private_nh_.param("laser_frame",      laser_frame_,      std::string("base_laser_link"));
  private_nh_.param("base_frame",       base_frame_,       std::string("base_link"));
  private_nh_.param("odom_frame",       fixed_frame_,      std::string("odom"));
  private_nh_.param("laser_odom_frame", laser_odom_frame_, std::string("odom"));

  // Default covariance diag :
  std::vector<Scalar> default_covariance(default_cov_diag_);
  private_nh_.param("covariance_diag", default_covariance, default_covariance);

  if (utils::all_positive(default_covariance))
  {
    if (default_covariance.size() == 3)
    {
      default_cov_diag_[0] = default_covariance[0];
      default_cov_diag_[1] = default_covariance[1];
      default_cov_diag_[5] = default_covariance[2];
    }
    else if (default_covariance.size() == 6)
    {
      default_cov_diag_.swap(default_covariance);
    }
    else
    {
      ROS_WARN_STREAM("Retrieved " << default_covariance.size()
                      << " covariance coeff.\n"
                      << "Should be 3 (xx, yy, tt)\n"
                      << " or 6 (xx, yy, zz, r_xx, r_yy, r_zz).\n"
                      << "Setting default: Identity*"
                      << default_cov_diag_val);
    }
  }
  else
  {
    ROS_ERROR_STREAM("Caught negative entry in default increment "
                     "covariance diagonal.\n Setting default: Identity*"
                     << default_cov_diag_val);
  }

  increment_covariance_(0,0) = default_cov_diag_[0];
  increment_covariance_(1,1) = default_cov_diag_[1];
  increment_covariance_(2,2) = default_cov_diag_[2];
  increment_covariance_(3,3) = default_cov_diag_[3];
  increment_covariance_(4,4) = default_cov_diag_[4];
  increment_covariance_(5,5) = default_cov_diag_[5];

  pose_covariance_ = increment_covariance_;

  // Configure derived class
  configured_ = configureImpl();

  return configured_;
}

LaserOdometryBase::ProcessReport
LaserOdometryBase::process(const sensor_msgs::LaserScanConstPtr& scan_msg,
                           geometry_msgs::Pose2DPtr pose_msg,
                           geometry_msgs::Pose2DPtr pose_increment_msg)
{
  assert_not_null(scan_msg);

  has_new_kf_   = false;
  current_time_ = scan_msg->header.stamp;

  // first message
  if (!initialized_)
  {
    initialized_ = initialize(scan_msg);

    fixed_origin_to_base_ = fixed_origin_ * fixed_to_base_;

    //pose_covariance_ = twist_covariance_;

    fillMsg(pose_msg);
    fillIncrementMsg(pose_increment_msg);

    ROS_INFO_STREAM_COND(initialized_, "LaserOdometry Initialized!");

    return ProcessReport{true, true};
  }

  preProcessing();

  Transform guess_relative_tf;

  // If an increment prior has been set, 'consum' it.
  // Otherwise predict from previously
  // computed relative_tf_
  if (!guess_relative_tf_.isApprox(Transform::Identity(), 1e-8))
  {
    guess_relative_tf  = guess_relative_tf_;
    guess_relative_tf_ = Transform::Identity();
  }
  else
  {
    guess_relative_tf = predict(relative_tf_);

    if (!isOthogonal(guess_relative_tf))
    {
      ROS_ERROR("guess_relative_tf, rotation matrix"
                " is not orthogonal.");
      guess_relative_tf = Transform::Identity();
    }
  }

  // account for the change since the last kf, in the fixed frame
  guess_relative_tf = guess_relative_tf * (fixed_to_base_ * fixed_to_base_kf_.inverse());

  if (!isOthogonal(guess_relative_tf))
  {
    ROS_ERROR_STREAM("guess_relative_tf, rotation matrix"
              " is not orthogonal.");
  }

  // the predicted change of the laser's position, in the laser frame
  const Transform pred_rel_tf_in_ltf = laser_to_base_ * fixed_to_base_.inverse() *
                                        guess_relative_tf * fixed_to_base_ * base_to_laser_;

  if (!isOthogonal(pred_rel_tf_in_ltf))
  {
    ROS_ERROR_STREAM("pred_rel_tf_in_ltf, rotation matrix"
              " is not orthogonal.");
  }

  // The actual computation
  const bool processed = process_impl(scan_msg, pred_rel_tf_in_ltf);

  if (processed)
  {
    if (!isOthogonal(increment_))
    {
      ROS_ERROR_STREAM("increment_, rotation matrix"
                " is not orthogonal.");
    }

    // the increment of the base's position, in the base frame
    relative_tf_ = base_to_laser_ * increment_ * laser_to_base_;

    if (!isOthogonal(relative_tf_))
    {
      ROS_ERROR_STREAM("relative_tf_, rotation matrix"
                " is not orthogonal.");
    }

    // update the pose in the fixed frame
    fixed_to_base_ = fixed_to_base_kf_ * relative_tf_;

    if (!isOthogonal(fixed_to_base_))
    {
      ROS_ERROR_STREAM("fixed_to_base_, rotation matrix"
                " is not orthogonal.");
    }

    // update the pose in the fixed 'origin' frame
    fixed_origin_to_base_ = fixed_origin_ * fixed_to_base_;

    if (!isOthogonal(fixed_origin_to_base_))
    {
      ROS_ERROR_STREAM("fixed_origin_to_base_, rotation matrix"
                " is not orthogonal.");
    }
  }
  else
  {
    relative_tf_.setIdentity();
    ROS_WARN("Error in laser matching");
  }

  // Retrieve pose2D
  fillMsg(pose_msg);
  fillIncrementMsg(pose_increment_msg);

  has_new_kf_ = isKeyFrame(relative_tf_);

  if (has_new_kf_)
  {
    // generate a keyframe

    fixed_to_base_kf_ = fixed_to_base_;

    reference_scan_ = scan_msg;

    isKeyFrame();
  }
  else
    isNotKeyFrame();

  postProcessing();

  return ProcessReport{processed, has_new_kf_};
}

LaserOdometryBase::ProcessReport
LaserOdometryBase::process(const sensor_msgs::LaserScanConstPtr& scan_ptr,
                           nav_msgs::OdometryPtr odom_ptr,
                           nav_msgs::OdometryPtr odom_increment_msg)
{
  assert_not_null(scan_ptr);

  geometry_msgs::Pose2DPtr pose_2d_ptr = boost::make_shared<geometry_msgs::Pose2D>();

  const auto process_report = process(scan_ptr, pose_2d_ptr);

  fillMsg(odom_ptr);
  fillIncrementMsg(odom_increment_msg);

  return process_report;
}

LaserOdometryBase::ProcessReport
LaserOdometryBase::process(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                           geometry_msgs::Pose2DPtr pose_msg,
                           geometry_msgs::Pose2DPtr pose_increment_msg)
{
  assert_not_null(cloud_msg);

  has_new_kf_   = false;
  current_time_ = cloud_msg->header.stamp;

  // first message
  if (!initialized_)
  {
    initialized_ = initialize(cloud_msg);

    fixed_origin_to_base_ = fixed_origin_ * fixed_to_base_;

    //pose_covariance_ = twist_covariance_;

    fillMsg(pose_msg);
    fillIncrementMsg(pose_increment_msg);

    ROS_INFO_STREAM_COND(initialized_, "LaserOdometry Initialized!");

    return ProcessReport{true, true};
  }

  preProcessing();

  Transform guess_relative_tf;

  // If an increment prior has been set, 'consum' it.
  // Otherwise predict from previously
  // computed relative_tf_
  if (guess_relative_tf_.isApprox(Transform::Identity(), 1e-8))
  {
    guess_relative_tf  = guess_relative_tf_;
    guess_relative_tf_ = Transform::Identity();
  }
  else
  {
    guess_relative_tf = predict(relative_tf_);
  }

  // account for the change since the last kf, in the fixed frame
  guess_relative_tf = guess_relative_tf * (fixed_to_base_ * fixed_to_base_kf_.inverse());

  // the predicted change of the laser's position, in the laser frame
  Transform pred_rel_tf_in_ltf = laser_to_base_ * fixed_to_base_.inverse() *
                                      guess_relative_tf * fixed_to_base_ * base_to_laser_ ;

  // The actual computation
  const bool processed = process_impl(cloud_msg, pred_rel_tf_in_ltf);

  if (processed)
  {
    // the increment of the base's position, in the base frame
    relative_tf_ = base_to_laser_ * increment_ * laser_to_base_;

    // update the pose in the fixed frame
    fixed_to_base_ = fixed_to_base_kf_ * relative_tf_;

    // update the pose in the fixed 'origin' frame
    fixed_origin_to_base_ = fixed_origin_ * fixed_to_base_;
  }
  else
  {
    relative_tf_.setIdentity();
    ROS_WARN("Error in laser matching");
  }

  // Retrieve pose2D
  fillMsg(pose_msg);
  fillIncrementMsg(pose_increment_msg);

  has_new_kf_ = isKeyFrame(increment_);

  if (has_new_kf_)
  {
    // generate a keyframe
    fixed_to_base_kf_ = fixed_to_base_;

    reference_cloud_ = cloud_msg;

    isKeyFrame();
  }
  else
    isNotKeyFrame();

  postProcessing();

  return ProcessReport{processed, has_new_kf_};
}

LaserOdometryBase::ProcessReport
LaserOdometryBase::process(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                           nav_msgs::OdometryPtr odom_ptr,
                           nav_msgs::OdometryPtr odom_increment_msg)
{
  assert_not_null(cloud_msg);

  geometry_msgs::Pose2DPtr pose_2d_ptr = boost::make_shared<geometry_msgs::Pose2D>();

  const auto process_report = process(cloud_msg, pose_2d_ptr);

  fillMsg(odom_ptr);
  fillIncrementMsg(odom_increment_msg);

  return process_report;
}

bool LaserOdometryBase::process_impl(const sensor_msgs::LaserScanConstPtr& /*laser_msg*/,
                                     const Transform& /*prediction*/)
{
  throw std::runtime_error("process_impl(sensor_msgs::LaserScanConstPtr) not implemented.");
}

bool LaserOdometryBase::process_impl(const sensor_msgs::PointCloud2ConstPtr& /*cloud_msg*/,
                                     const Transform& /*prediction*/)
{
  throw std::runtime_error("process_impl(sensor_msgs::PointCloud2ConstPtr) not implemented.");
}

const Transform& LaserOdometryBase::getEstimatedPose() const noexcept
{
  return fixed_origin_to_base_;
}

void LaserOdometryBase::reset()
{
  initialized_ = false;
  has_new_kf_  = false;

  increment_         = Transform::Identity();
  relative_tf_       = Transform::Identity();
  guess_relative_tf_ = Transform::Identity();
  fixed_to_base_kf_  = fixed_to_base_;


  reference_scan_  = nullptr;
  reference_cloud_ = nullptr;
}

void LaserOdometryBase::hardReset()
{
  /// @todo reset configured_ too ? :s
  /// implies reseting 'laser_frame_' etc too.

  reset();

  base_to_laser_     = Transform::Identity();
  laser_to_base_     = Transform::Identity();
  fixed_origin_      = Transform::Identity();
  fixed_to_base_     = Transform::Identity();
}

bool LaserOdometryBase::configured() const noexcept
{
  return configured_;
}

bool LaserOdometryBase::configureImpl()
{
  return true;
}

bool LaserOdometryBase::initialize(const sensor_msgs::LaserScanConstPtr&   /*scan_msg*/)
{
  return true;
}

bool LaserOdometryBase::initialize(const sensor_msgs::PointCloud2ConstPtr& /*cloud_msg*/)
{
  return true;
}

Transform LaserOdometryBase::predict(const Transform& /*tf*/)
{
  return Transform::Identity();
}

void LaserOdometryBase::preProcessing()
{

}

void LaserOdometryBase::postProcessing()
{

}

bool LaserOdometryBase::isKeyFrame(const Transform& /*increment*/)
{
  return true;
}

void LaserOdometryBase::isKeyFrame()
{

}

void LaserOdometryBase::isNotKeyFrame()
{

}

OdomType LaserOdometryBase::odomType() const noexcept
{
  ROS_WARN("odomType() function called but not overloaded!");
  return OdomType::Unknown;
}

bool LaserOdometryBase::hasNewKeyFrame() const noexcept
{
  return has_new_kf_;
}

void LaserOdometryBase::setKeyFrame(const sensor_msgs::LaserScanConstPtr& key_frame_msg)
{
  assert_not_null(key_frame_msg);

  initialized_ = initialize(key_frame_msg);

  fixed_origin_to_base_ = fixed_origin_ * fixed_to_base_;

  //pose_covariance_ = twist_covariance_;

  ROS_INFO_STREAM_COND(initialized_, "LaserOdometry Initialized!");

  /// @todo since we're setting the key-frame
  /// we probably should reset some transforms
  reference_scan_ = key_frame_msg;
}

void LaserOdometryBase::setKeyFrame(const sensor_msgs::PointCloud2ConstPtr& key_frame_msg)
{
  assert_not_null(key_frame_msg);

  initialized_ = initialize(key_frame_msg);

  fixed_origin_to_base_ = fixed_origin_ * fixed_to_base_;

  //pose_covariance_ = twist_covariance_;

  ROS_INFO_STREAM_COND(initialized_, "LaserOdometry Initialized!");

  /// @todo since we're setting the key-frame
  /// we probably should reset some transforms
  reference_cloud_ = key_frame_msg;
}

void LaserOdometryBase::getKeyFrame(sensor_msgs::LaserScanConstPtr& key_frame_msg) const noexcept
{
  key_frame_msg = reference_scan_;
}

void LaserOdometryBase::getKeyFrame(sensor_msgs::PointCloud2ConstPtr& key_frame_msg) const noexcept
{
  key_frame_msg = reference_cloud_;
}

////////////////////////
///                  ///
/// Guetter / Setter ///
///                  ///
////////////////////////

Transform& LaserOdometryBase::getOrigin()
{
  return fixed_origin_;
}

const Transform& LaserOdometryBase::getOrigin() const
{
  return fixed_origin_;
}

void LaserOdometryBase::setOrigin(const Transform& origin)
{
  fixed_origin_ = origin;
}

Transform& LaserOdometryBase::getInitialGuess()
{
  return guess_relative_tf_;
}

const Transform& LaserOdometryBase::getInitialGuess() const
{
  return guess_relative_tf_;
}

void LaserOdometryBase::setInitialGuess(const Transform& guess)
{
  guess_relative_tf_ = guess;
}

Transform& LaserOdometryBase::getLaserPose()
{
  return base_to_laser_;
}

const Transform& LaserOdometryBase::getLaserPose() const
{
  return base_to_laser_;
}

void LaserOdometryBase::setLaserPose(const Transform& base_to_laser)
{
  base_to_laser_ = base_to_laser;
  laser_to_base_ = base_to_laser.inverse();
}

const std::string& LaserOdometryBase::getFrameBase()  const noexcept
{
  return base_frame_;
}

const std::string& LaserOdometryBase::getFrameLaser() const noexcept
{
  return laser_frame_;
}

const std::string& LaserOdometryBase::getFrameFixed() const noexcept
{
  return fixed_frame_;
}

const std::string& LaserOdometryBase::getFrameOdom()  const noexcept
{
  return laser_odom_frame_;
}

void LaserOdometryBase::setFrameBase(const std::string& frame)
{
  base_frame_ = frame;
}

void LaserOdometryBase::setFrameLaser(const std::string& frame)
{
  laser_frame_ = frame;
}

void LaserOdometryBase::setFrameFixed(const std::string& frame)
{
  fixed_frame_ = frame;
}

void LaserOdometryBase::setFrameOdom(const std::string& frame)
{
  laser_odom_frame_ = frame;
}

const ros::Time& LaserOdometryBase::getCurrentTime() const noexcept
{
  return current_time_;
}

} /* namespace laser_odometry */
