#include "laser_odometry_core/laser_odometry_base.h"
#include "laser_odometry_core/laser_odometry_conversion.h"

#include "laser_odometry_core/laser_odometry_utils.h"

// The output ROS messages supported
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>

#define assert_not_null(x) \
  assert(x != nullptr);

#define assert_proper_rotation(x) \
  if (!isRotationProper(x)) \
  { \
    ROS_ERROR_STREAM("l." << __LINE__ << " " << #x \
       << " , rotation matrix" \
       " is not orthogonal positive. Determinant " \
        << x.linear().determinant()); \
  } \

namespace laser_odometry
{

// Some Specialization
// Most of them resolve to passing arg by reference.
template <>
void LaserOdometryBase::fillMsg<std::nullptr_t>(std::nullptr_t&&)
{
  //
}

template <>
void LaserOdometryBase::fillMsg<geometry_msgs::Pose2DPtr&>(geometry_msgs::Pose2DPtr& msg_ptr)
{
  if (msg_ptr == nullptr) return;

  msg_ptr->x = fixed_origin_to_base_.translation()(0);
  msg_ptr->y = fixed_origin_to_base_.translation()(1);
  msg_ptr->theta = utils::getYaw(fixed_origin_to_base_.rotation());
}

template <>
void LaserOdometryBase::fillMsg<nav_msgs::OdometryPtr&>(nav_msgs::OdometryPtr& msg_ptr)
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
void LaserOdometryBase::fillMsg<TransformWithCovariancePtr&>(TransformWithCovariancePtr& msg_ptr)
{
  if (msg_ptr == nullptr) return;

  msg_ptr->transform_  = fixed_origin_to_base_;
  msg_ptr->covariance_ = pose_covariance_;
}

template <>
void LaserOdometryBase::fillIncrementMsg<std::nullptr_t>(std::nullptr_t&&)
{
  //
}

template <>
void LaserOdometryBase::fillIncrementMsg<geometry_msgs::Pose2DPtr&>(geometry_msgs::Pose2DPtr& msg_ptr)
{
  if (msg_ptr == nullptr) return;

  msg_ptr->x = relative_tf_.translation()(0);
  msg_ptr->y = relative_tf_.translation()(1);
  msg_ptr->theta = utils::getYaw(relative_tf_.rotation());
}

template <>
void LaserOdometryBase::fillIncrementMsg<nav_msgs::OdometryPtr&>(nav_msgs::OdometryPtr& msg_ptr)
{
  if (msg_ptr == nullptr) return;

  msg_ptr->header.stamp    = current_time_;
  msg_ptr->header.frame_id = "last_key_frame"; /// @todo this frame does not exist. Should it?
  msg_ptr->child_frame_id  = base_frame_;

  conversion::toRos(relative_tf_, msg_ptr->pose.pose);

  conversion::toRos(increment_covariance_, msg_ptr->pose.covariance);
}

template <>
void LaserOdometryBase::fillIncrementMsg<TransformWithCovariancePtr&>(TransformWithCovariancePtr& msg_ptr)
{
  if (msg_ptr == nullptr) return;

  msg_ptr->transform_  = relative_tf_;
  msg_ptr->covariance_ = increment_covariance_;
}

// Class functions definition

bool LaserOdometryBase::configure()
{
  hardReset();

  private_nh_.param("laser_frame",      laser_frame_,      laser_frame_);
  private_nh_.param("base_frame",       base_frame_,       base_frame_);
  private_nh_.param("odom_frame",       fixed_frame_,      fixed_frame_);
  private_nh_.param("laser_odom_frame", laser_odom_frame_, laser_odom_frame_);

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

  resetCovarianceDefault();

  pose_covariance_ = increment_covariance_;

  // Configure derived class
  configured_ = configureImpl();

  return configured_;
}

LaserOdometryBase::ProcessReport
LaserOdometryBase::process(const sensor_msgs::LaserScanConstPtr& scan_msg)
{
  assert_not_null(scan_msg);

  has_new_kf_   = false;
  current_time_ = scan_msg->header.stamp;

  // first message
  if (!initialized_)
  {
    initialized_ = initialize(scan_msg);

    fixed_origin_to_base_ = fixed_to_base_ * fixed_origin_;

    ROS_INFO_STREAM_COND(initialized_, "LaserOdometry Initialized!");

    return ProcessReport{true, true};
  }

  preProcessing();

  // the predicted change of the laser's position, in the laser frame
//  const Transform pred_rel_tf_in_ltf = laser_to_base_ * fixed_to_base_.inverse() *
//                                        incrementPrior() * fixed_to_base_ * base_to_laser_;

  const Transform pred_rel_tf_in_ltf = laser_to_base_ * getIncrementPrior() * base_to_laser_;

  // The actual computation
  const bool processed = processImpl(scan_msg, pred_rel_tf_in_ltf);

  posePlusIncrement(processed);

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
LaserOdometryBase::process(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  assert_not_null(cloud_msg);

  has_new_kf_   = false;
  current_time_ = cloud_msg->header.stamp;

  // first message
  if (!initialized_)
  {
    initialized_ = initialize(cloud_msg);

    fixed_origin_to_base_ = fixed_to_base_ * fixed_origin_;

    ROS_INFO_STREAM_COND(initialized_, "LaserOdometry Initialized!");

    return ProcessReport{true, true};
  }

  preProcessing();

  // the predicted change of the laser's position, in the laser frame
//  const Transform pred_rel_tf_in_ltf = laser_to_base_ * fixed_to_base_.inverse() *
//                                        incrementPrior() * fixed_to_base_ * base_to_laser_;

  const Transform pred_rel_tf_in_ltf = laser_to_base_ * getIncrementPrior() * base_to_laser_;

  // The actual computation
  const bool processed = processImpl(cloud_msg, pred_rel_tf_in_ltf);

  posePlusIncrement(processed);

  has_new_kf_ = isKeyFrame(relative_tf_);

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

bool LaserOdometryBase::processImpl(const sensor_msgs::LaserScanConstPtr& /*laser_msg*/,
                                    const Transform& /*prediction*/)
{
  throw std::runtime_error("processImpl(sensor_msgs::LaserScanConstPtr) not implemented.");
}

bool LaserOdometryBase::processImpl(const sensor_msgs::PointCloud2ConstPtr& /*cloud_msg*/,
                                    const Transform& /*prediction*/)
{
  throw std::runtime_error("processImpl(sensor_msgs::PointCloud2ConstPtr) not implemented.");
}

Transform LaserOdometryBase::getIncrementPrior()
{
  Transform guess_relative_tf = Transform::Identity();

  // If an increment prior has been set, 'consum' it.
  // Otherwise predict from previously
  // computed relative_tf_
  if (!utils::isIdentity(guess_relative_tf_) &&
       utils::isRotationProper(guess_relative_tf_))
  {
    guess_relative_tf  = guess_relative_tf_;
    guess_relative_tf_ = Transform::Identity();
  }
  else
  {
    guess_relative_tf = predict(relative_tf_);

    if (!utils::isRotationProper(guess_relative_tf))
    {
      utils::makeOrthogonal(guess_relative_tf);
    }
  }

  // account for the change since the last kf, in the fixed frame
  guess_relative_tf = guess_relative_tf * (fixed_to_base_ * fixed_to_base_kf_.inverse());

  return guess_relative_tf;
}

void LaserOdometryBase::posePlusIncrement(const bool processed)
{
  if (processed)
  {
    if (!utils::isRotationProper(increment_), 1e-10)
    {
      ROS_DEBUG_STREAM("increment_'s rotation matrix is not proper.");
      utils::makeOrthogonal(increment_);
    }

    // the increment of the base's position, in the base frame
    relative_tf_ = base_to_laser_ * increment_ * laser_to_base_;

    /// @todo increment_covariance_ in laser frame.
    /// Is it simply rotating the covariance ?
    Eigen::Matrix<Scalar, 6, 6> R = Eigen::Matrix<Scalar, 6, 6>::Identity();
    R.topLeftCorner<3,3>()     = base_to_laser_.rotation();
    R.bottomRightCorner<3,3>() = laser_to_base_.rotation();

    /// @todo if no set by plugin, default covariance spins...
    increment_covariance_ = R * increment_covariance_ * R.inverse();

    // update the pose in the fixed frame
    fixed_to_base_ = fixed_to_base_kf_ * relative_tf_;
//    fixed_to_base_ = relative_tf_ * fixed_to_base_kf_;

    // update the pose in the fixed 'origin' frame
    fixed_origin_to_base_ = fixed_origin_ * fixed_to_base_;
//    fixed_origin_to_base_ = fixed_to_base_ * fixed_origin_;
  }
  else
  {
    relative_tf_.setIdentity();
    ROS_WARN("Error in laser matching.");
  }
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

  base_to_laser_     = Transform::Identity();
  laser_to_base_     = Transform::Identity();
  fixed_origin_      = Transform::Identity();
  fixed_to_base_     = Transform::Identity();

  reset();
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

void LaserOdometryBase::resetCovarianceDefault()
{
  increment_covariance_ = Covariance::Identity();

  increment_covariance_(0,0) = default_cov_diag_[0];
  increment_covariance_(1,1) = default_cov_diag_[1];
  increment_covariance_(2,2) = default_cov_diag_[2];
  increment_covariance_(3,3) = default_cov_diag_[3];
  increment_covariance_(4,4) = default_cov_diag_[4];
  increment_covariance_(5,5) = default_cov_diag_[5];

  //pose_covariance_ = increment_covariance_;
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

////////////////////////
///                  ///
/// Guetter / Setter ///
///                  ///
////////////////////////

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
  if (utils::isRotationProper(origin))
  {
    fixed_origin_ = origin;
  }
  else
  {
    ROS_ERROR("setOrigin:, origin's rotation matrix"
              " is not orthogonal.\nSetting Identity instead.");
    fixed_origin_ = Transform::Identity();
  }
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
  if (utils::isRotationProper(guess))
  {
    guess_relative_tf_ = guess;
  }
  else
  {
    ROS_ERROR("setInitialGuess:, initial guess's rotation matrix"
              " is not orthogonal.\nSetting Identity instead.");
    guess_relative_tf_ = Transform::Identity();
  }
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
  if (utils::isRotationProper(base_to_laser))
  {
    base_to_laser_ = base_to_laser;
    laser_to_base_ = base_to_laser.inverse();
  }
  else
  {
    ROS_ERROR("setLaserPose:, laser pose's rotation matrix"
              " is not orthogonal.\nSetting Identity instead.");
    base_to_laser_ = Transform::Identity();
    laser_to_base_ = Transform::Identity();
  }
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

void LaserOdometryBase::setType(const std::string& type)
{
  laser_odometry_type_ = type;
}

std::string LaserOdometryBase::getType() const noexcept
{
  return laser_odometry_type_;
}

} /* namespace laser_odometry */
