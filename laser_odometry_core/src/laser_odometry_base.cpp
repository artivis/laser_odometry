#include "laser_odometry_core/laser_odometry_base.h"
#include "laser_odometry_core/laser_odometry_conversion.h"

#include "laser_odometry_core/laser_odometry_utils.h"

// The output ROS messages supported
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>

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
  conversion::toRos(fixed_origin_to_base_covariance_, msg_ptr->pose.covariance);

  //msg_ptr->pose.covariance  = pose_covariance_;
  //msg_ptr->twist.covariance = pose_twist_covariance_;
}

template <>
void LaserOdometryBase::fillMsg<TransformWithCovariancePtr&>(TransformWithCovariancePtr& msg_ptr)
{
  if (msg_ptr == nullptr) return;

  msg_ptr->transform_  = fixed_origin_to_base_;
  msg_ptr->covariance_ = fixed_origin_to_base_covariance_;
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

  msg_ptr->x = increment_in_base_.translation()(0);
  msg_ptr->y = increment_in_base_.translation()(1);
  msg_ptr->theta = utils::getYaw(increment_in_base_.rotation());
}

template <>
void LaserOdometryBase::fillIncrementMsg<nav_msgs::OdometryPtr&>(nav_msgs::OdometryPtr& msg_ptr)
{
  if (msg_ptr == nullptr) return;

  msg_ptr->header.stamp    = current_time_;
  msg_ptr->header.frame_id = "last_key_frame"; /// @todo this frame does not exist. Should it?
  msg_ptr->child_frame_id  = base_frame_;

  conversion::toRos(increment_in_base_, msg_ptr->pose.pose);
  conversion::toRos(increment_covariance_in_base_, msg_ptr->pose.covariance);
}

template <>
void LaserOdometryBase::fillIncrementMsg<TransformWithCovariancePtr&>(TransformWithCovariancePtr& msg_ptr)
{
  if (msg_ptr == nullptr) return;

  msg_ptr->transform_  = increment_in_base_;
  msg_ptr->covariance_ = increment_covariance_in_base_;
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

      ROS_INFO_STREAM("Default covariance diagonal: ["
                      << default_cov_diag_[0] << ","
                      << default_cov_diag_[1] << ","
                      << default_cov_diag_[2] << "].");
    }
    else if (default_covariance.size() == 6)
    {
      default_cov_diag_.swap(default_covariance);

      if (odomType() == OdomType::Odom2D or odomType() == OdomType::Odom2DCov)
      {
        default_cov_diag_[2] = 0;
        default_cov_diag_[3] = 0;
        default_cov_diag_[4] = 0;
      }

      ROS_INFO_STREAM("Default covariance diagonal: ["
                      << default_cov_diag_[0] << ","
                      << default_cov_diag_[1] << ","
                      << default_cov_diag_[2] << ","
                      << default_cov_diag_[3] << ","
                      << default_cov_diag_[4] << ","
                      << default_cov_diag_[5] << "].");
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

  // Configure derived class
  configured_ = configureImpl();

  return configured_;
}

LaserOdometryBase::ProcessReport
LaserOdometryBase::process(const sensor_msgs::LaserScanConstPtr& scan_msg)
{
  if (scan_msg == nullptr)
  {
    ROS_WARN("Laser odometry process function received a nullptr input message!");
    return ProcessReport::ErrorReport();
  }

  ros::WallTime start = ros::WallTime::now();

  has_new_kf_   = false;
  current_time_ = scan_msg->header.stamp;

  // first message
  if (!initialized_)
  {
    initialized_ = initialize(scan_msg);

    // update the pose in the fixed 'origin' frame
    fixed_origin_to_base_ = fixed_origin_ * fixed_to_base_;

    // the right jacobian of the above pose composition
    Eigen::Matrix<Scalar, 6, 6> jac_right_comp_orig = Eigen::Matrix<Scalar, 6, 6>::Identity();
    jac_right_comp_orig.bottomRightCorner<3,3>() = fixed_to_base_.rotation().transpose();
    jac_right_comp_orig.topRightCorner<3,3>() = - (fixed_origin_.rotation()*utils::skew(fixed_to_base_.translation()));

    // the left jacobian of the above pose composition
    Eigen::Matrix<Scalar, 6, 6> jac_left_comp_orig = Eigen::Matrix<Scalar, 6, 6>::Identity();
    jac_left_comp_orig.topLeftCorner<3,3>() = fixed_origin_.rotation();

    fixed_origin_to_base_covariance_ = jac_right_comp_orig * fixed_to_base_covariance_ * jac_left_comp_orig + fixed_origin_covariance_;

    ROS_INFO_STREAM_COND(initialized_, "LaserOdometry Initialized!");

    return ProcessReport{true, true};
  }

  preProcessing();

  // the predicted change of the laser's position, in the laser frame
  const Transform increment_prior_in_laser = getIncrementPriorInLaserFrame();

  // The actual computation
  const bool processed = processImpl(scan_msg, increment_prior_in_laser);

  assertIncrement();
  assertIncrementCovariance();

  posePlusIncrement(processed);

  has_new_kf_ = isKeyFrame(increment_in_base_);

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

  execution_time_ = ros::WallTime::now() - start;

  return ProcessReport{processed, has_new_kf_};
}

LaserOdometryBase::ProcessReport
LaserOdometryBase::process(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  if (cloud_msg == nullptr)
  {
    ROS_WARN("Laser odometry process function received a nullptr input message!");
    return ProcessReport::ErrorReport();
  }

  ros::WallTime start = ros::WallTime::now();

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
  const Transform increment_prior_in_laser = getIncrementPriorInLaserFrame();

  // The actual computation
  const bool processed = processImpl(cloud_msg, increment_prior_in_laser);

  assertIncrement();
  assertIncrementCovariance();

  posePlusIncrement(processed);

  has_new_kf_ = isKeyFrame(increment_in_base_);

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

  execution_time_ = ros::WallTime::now() - start;

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

Transform LaserOdometryBase::getIncrementPriorInKeyFrame()
{
  Transform increment_in_base_prior = Transform::Identity();

  // If an increment prior has been set, 'consum' it.
  // Otherwise predict from previously
  // computed increment_in_base_
  if (!utils::isIdentity(increment_in_base_prior_) &&
       utils::isRotationProper(increment_in_base_prior_))
  {
    increment_in_base_prior  = increment_in_base_prior_;
    increment_in_base_prior_ = Transform::Identity();
  }
  else
  {
    increment_in_base_prior = predict(increment_in_base_);

    if (!utils::isRotationProper(increment_in_base_prior))
    {
      utils::makeOrthogonal(increment_in_base_prior);
    }
  }

  // account for the change since the last kf, in the fixed frame
  increment_in_base_prior = increment_in_base_prior * (fixed_to_base_ * fixed_to_base_kf_.inverse());

  return increment_in_base_prior;
}

Transform LaserOdometryBase::getIncrementPriorInLaserFrame()
{
  return laser_to_base_ * getIncrementPriorInKeyFrame() * base_to_laser_;

//  return laser_to_base_ * fixed_to_base_.inverse() *
//          incrementPrior() * fixed_to_base_ * base_to_laser_;
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
    const Transform tmp = base_to_laser_ * increment_;
    increment_in_base_ = tmp * laser_to_base_;

    // the left jacobian of the above pose composition
    Eigen::Matrix<Scalar, 6, 6> jac_left = Eigen::Matrix<Scalar, 6, 6>::Identity();
    jac_left.bottomRightCorner<3,3>()    = increment_.rotation().transpose();
    jac_left.topRightCorner<3,3>()       = -(base_to_laser_.rotation()*utils::skew(increment_.translation()));

    // the right jacobian of the above pose composition
    Eigen::Matrix<Scalar, 6, 6> jac_right = Eigen::Matrix<Scalar, 6, 6>::Identity();
    jac_right.topLeftCorner<3,3>()        = base_to_laser_.rotation();

    increment_covariance_in_base_ = jac_left  * increment_covariance_     * jac_left.transpose() +
                                    jac_right * base_to_laser_covariance_ * jac_right.transpose();

    // the left jacobian of the above pose composition
    jac_left.bottomRightCorner<3,3>() = laser_to_base_.rotation().transpose();
    jac_left.topRightCorner<3,3>()    = -(tmp.rotation()*utils::skew(laser_to_base_.translation()));

    // the right jacobian of the above pose composition (tmp)
    jac_right.topLeftCorner<3,3>() = tmp.rotation();

    increment_covariance_in_base_ = jac_left  * laser_to_base_covariance_     * jac_left.transpose() +
                                    jac_right * increment_covariance_in_base_ * jac_right.transpose();

    // update the pose in the fixed frame
    fixed_to_base_ = fixed_to_base_kf_ * increment_in_base_;

    // the left jacobian of the above pose composition
    jac_left.bottomRightCorner<3,3>() = increment_in_base_.rotation().transpose();
    jac_left.topRightCorner<3,3>() = -(fixed_to_base_kf_.rotation()*utils::skew(increment_in_base_.translation()));

    // the right jacobian of the above pose composition
    jac_right.topLeftCorner<3,3>() = fixed_to_base_kf_.rotation();

    fixed_to_base_covariance_ = jac_left  * increment_covariance_in_base_ * jac_left.transpose() +
                                jac_right * fixed_to_base_kf_covariance_  * jac_right.transpose();

    // update the pose in the fixed 'origin' frame
    fixed_origin_to_base_ = fixed_origin_ * fixed_to_base_;

    // the left jacobian of the above pose composition
    jac_left.bottomRightCorner<3,3>() = fixed_to_base_.rotation().transpose();
    jac_left.topRightCorner<3,3>() = - (fixed_origin_.rotation()*utils::skew(fixed_to_base_.translation()));

    // the right jacobian of the above pose composition
    jac_right.topLeftCorner<3,3>() = fixed_origin_.rotation();

    fixed_origin_to_base_covariance_ = jac_left  * fixed_to_base_covariance_ * jac_left.transpose() +
                                       jac_right * fixed_origin_covariance_  * jac_right.transpose();

    ROS_DEBUG_STREAM("increment_covariance_:\n"            << increment_covariance_);
    ROS_DEBUG_STREAM("increment_covariance_in_base_:\n"    << increment_covariance_in_base_);
    ROS_DEBUG_STREAM("fixed_to_base_kf_covariance_:\n"     << fixed_to_base_kf_covariance_);
    ROS_DEBUG_STREAM("fixed_to_base_covariance_:\n"        << fixed_to_base_covariance_);
    ROS_DEBUG_STREAM("fixed_origin_covariance_:\n"         << fixed_origin_covariance_);
    ROS_DEBUG_STREAM("fixed_origin_to_base_covariance_:\n" << fixed_origin_to_base_covariance_);
  }
  else
  {
    increment_in_base_.setIdentity();
    increment_covariance_in_base_.setZero();
    ROS_WARN("Error in laser matching.");
  }
}

void LaserOdometryBase::reset()
{
  initialized_ = false;
  has_new_kf_  = false;

  increment_         = Transform::Identity();
  increment_in_base_       = Transform::Identity();
  increment_in_base_prior_ = Transform::Identity();
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
  increment_covariance_ = defaultCovariance();

  fixed_origin_covariance_         = defaultCovariance();
  fixed_origin_to_base_covariance_ = defaultCovariance();
  fixed_to_base_covariance_        = defaultCovariance();
  fixed_to_base_kf_covariance_     = defaultCovariance();

  laser_to_base_covariance_ = defaultCovariance();
  base_to_laser_covariance_ = defaultCovariance();
}

OdomType LaserOdometryBase::odomType() const noexcept
{
  ROS_WARN_THROTTLE(1, "odomType() function called but not overloaded!");
  return OdomType::Unknown;
}

bool LaserOdometryBase::hasNewKeyFrame() const noexcept
{
  return has_new_kf_;
}

void LaserOdometryBase::assertIncrement()
{
  if (!utils::isRotationProper(increment_))
  {
    utils::makeOrthogonal(increment_);
  }
}

void LaserOdometryBase::assertIncrementCovariance()
{
  if (!utils::isCovariance(increment_covariance_))
  {
    increment_covariance_ = defaultCovariance();
  }
}

Covariance LaserOdometryBase::defaultCovariance() const noexcept
{
  Covariance default_covariance = Covariance::Zero();

  switch (odomType())
  {
  case OdomType::Unknown:
    ///@todo Not sure what's better here ...
    break;
  case OdomType::Odom2D:
    default_covariance(0,0) = default_cov_diag_val;
    default_covariance(1,1) = default_cov_diag_val;
    default_covariance(5,5) = default_cov_diag_val;
    break;
  case OdomType::Odom2DCov:
    default_covariance(0,0) = default_cov_diag_val;
    default_covariance(1,1) = default_cov_diag_val;
    default_covariance(5,5) = default_cov_diag_val;
    break;
  case OdomType::Odom3D:
    default_covariance = Covariance::Identity() * default_cov_diag_val;
    break;
  case OdomType::Odom3DCov:
    default_covariance = Covariance::Identity() * default_cov_diag_val;
    break;
  default:
    break;
  }

  return default_covariance;
}

////////////////////////
///                  ///
/// Guetter / Setter ///
///                  ///
////////////////////////

const Transform& LaserOdometryBase::getEstimatedPose() const noexcept
{
  return fixed_origin_to_base_;
}

void LaserOdometryBase::getEstimatedPose(Transform& estimated_pose,
                                         Covariance& estimated_pose_covariance) const noexcept
{
  estimated_pose = fixed_origin_to_base_;
  estimated_pose_covariance = fixed_origin_to_base_covariance_;
}

/**
 * @brief Return the last key-frame estimated pose.
 * @return A const-reference Transform.
 */
const Transform& LaserOdometryBase::getKeyFrameEstimatedPose() const noexcept
{
  return fixed_to_base_kf_;
}

/**
 * @brief Get the origin frame together with it's covariance matrix.
 * @param origin The origin transform.
 * @param origin_covariance The origin transform's covariance.
 */
void LaserOdometryBase::getKeyFrameEstimatedPose(Transform& kf_estimated_pose,
                                                 Covariance& kf_estimated_pose_covariance) const noexcept
{
  kf_estimated_pose = fixed_to_base_kf_;
  kf_estimated_pose_covariance = fixed_to_base_kf_covariance_;
}

void LaserOdometryBase::setKeyFrame(const sensor_msgs::LaserScanConstPtr& key_frame_msg)
{
  if (key_frame_msg == nullptr)
  {
    ROS_WARN("Laser odometry setKeyFrame function received a nullptr input message!");
    return;
  }

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
  if (key_frame_msg == nullptr)
  {
    ROS_WARN("Laser odometry setKeyFrame function received a nullptr input message!");
    return;
  }

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

const Transform& LaserOdometryBase::getOrigin() const
{
  return fixed_origin_;
}

void LaserOdometryBase::getOrigin(Transform& origin,
                                  Covariance& origin_covariance) const
{
  origin = fixed_origin_;
  origin_covariance = fixed_origin_covariance_;
}

void LaserOdometryBase::setOrigin(const Transform& origin,
                                  const Covariance& origin_covariance)
{
  if (utils::isRotationProper(origin))
  {
    fixed_origin_ = origin;

    if (utils::isCovariance(origin_covariance))
    {
      fixed_origin_covariance_ = origin_covariance;
    }
    else
    {
      ROS_WARN("setOrigin:, origin's covariance matrix"
               " is not proper.\nSetting default instead.");
      fixed_origin_covariance_= defaultCovariance();
    }
  }
  else
  {
    ROS_ERROR("setOrigin:, origin's rotation matrix"
              " is not orthogonal.\nSetting Identity instead.");
    fixed_origin_ = Transform::Identity();
  }
}

void LaserOdometryBase::setOrigin(const Transform& origin)
{
  setOrigin(origin, defaultCovariance());
}

const Transform& LaserOdometryBase::getIncrementPrior() const
{
  return increment_in_base_prior_;
}

void LaserOdometryBase::setIncrementPrior(const Transform& increment_in_base_prior)
{
  if (utils::isRotationProper(increment_in_base_prior))
  {
    increment_in_base_prior_ = increment_in_base_prior;
  }
  else
  {
    ROS_ERROR("setInitialGuess:, initial guess's rotation matrix"
              " is not orthogonal.\nSetting Identity instead.");
    increment_in_base_prior_ = Transform::Identity();
  }
}

const Transform& LaserOdometryBase::getLaserPose() const
{
  return base_to_laser_;
}

void LaserOdometryBase::getLaserPose(Transform& base_to_laser,
                                     Covariance& base_to_laser_covariance) const
{
  base_to_laser = base_to_laser_;
  base_to_laser_covariance = base_to_laser_covariance_;
}

void LaserOdometryBase::setLaserPose(const Transform& base_to_laser,
                                     const Covariance& base_to_laser_covariance)
{
  if (utils::isRotationProper(base_to_laser))
  {
    base_to_laser_ = base_to_laser;
    laser_to_base_ = base_to_laser.inverse();

    Covariance tmp = base_to_laser_covariance;

    if (!utils::isCovariance(tmp))
    {
      ROS_ERROR("setLaserPose:, laser pose's covariance matrix"
                " is not proper.\nSetting default instead.");

      tmp = defaultCovariance();
    }

    base_to_laser_covariance_ = tmp;

    // the left jacobian of the inverse
    Eigen::Matrix<Scalar, 6, 6> jac_left_inv = Eigen::Matrix<Scalar, 6, 6>::Identity();
    jac_left_inv.topLeftCorner<3,3>() = laser_to_base_.rotation();
    jac_left_inv.topRightCorner<3,3>() = utils::skew(laser_to_base_.translation()) * laser_to_base_.rotation();
    jac_left_inv.bottomRightCorner<3,3>() = laser_to_base_.rotation();

    // the right jacobian of the above pose composition
    //Eigen::Matrix<Scalar, 6, 6> jac_right_inv = jac_left_inv.transpose();

    laser_to_base_covariance_ = jac_left_inv * base_to_laser_covariance * jac_left_inv.transpose();
  }
  else
  {
    ROS_ERROR("setLaserPose:, laser pose's rotation matrix"
              " is not orthogonal.\nSetting Identity instead.");
    base_to_laser_ = Transform::Identity();
    laser_to_base_ = Transform::Identity();

    base_to_laser_covariance_ = defaultCovariance();
    laser_to_base_covariance_ = defaultCovariance();
  }
}

void LaserOdometryBase::setLaserPose(const Transform& base_to_laser)
{
  setLaserPose(base_to_laser, defaultCovariance());
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

const ros::WallDuration& LaserOdometryBase::getExecutionTime() const noexcept
{
  return execution_time_;
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
