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

  msg_ptr->x = w_T_kf_.translation()(0);
  msg_ptr->y = w_T_kf_.translation()(1);
  msg_ptr->theta = utils::getYaw(w_T_kf_.rotation());
}

template <>
void LaserOdometryBase::fillMsg<nav_msgs::OdometryPtr&>(nav_msgs::OdometryPtr& msg_ptr)
{
  if (msg_ptr == nullptr) return;

  msg_ptr->header.stamp    = current_time_;
  msg_ptr->header.frame_id = laser_odom_frame_;
  msg_ptr->child_frame_id  = base_frame_;

  conversion::toRos(w_T_kf_, msg_ptr->pose.pose);
  conversion::toRos<Covariance>(w_T_kf_cov_ + noise_2d_3d_, msg_ptr->pose.covariance);
}

template <>
void LaserOdometryBase::fillMsg<TransformWithCovariancePtr&>(TransformWithCovariancePtr& msg_ptr)
{
  if (msg_ptr == nullptr) return;

  msg_ptr->transform_  = w_T_kf_;
  msg_ptr->covariance_ = w_T_kf_cov_ + noise_2d_3d_;
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

  const Transform increment_since_keyframe = getIncrementSinceKeyFrame();

  msg_ptr->x = increment_since_keyframe.translation()(0);
  msg_ptr->y = increment_since_keyframe.translation()(1);
  msg_ptr->theta = utils::getYaw(increment_since_keyframe.rotation());
}

template <>
void LaserOdometryBase::fillIncrementMsg<nav_msgs::OdometryPtr&>(nav_msgs::OdometryPtr& msg_ptr)
{
  if (msg_ptr == nullptr) return;

  msg_ptr->header.stamp    = current_time_;
  msg_ptr->header.frame_id = base_frame_;
  msg_ptr->child_frame_id  = laser_frame_;

  Transform  increment_since_kf;
  Covariance increment_since_kf_cov;
  getIncrementSinceKeyFrame(increment_since_kf, increment_since_kf_cov);

  conversion::toRos(increment_since_kf, msg_ptr->pose.pose);
  conversion::toRos<Covariance>(increment_since_kf_cov + noise_2d_3d_, msg_ptr->pose.covariance);
}

template <>
void LaserOdometryBase::fillIncrementMsg<TransformWithCovariancePtr&>(TransformWithCovariancePtr& msg_ptr)
{
  if (msg_ptr == nullptr) return;

  msg_ptr->transform_  = increment_;
  msg_ptr->covariance_ = increment_covariance_ + noise_2d_3d_;
}

template <>
void LaserOdometryBase::updateKfMsg(sensor_msgs::LaserScanConstPtr& msg)
{
  reference_scan_ = msg;
}

template <>
void LaserOdometryBase::updateKfMsg(sensor_msgs::PointCloud2ConstPtr& msg)
{
  reference_cloud_ = msg;
}

} /* namespace laser_odometry */

namespace laser_odometry
{

// Class functions definition

bool LaserOdometryBase::configure()
{
  hardReset();

  utils::getParam(private_nh_, "laser_frame",      laser_frame_,      laser_frame_,      true);
  utils::getParam(private_nh_, "base_frame",       base_frame_,       base_frame_,       true);
  utils::getParam(private_nh_, "fixed_frame",      fixed_frame_,      fixed_frame_,      true);
  utils::getParam(private_nh_, "laser_odom_frame", laser_odom_frame_, laser_odom_frame_, true);

  // Default covariance diag :
  std::vector<Scalar> default_covariance;
  utils::getParam(private_nh_, "covariance_diag", default_covariance, default_cov_diag_, true);

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
  Transform hat_b_T_i = Transform::Identity();

  // If an increment prior has been set, 'consum' it.
  // Otherwise predict from previously
  // computed b_T_i_
  if (!utils::isIdentity(hat_increment_) &&
       utils::isRotationProper(hat_increment_))
  {
    hat_b_T_i = hat_increment_;
    hat_increment_ = Transform::Identity();
  }
  else
  {
    hat_b_T_i = predict(b_T_i_);

    if (!utils::isRotationProper(hat_b_T_i))
    {
      utils::makeOrthogonal(hat_b_T_i);
    }
  }

  // account for the change since the last kf, in the fixed frame
  return getIncrementSinceKeyFrame() * hat_b_T_i;
}

Transform LaserOdometryBase::getIncrementPriorInLaserFrame()
{
  return l_T_b_ * getIncrementPriorInKeyFrame() * b_T_l_;
}

Transform LaserOdometryBase::getIncrementSinceKeyFrame() const
{
  return o_T_kf_.inverse() * o_T_b_;
}

void LaserOdometryBase::getIncrementSinceKeyFrame(Transform& increment,
                                                  Covariance& increment_cov)
{
  /// jacobian of the inverse function
  /// J_Tinv_T = [I -Rtinv[tt]x ; 0 Tr^T]
  kfTo_J_oTkf_.topRightCorner<3,3>()    = -o_T_kf_.rotation()*utils::skew(o_T_kf_.translation());
  kfTo_J_oTkf_.bottomRightCorner<3,3>() =  o_T_kf_.rotation();

  kf_T_o_cov_.noalias() = kfTo_J_oTkf_ * o_T_kf_cov_ * kfTo_J_oTkf_.transpose();

  increment = oplus(o_T_kf_.inverse(), o_T_b_, kfTi_J_kfTo_, kfTi_J_oTb_);

  increment_cov.noalias() =
      kfTi_J_kfTo_ * kf_T_o_cov_ * kfTi_J_kfTo_.transpose() +
      kfTi_J_oTb_  * o_T_b_cov_  * kfTi_J_oTb_.transpose();
}

Transform LaserOdometryBase::oplus(const Transform& A,
                                   const Transform& B,
                                   Jacobian& C_J_A,
                                   Jacobian& C_J_B)
{
  /// C_J_A = [ I -Ra [tb]x ; 0 Ra^T ]
  C_J_A.setIdentity();
  C_J_A.topRightCorner<3,3>()    = -(A.rotation()*utils::skew(B.translation()));
  C_J_A.bottomRightCorner<3,3>() =   A.rotation().transpose();

  /// C_J_B = [ Ra 0 ; 0 I ]
  C_J_B.setIdentity();
  C_J_B.topLeftCorner<3,3>() = A.rotation();

  return A*B;
}

void LaserOdometryBase::posePlusIncrement(const bool processed)
{
  if (processed)
  {
    if (!utils::isRotationProper(increment_, 1e-10))
    {
      ROS_DEBUG_STREAM("increment_'s rotation matrix is not proper.");
      utils::makeOrthogonal(increment_);
    }

    // b_T_it = b_T_l_ * l_T_i
    const Transform b_T_it = oplus(b_T_l_, increment_, bTit_J_bTl_, bTit_J_i_);

    b_T_i_cov_.noalias() =
        bTit_J_bTl_ * b_T_l_cov_            * bTit_J_bTl_.transpose() +
        bTit_J_i_   * increment_covariance_ * bTit_J_i_.transpose();

    // the increment of the base's position, in the base frame
    b_T_i_ = oplus(b_T_it, l_T_b_, bTi_J_lTb_, bTi_J_bTit_);

    b_T_i_cov_ = bTi_J_lTb_  * b_T_i_cov_ * bTi_J_lTb_.transpose() +
                 bTi_J_bTit_ * l_T_b_cov_ * bTi_J_bTit_.transpose();

    // update the pose in the origin frame
    o_T_b_ = oplus(o_T_kf_, b_T_i_, oTb_J_oTkf_, oTb_J_bTi_);

    o_T_b_cov_.noalias() =
        oTb_J_oTkf_ * o_T_kf_cov_ * oTb_J_oTkf_.transpose() +
        oTb_J_bTi_  * b_T_i_cov_  * oTb_J_bTi_.transpose();

    ROS_DEBUG_STREAM("increment_covariance:\n"            << increment_covariance_);
    ROS_DEBUG_STREAM("increment_covariance_in_base:\n"    << b_T_i_cov_);
    ROS_DEBUG_STREAM("fixed_to_base_kf_covariance:\n"     << o_T_kf_cov_);
    ROS_DEBUG_STREAM("fixed_to_base_covariance:\n"        << o_T_b_cov_);
    ROS_DEBUG_STREAM("fixed_origin_covariance:\n"         << w_T_o_cov_);
    ROS_DEBUG_STREAM("fixed_origin_to_base_covariance:\n" << w_T_kf_cov_);
  }
  else
  {
    b_T_i_.setIdentity();
    b_T_i_cov_.setZero();
    ROS_WARN("Error in laser matching.");
  }
}

void LaserOdometryBase::updateKfTransform()
{
  o_T_kf_ = o_T_b_;
  o_T_kf_cov_ = o_T_b_cov_;

  // update the pose in the world frame
  w_T_kf_ = oplus(w_T_o_, o_T_kf_, wTb_J_wTo_, wTb_J_oTkf_);

  w_T_kf_cov_ = wTb_J_wTo_  * w_T_o_cov_  * wTb_J_wTo_.transpose() +
                wTb_J_oTkf_ * o_T_kf_cov_ * wTb_J_oTkf_.transpose();
}

void LaserOdometryBase::reset()
{
  initialized_ = false;
  has_new_kf_  = false;

  increment_     = Transform::Identity();
  hat_increment_ = Transform::Identity();
  b_T_i_         = Transform::Identity();
  o_T_kf_        = o_T_b_;

  reference_scan_  = nullptr;
  reference_cloud_ = nullptr;
}

void LaserOdometryBase::hardReset()
{
  /// @todo reset configured_ too ? :s
  /// implies reseting 'laser_frame_' etc too.

  b_T_l_     = Transform::Identity();
  l_T_b_     = Transform::Identity();
  w_T_o_     = Transform::Identity();
  o_T_b_     = Transform::Identity();

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

void LaserOdometryBase::initializeFrames()
{
  // update the pose in the fixed 'origin' frame
  w_T_kf_ = oplus(w_T_o_, o_T_b_, wTb_J_wTo_, wTb_J_oTb_);

  w_T_kf_cov_.noalias() =
      wTb_J_wTo_ * w_T_o_cov_  * wTb_J_wTo_.transpose() +
      wTb_J_oTb_ * o_T_b_cov_  * wTb_J_oTb_.transpose() ;
}

Transform LaserOdometryBase::predict(const Transform& /*tf*/)
{
  return Transform::Identity();
}

void LaserOdometryBase::preProcessing()
{
  //
}

void LaserOdometryBase::postProcessing()
{
  //
}

bool LaserOdometryBase::isKeyFrame(const Transform& /*increment*/)
{
  return true;
}

void LaserOdometryBase::isKeyFrame()
{
  //
}

void LaserOdometryBase::isNotKeyFrame()
{
  //
}

void LaserOdometryBase::resetCovarianceDefault()
{
  increment_covariance_ = defaultCovariance();

  w_T_o_cov_  = defaultCovariance();
  w_T_kf_cov_ = defaultCovariance();
  o_T_b_cov_  = defaultCovariance();
  o_T_kf_cov_ = defaultCovariance();

  l_T_b_cov_ = defaultCovariance();
  b_T_l_cov_ = defaultCovariance();
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
  return w_T_kf_;
}

void LaserOdometryBase::getEstimatedPose(Transform& estimated_pose,
                                         Covariance& estimated_pose_covariance) const noexcept
{
  estimated_pose = w_T_kf_;
  estimated_pose_covariance = w_T_kf_cov_ + noise_2d_3d_;
}

void LaserOdometryBase::setKeyFrame(const sensor_msgs::LaserScanConstPtr& key_frame_msg)
{
  if (key_frame_msg == nullptr)
  {
    ROS_WARN("Laser odometry setKeyFrame function received a nullptr input message!");
    return;
  }

  initialized_ = initialize(key_frame_msg);

  w_T_kf_ = w_T_o_ * o_T_b_;

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

  w_T_kf_ = w_T_o_ * o_T_b_;

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
  return w_T_o_;
}

void LaserOdometryBase::getOrigin(Transform& origin,
                                  Covariance& origin_covariance) const
{
  origin = w_T_o_;
  origin_covariance = w_T_o_cov_;
}

void LaserOdometryBase::setOrigin(const Transform& origin,
                                  const Covariance& origin_covariance)
{
  if (utils::isRotationProper(origin))
  {
    w_T_o_ = origin;

    if (utils::isCovariance(origin_covariance))
    {
      w_T_o_cov_ = origin_covariance;
    }
    else
    {
      ROS_WARN("setOrigin:, origin's covariance matrix"
               " is not proper.\nSetting default instead.");
      w_T_o_cov_= defaultCovariance();
    }
  }
  else
  {
    ROS_ERROR("setOrigin:, origin's rotation matrix"
              " is not orthogonal.\nSetting Identity instead.");
    w_T_o_ = Transform::Identity();
  }
}

void LaserOdometryBase::setOrigin(const Transform& origin)
{
  setOrigin(origin, defaultCovariance());
}

const Transform& LaserOdometryBase::getIncrementPrior() const
{
  return hat_increment_;
}

void LaserOdometryBase::setIncrementPrior(const Transform& hat_increment)
{
  if (utils::isRotationProper(hat_increment))
  {
    hat_increment_ = hat_increment;
  }
  else
  {
    ROS_ERROR("setInitialGuess:, initial guess's rotation matrix"
              " is not orthogonal.\nSetting Identity instead.");
    hat_increment_ = Transform::Identity();
  }
}

const Transform& LaserOdometryBase::getLaserPose() const
{
  return b_T_l_;
}

void LaserOdometryBase::getLaserPose(Transform& base_to_laser,
                                     Covariance& base_to_laser_covariance) const
{
  base_to_laser = b_T_l_;
  base_to_laser_covariance = b_T_l_cov_;
}

void LaserOdometryBase::setLaserPose(const Transform& base_to_laser,
                                     const Covariance& base_to_laser_covariance)
{
  if (utils::isRotationProper(base_to_laser))
  {
    b_T_l_ = base_to_laser;
    l_T_b_ = base_to_laser.inverse();

    Covariance tmp = base_to_laser_covariance;

    if (!utils::isCovariance(tmp))
    {
      ROS_ERROR("setLaserPose:, laser pose's covariance matrix"
                " is not proper.\nSetting default instead.");

      tmp = defaultCovariance();
    }

    b_T_l_cov_ = tmp;

    /// jacobian of the inverse function
    /// J_Tinv_T = [I -Rtinv[tt]x ; 0 Tr^T]
    Jacobian lTb_J_bTl = Jacobian::Identity();
    lTb_J_bTl.topRightCorner<3,3>()    = -l_T_b_.rotation()*utils::skew(b_T_l_.translation());
    lTb_J_bTl.bottomRightCorner<3,3>() =  b_T_l_.rotation();

    l_T_b_cov_ = lTb_J_bTl * base_to_laser_covariance * lTb_J_bTl.transpose();
  }
  else
  {
    ROS_ERROR("setLaserPose:, laser pose's rotation matrix"
              " is not orthogonal.\nSetting Identity instead.");
    b_T_l_ = Transform::Identity();
    l_T_b_ = Transform::Identity();

    b_T_l_cov_ = defaultCovariance();
    l_T_b_cov_ = defaultCovariance();
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
