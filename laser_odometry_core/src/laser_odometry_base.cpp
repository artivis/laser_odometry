#include <laser_odometry_core/laser_odometry_core.h>

#include <laser_odometry_core/laser_odometry_utils.h>

#include <boost/assign/list_of.hpp>

#define assert_not_null(x) \
  assert(x != nullptr);

namespace laser_odometry
{

bool LaserOdometryBase::configure()
{
  reset();

  private_nh_.param("laser_frame",      laser_frame_,      std::string("base_laser_link"));
  private_nh_.param("base_frame",       base_frame_,       std::string("base_link"));
  private_nh_.param("odom_frame",       fixed_frame_,      std::string("odom"));
  private_nh_.param("laser_odom_frame", laser_odom_frame_, std::string("odom"));

  // Default covariance diag :
  std::vector<double> default_covariance;
  private_nh_.param("covariance_diag", default_covariance, default_covariance);

  if (default_covariance.size() != 6)
  {
    ROS_WARN_STREAM("Retrieved " << default_covariance.size()
                    << " covariance coeff. Should be 6. Setting default.");

    default_covariance.resize(6);
    std::fill_n(default_covariance.begin(), 6, 1e-8);
  }

  increment_covariance_ = boost::assign::list_of
               (static_cast<double>(default_covariance[0]))  (0)  (0)  (0)  (0) (0)
               (0)  (static_cast<double>(default_covariance[1]))  (0)  (0)  (0) (0)
               (0)  (0)  (static_cast<double>(default_covariance[2]))  (0)  (0) (0)
               (0)  (0)  (0)  (static_cast<double>(default_covariance[3]))  (0) (0)
               (0)  (0)  (0)  (0)  (static_cast<double>(default_covariance[4])) (0)
               (0)  (0)  (0)  (0)  (0)  (static_cast<double>(default_covariance[5]));

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
//  assert_not_null(pose_msg);

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

  tf::Transform guess_relative_tf_ = predict(relative_tf_);

  // account for the change since the last kf, in the fixed frame
  guess_relative_tf_ = guess_relative_tf_ * (fixed_to_base_ * fixed_to_base_kf_.inverse());

  // the predicted change of the laser's position, in the laser frame
  tf::Transform pred_rel_tf_in_ltf = laser_to_base_ * fixed_to_base_.inverse() *
                                      guess_relative_tf_ * fixed_to_base_ * base_to_laser_ ;

  // The actual computation
  const bool processed = process_impl(scan_msg, pred_rel_tf_in_ltf);

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
//  assert_not_null(odom_ptr);

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
//  assert_not_null(pose_msg);

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

  tf::Transform guess_relative_tf_ = predict(relative_tf_);

  // account for the change since the last kf, in the fixed frame
  guess_relative_tf_ = guess_relative_tf_ * (fixed_to_base_ * fixed_to_base_kf_.inverse());

  // the predicted change of the laser's position, in the laser frame
  tf::Transform pred_rel_tf_in_ltf = laser_to_base_ * fixed_to_base_.inverse() *
                                      guess_relative_tf_ * fixed_to_base_ * base_to_laser_ ;

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
//  assert_not_null(odom_ptr);

  geometry_msgs::Pose2DPtr pose_2d_ptr = boost::make_shared<geometry_msgs::Pose2D>();

  const auto process_report = process(cloud_msg, pose_2d_ptr);

  fillMsg(odom_ptr);
  fillIncrementMsg(odom_increment_msg);

  return process_report;
}

bool LaserOdometryBase::process_impl(const sensor_msgs::LaserScanConstPtr& /*laser_msg*/,
                                     const tf::Transform& /*prediction*/)
{
  throw std::runtime_error("process_impl(sensor_msgs::LaserScanConstPtr) not implemented.");
}

bool LaserOdometryBase::process_impl(const sensor_msgs::PointCloud2ConstPtr& /*cloud_msg*/,
                                     const tf::Transform& /*prediction*/)
{
  throw std::runtime_error("process_impl(sensor_msgs::PointCloud2ConstPtr) not implemented.");
}

const tf::Transform& LaserOdometryBase::getEstimatedPose() const noexcept
{
  return fixed_origin_to_base_;
}

void LaserOdometryBase::reset()
{
  increment_         = tf::Transform::getIdentity();

  base_to_laser_     = tf::Transform::getIdentity();
  laser_to_base_     = tf::Transform::getIdentity();
  relative_tf_       = tf::Transform::getIdentity();
  fixed_origin_      = tf::Transform::getIdentity();
  fixed_to_base_     = tf::Transform::getIdentity();
  guess_relative_tf_ = tf::Transform::getIdentity();
  fixed_to_base_kf_  = tf::Transform::getIdentity();

  reference_scan_  = nullptr;
  reference_cloud_ = nullptr;
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

tf::Transform LaserOdometryBase::predict(const tf::Transform& /*tf*/)
{
  return tf::Transform::getIdentity();
}

void LaserOdometryBase::preProcessing()
{

}

void LaserOdometryBase::postProcessing()
{

}

bool LaserOdometryBase::isKeyFrame(const tf::Transform& /*increment*/)
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

void LaserOdometryBase::setKeyFrame(const sensor_msgs::LaserScanConstPtr& kframe)
{
  initialized_ = initialize(kframe);

  fixed_origin_to_base_ = fixed_origin_ * fixed_to_base_;

  //pose_covariance_ = twist_covariance_;

  ROS_INFO_STREAM_COND(initialized_, "LaserOdometry Initialized!");

  /// @todo since we're setting the key-frame
  /// we probably should reset some transforms
  reference_scan_ = kframe;
}

void LaserOdometryBase::setKeyFrame(const sensor_msgs::PointCloud2ConstPtr& kframe)
{
  initialized_ = initialize(kframe);

  fixed_origin_to_base_ = fixed_origin_ * fixed_to_base_;

  //pose_covariance_ = twist_covariance_;

  ROS_INFO_STREAM_COND(initialized_, "LaserOdometry Initialized!");

  /// @todo since we're setting the key-frame
  /// we probably should reset some transforms
  reference_cloud_ = kframe;
}

void LaserOdometryBase::getKeyFrame(sensor_msgs::LaserScanConstPtr& kframe) const noexcept
{
  kframe = reference_scan_;
}

void LaserOdometryBase::getKeyFrame(sensor_msgs::PointCloud2ConstPtr& kframe) const noexcept
{
  kframe = reference_cloud_;
}

////////////////////////
///                  ///
/// Guetter / Setter ///
///                  ///
////////////////////////

tf::Transform& LaserOdometryBase::getOrigin()
{
  return fixed_origin_;
}

const tf::Transform& LaserOdometryBase::getOrigin() const
{
  return fixed_origin_;
}

void LaserOdometryBase::setOrigin(const tf::Transform& origin)
{
  fixed_origin_ = origin;
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

tf::Transform& LaserOdometryBase::getLaserPose()
{
  return base_to_laser_;
}

const tf::Transform& LaserOdometryBase::getLaserPose() const
{
  return base_to_laser_;
}

void LaserOdometryBase::setLaserPose(const tf::Transform& base_to_laser)
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
