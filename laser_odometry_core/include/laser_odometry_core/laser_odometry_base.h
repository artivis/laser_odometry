#ifndef _LASER_ODOMETRY_CORE_LASER_ODOMETRY_BASE_H_
#define _LASER_ODOMETRY_CORE_LASER_ODOMETRY_BASE_H_

// The input ROS messages supported
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

// The output ROS messages supported
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
//#include <geometry_msgs/PoseWithCovarianceStamped.h>

// More ROS header
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

namespace laser_odometry
{
  enum class OdomType : std::size_t
  {
    Odom2D,
    Odom2DCov,
    Odom3D,
    Odom3DCov
  };

  class LaserOdometryBase
  {
  public:

    using covariance_t = geometry_msgs::PoseWithCovariance::_covariance_type;

    struct ProcessReport;

  public:

    LaserOdometryBase()          = default;
    virtual ~LaserOdometryBase() = default;

    virtual ProcessReport process(const sensor_msgs::LaserScanConstPtr& /*cloud_msg*/,
                                  geometry_msgs::Pose2DPtr /*pose_msg*/,
                                  geometry_msgs::Pose2DPtr relative_pose_msg = nullptr);

    virtual ProcessReport process(const sensor_msgs::LaserScanConstPtr& /*cloud_msg*/,
                                  nav_msgs::OdometryPtr /*pose_msg*/,
                                  nav_msgs::OdometryPtr relative_odom_msg = nullptr);

    virtual ProcessReport process(const sensor_msgs::PointCloud2ConstPtr& /*cloud_msg*/,
                                  geometry_msgs::Pose2DPtr /*pose_msg*/,
                                  geometry_msgs::Pose2DPtr relative_pose_msg = nullptr);

    virtual ProcessReport process(const sensor_msgs::PointCloud2ConstPtr& /*cloud_msg*/,
                                  nav_msgs::OdometryPtr /*pose_msg*/,
                                  nav_msgs::OdometryPtr relative_odom_msg = nullptr);

    const tf::Transform& getEstimatedPose() const noexcept;

    virtual void reset();

    bool configure();

    bool configured() const noexcept;

    /* Guetter / Setter */

    tf::Transform& getOrigin();
    const tf::Transform& getOrigin() const;

    void setOrigin(const tf::Transform& origin);

    tf::Transform& getInitialGuess();
    const tf::Transform& getInitialGuess() const;

    void setInitialGuess(const tf::Transform& guess);

    tf::Transform& getLaserPose();
    const tf::Transform& getLaserPose() const;

    void setLaserPose(const tf::Transform& base_to_laser);

    const std::string& getFrameBase()  const noexcept;
    const std::string& getFrameLaser() const noexcept;
    const std::string& getFrameFixed() const noexcept;
    const std::string& getFrameOdom()  const noexcept;

    void setFrameBase(const std::string& frame);
    void setFrameLaser(const std::string& frame);
    void setFrameFixed(const std::string& frame);
    void setFrameOdom(const std::string& frame);

    const ros::Time& getCurrentTime() const noexcept;

    virtual OdomType odomType() const;

  protected:

    bool configured_   = false;
    bool initialized_  = false;
    bool broadcast_tf_ = false;

    covariance_t covariance_;

    ros::NodeHandle private_nh_ = ros::NodeHandle("~");

    std::string base_frame_       = "base_link";
    std::string laser_frame_      = "base_laser_link";
    std::string world_frame_      = "world";
    std::string laser_odom_frame_ = "odom";

    tf::Transform base_to_laser_; // static, cached
    tf::Transform laser_to_base_; // static, cached, calculated from base_to_laser_

    tf::Transform correction;    // last_scan-to-curent_scan tf

    tf::Transform relative_tf_;          // last_scan-to-curent_scan tf
    tf::Transform guess_relative_tf_;    // initial guess last_scan-to-curent_scan tf

    tf::Transform world_to_base_;        // world-to-base tf, integrated odom
    tf::Transform world_to_base_kf_;     // world-to-base in the last key-frame tf
    tf::Transform world_origin_;         // world-origin tf
    tf::Transform world_origin_to_base_; // world-origin-to-base tf, integrated odom

    sensor_msgs::LaserScanConstPtr   reference_scan_;
    sensor_msgs::PointCloud2ConstPtr reference_cloud_;

    ros::Time current_time_;

    virtual bool configureImpl() = 0;

    virtual bool initialize(const sensor_msgs::LaserScanConstPtr&   scan_msg);
    virtual bool initialize(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

    virtual tf::Transform predict(const tf::Transform& tf);

    virtual void preProcessing();

    virtual bool process_impl(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                              const tf::Transform& prediction);

    virtual void postProcessing();

    virtual bool isKeyFrame(const tf::Transform& tf);

    virtual tf::Transform expressFromLaserToBase(const tf::Transform& tf_in_lf);

    template <typename T>
    void fillMsg(T& msg_ptr);
  };

  typedef boost::shared_ptr<LaserOdometryBase> LaserOdometryPtr;

  template <>
  inline void LaserOdometryBase::fillMsg<geometry_msgs::Pose2DPtr>(geometry_msgs::Pose2DPtr& msg_ptr)
  {
    msg_ptr->x = world_origin_to_base_.getOrigin().getX();
    msg_ptr->y = world_origin_to_base_.getOrigin().getY();
    msg_ptr->theta = tf::getYaw(world_origin_to_base_.getRotation());
  }

  template <>
  inline void LaserOdometryBase::fillMsg<nav_msgs::OdometryPtr>(nav_msgs::OdometryPtr& msg_ptr)
  {
    msg_ptr->header.stamp    = current_time_;
    msg_ptr->header.frame_id = laser_odom_frame_;
    msg_ptr->child_frame_id  = base_frame_;

    msg_ptr->pose.pose.position.x = world_origin_to_base_.getOrigin().getX();
    msg_ptr->pose.pose.position.y = world_origin_to_base_.getOrigin().getY();
    msg_ptr->pose.pose.position.z = 0;

    tf::quaternionTFToMsg(world_origin_to_base_.getRotation(),
                          msg_ptr->pose.pose.orientation);

    msg_ptr->pose.covariance = covariance_;
  }

} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_CORE_LASER_ODOMETRY_BASE_H_ */
