#ifndef _LASER_ODOMETRY_CORE_LASER_ODOMETRY_CORE_H_
#define _LASER_ODOMETRY_CORE_LASER_ODOMETRY_CORE_H_

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

namespace laser_odometry
{

  class LaserOdometryBase
  {
  public:

    LaserOdometryBase()          = default;
    virtual ~LaserOdometryBase() = default;

    virtual bool process(const sensor_msgs::LaserScanPtr /*scan_ptr*/,
                         geometry_msgs::Pose2DPtr /*pose_ptr*/,
                         geometry_msgs::Pose2DPtr relative_pose_ptr = nullptr) = 0;

    virtual bool process(const sensor_msgs::LaserScanPtr /*scan_ptr*/,
                         nav_msgs::OdometryPtr /*odom_ptr*/,
                         nav_msgs::OdometryPtr relative_odom_ptr = nullptr);

    tf::Transform getEstimatedPose() const noexcept;

    virtual void reset();

    bool configure();

    bool configured() const noexcept;

    tf::Transform& getOrigin();
    const tf::Transform& getOrigin() const;

    void setOrigin(const tf::Transform& origin);

    tf::Transform& getInitialGuess();
    const tf::Transform& getInitialGuess() const;

    void setInitialGuess(const tf::Transform& guess);

    tf::Transform& getLaserPose();
    const tf::Transform& getLaserPose() const;

    void setLaserPose(const tf::Transform& base_to_laser);

    std::string getFrameBase()  const noexcept;
    std::string getFrameLaser() const noexcept;
    std::string getFrameFixed() const noexcept;
    std::string getFrameOdom()  const noexcept;

    void setFrameBase(const std::string& frame);
    void setFrameLaser(const std::string& frame);
    void setFrameFixed(const std::string& frame);
    void setFrameOdom(const std::string& frame);

    ros::Time getCurrentTime() const noexcept;

  protected:

    bool configured_   = false;
    bool broadcast_tf_ = false;

    std::vector<double> default_covariance_;

    ros::NodeHandle private_nh_ = ros::NodeHandle("~");

    std::string base_frame_  = "base_link";
    std::string laser_frame_ = "base_laser_link";
    std::string world_frame_ = "world";
    std::string laser_odom_frame_ = "odom";

    tf::Transform base_to_laser_; // static, cached
    tf::Transform laser_to_base_; // static, cached, calculated from base_to_laser_

    tf::Transform relative_tf_;          // last_scan-to-curent_scan tf
    tf::Transform guess_relative_tf_;    // initial guess last_scan-to-curent_scan tf
    tf::Transform world_to_base_;        // world-to-base tf, integrated odom
    tf::Transform world_origin_;         // world-origin tf
    tf::Transform world_origin_to_base_; // world-origin-to-base tf, integrated odom

    sensor_msgs::LaserScan reference_scan_;

    ros::Time current_time_;

    virtual bool configureImpl() = 0;
    virtual tf::Transform predict(const tf::Transform& tf);

    virtual void fillOdomMsg(const sensor_msgs::LaserScanPtr current_scan_ptr,
                             nav_msgs::OdometryPtr odom_ptr);

    virtual void fillPose2DMsg(geometry_msgs::Pose2DPtr pose_ptr);

    using Covariance = geometry_msgs::PoseWithCovariance::_covariance_type;
    void fillCovariance(Covariance& covariance);
  };

  typedef boost::shared_ptr<LaserOdometryBase> LaserOdometryPtr;

} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_CORE_LASER_ODOMETRY_CORE_H_ */
