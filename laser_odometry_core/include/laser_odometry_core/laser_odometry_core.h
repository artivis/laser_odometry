#ifndef _LASER_ODOMETRY_CORE_LASER_ODOMETRY_CORE_H_
#define _LASER_ODOMETRY_CORE_LASER_ODOMETRY_CORE_H_

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

namespace laser_odometry
{

  //Forward declaration
  class LaserOdometry;

  class LaserOdometryBase
  {
    friend class LaserOdometry;

  public:

    LaserOdometryBase()          = default;
    virtual ~LaserOdometryBase() = default;

    virtual bool process(const sensor_msgs::LaserScanPtr /*scan_ptr*/,
                         geometry_msgs::Pose2DPtr /*pose_ptr*/) = 0;

    virtual bool process(const sensor_msgs::LaserScanPtr scan_ptr,
                         geometry_msgs::PosePtr pose_ptr);

    virtual bool process(const sensor_msgs::LaserScanPtr scan_ptr,
                         geometry_msgs::PoseWithCovariancePtr pose_ptr);

    virtual bool process(const sensor_msgs::LaserScanPtr scan_ptr,
                         geometry_msgs::PoseWithCovarianceStampedPtr pose_ptr);

    bool configure();

    bool configured() const noexcept;

    tf::Transform& getOrigin();
    const tf::Transform& getOrigin() const;

    void setOrigin(const tf::Transform& origin);

  protected:

    bool configured_   = false;
    bool broadcast_tf_ = false;

    std::vector<double> default_covariance_;

    ros::NodeHandle private_nh_ = ros::NodeHandle("~");

    std::string base_frame_  = "base_link";
    std::string laser_frame_ = "base_laser_link";
    std::string world_frame_ = "world";
    std::string laser_odom_frame_ = "laser_odom";

    tf::Transform base_to_laser_; // static, cached
    tf::Transform laser_to_base_; // static, cached, calculated from base_to_laser_

    tf::Transform relative_tf_;   // last_scan-to-curent_scan tf
    tf::Transform world_origin_;  // world-origin tf
    tf::Transform world_to_base_; // world-to-base tf, integrated odom

    sensor_msgs::LaserScan last_scan_;

    ros::Time current_time_;

    virtual bool configureImpl() = 0;
    virtual tf::Transform predict(const tf::Transform& tf);

    using Covariance = geometry_msgs::PoseWithCovariance::_covariance_type;
    void fillCovariance(Covariance& covariance);
  };

  typedef boost::shared_ptr<LaserOdometryBase> LaserOdometryBasePtr;

} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_CORE_LASER_ODOMETRY_CORE_H_ */
