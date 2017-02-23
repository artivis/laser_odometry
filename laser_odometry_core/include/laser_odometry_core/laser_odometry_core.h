#ifndef _LASER_ODOMETRY_CORE_LASER_ODOMETRY_H_
#define _LASER_ODOMETRY_CORE_LASER_ODOMETRY_H_

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <property_bag/property_bag.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

namespace laser_odometry
{

  class LaserOdometryBase
  {
  public:

    LaserOdometryBase()          = default;
    virtual ~LaserOdometryBase() = default;

    virtual bool configure(const property_bag::PropertyBag &parameters);

    virtual bool getOrigin();

    virtual bool process(const sensor_msgs::LaserScan& /*scan*/,
                         geometry_msgs::Pose2DPtr /*pose*/) = 0;

    virtual bool process(const sensor_msgs::LaserScan& scan,
                         geometry_msgs::PoseWithCovarianceStampedPtr pose);

    virtual void clear(){
      /* @todo */
      configured_ = false;
    }

  protected:

    bool configured_ = false;
    bool publish_tf_ = false;

    std::string base_frame_;
    std::string laser_frame_;
    std::string world_frame_;

    tf::Transform base_to_laser_; // static, cached
    tf::Transform laser_to_base_; // static, cached, calculated from base_to_laser_

    tf::Transform relative_tf_;   // last_scan-to-curent_scan tf
    tf::Transform world_origin_;  // world-origin tf
    tf::Transform world_to_base_; // world-to-base tf, integrated odom

    sensor_msgs::LaserScan last_scan_;

    ros::Time current_time_;

    tf::TransformBroadcaster tf_broadcaster_;

    virtual tf::Transform predict(const tf::Transform& tf);
    virtual void broadcastTf();
  };

  typedef boost::shared_ptr<LaserOdometryBase> LaserOdometryBasePtr;

} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_CORE_LASER_ODOMETRY_H_ */
