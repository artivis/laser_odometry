#ifndef _LASER_ODOMETRY_CORE_LASER_ODOMETRY_NODE_H_
#define _LASER_ODOMETRY_CORE_LASER_ODOMETRY_NODE_H_

#include <laser_odometry_core/laser_odometry_core.h>

namespace laser_odometry
{

class LaserOdometryNode
{
public:

  LaserOdometryNode();
  virtual ~LaserOdometryNode() = default;

  void initialize();

  void LaserCallback(sensor_msgs::LaserScanPtr new_scan);

  bool broadcastTf() const noexcept;
  void broadcastTf(const bool broadcast) noexcept;

  void process();

protected:

  bool configured_   = false;
  bool broadcast_tf_ = true;
  bool init_origin_  = false;
  bool new_scan_     = false;
  bool publish_odom_ = false;

  ros::NodeHandle private_nh_;

  sensor_msgs::LaserScanPtr latest_scan_;

  LaserOdometryPtr laser_odom_ptr_;

  tf::TransformBroadcaster tf_broadcaster_;

  ros::Subscriber laser_sub_;
  ros::Publisher  pub_;

  void sendTransform();
  void publish(const nav_msgs::OdometryPtr odom_ptr);
  void publish(const geometry_msgs::Pose2DPtr odom_ptr);
};

} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_CORE_LASER_ODOMETRY_NODE_H_ */