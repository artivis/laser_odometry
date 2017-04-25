#ifndef _LASER_ODOMETRY_CORE_LASER_ODOMETRY_NODE_H_
#define _LASER_ODOMETRY_CORE_LASER_ODOMETRY_NODE_H_

#include <laser_odometry_core/laser_odometry_core.h>

#include <topic_tools/shape_shifter.h>

namespace laser_odometry
{

class LaserOdometryNode
{
public:

  LaserOdometryNode();
  virtual ~LaserOdometryNode() = default;

  void setLaserFromTf(const ros::Time& t = ros::Time(0),
                      const ros::Duration& d = ros::Duration(1));

  void process();

protected:

  bool configured_   = false;
  bool fixed_sensor_ = true;
  bool broadcast_tf_ = true;
  bool init_origin_  = false;
  bool new_scan_     = false;
  bool new_cloud_    = false;
  bool publish_odom_ = false;

  LaserOdometryPtr laser_odom_ptr_;

  std_msgs::Header::_stamp_type    current_stamp_;
  sensor_msgs::LaserScanConstPtr   latest_scan_;
  sensor_msgs::PointCloud2ConstPtr latest_cloud_;

  ros::NodeHandle private_nh_;

  tf::TransformBroadcaster tf_broadcaster_;

  ros::Subscriber sub_;
  ros::Publisher  pub_;

  void initialize();

  void LaserCallback(const sensor_msgs::LaserScanConstPtr&   new_scan);
  void CloudCallback(const sensor_msgs::PointCloud2ConstPtr& new_cloud);

  bool broadcastTf() const noexcept;
  void broadcastTf(const bool broadcast) noexcept;

  void resetListenerWithType(const topic_tools::ShapeShifter::Ptr &new_s);

  void sendTransform();
  void publish(const nav_msgs::OdometryPtr odom_ptr) const;
  void publish(const geometry_msgs::Pose2DPtr odom_ptr) const;
};

} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_CORE_LASER_ODOMETRY_NODE_H_ */
