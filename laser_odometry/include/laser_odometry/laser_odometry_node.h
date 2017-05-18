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
  bool broadcast_tf_ = false;
  bool init_origin_  = false;
  bool new_scan_     = false;
  bool new_cloud_    = false;
  bool publish_odom_ = true;

  int throttle_ = 1;

  LaserOdometryPtr laser_odom_ptr_;

  std_msgs::Header::_stamp_type    current_stamp_;
  sensor_msgs::LaserScanConstPtr   latest_scan_;
  sensor_msgs::PointCloud2ConstPtr latest_cloud_;

  ros::NodeHandle private_nh_;

  tf::TransformBroadcaster tf_broadcaster_;

  ros::Subscriber sub_;
  ros::Publisher  pub_odom_;
  ros::Publisher  pub_kframe_;

  void initialize();

  void LaserCallback(const sensor_msgs::LaserScanConstPtr&   new_scan);
  void CloudCallback(const sensor_msgs::PointCloud2ConstPtr& new_cloud);

  bool broadcastTf() const noexcept;
  void broadcastTf(const bool broadcast) noexcept;

  void resetListenerWithType(const topic_tools::ShapeShifter::Ptr &new_s);

  void sendTransform();

  template <typename T>
  void publish(const T& msg) const;
};

template <typename T>
void LaserOdometryNode::publish(const T& msg) const
{
  if (pub_odom_.getNumSubscribers() > 0)
    pub_odom_.publish(msg);
}

template <>
void LaserOdometryNode::publish(const sensor_msgs::LaserScanConstPtr& msg) const
{
  if (msg == nullptr) return;

  if (pub_kframe_.getNumSubscribers() > 0)
    pub_kframe_.publish(msg);
}

template <>
void LaserOdometryNode::publish(const sensor_msgs::PointCloud2ConstPtr& msg) const
{
  if (msg == nullptr) return;

  if (pub_kframe_.getNumSubscribers() > 0)
    pub_kframe_.publish(msg);
}

} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_CORE_LASER_ODOMETRY_NODE_H_ */
