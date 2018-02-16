#ifndef _LASER_ODOMETRY_CORE_LASER_ODOMETRY_NODE_H_
#define _LASER_ODOMETRY_CORE_LASER_ODOMETRY_NODE_H_

#include <laser_odometry_core/laser_odometry_core.h>

#include <topic_tools/shape_shifter.h>
#include <tf2_ros/transform_broadcaster.h>

namespace laser_odometry
{

namespace detail
{

template <typename E>
constexpr typename std::underlying_type<E>::type get_underlying(const E& e) noexcept
{
  return static_cast<typename std::underlying_type<E>::type>(e);
}

template<class Enum, class T>
Enum to_enum(const T& e) { return static_cast<Enum>(e); }

} // namespace detail

/**
 * @brief The LaserOdometryNode class.
 * It basically holds the listeners/publishers etc
 * for the laser_odometry plugin hold in
 * laser_odometry::LaserOdometryPtr.
 */
class LaserOdometryNode
{
public:

  /// @brief Default constructor.
  LaserOdometryNode();

  /// @brief Default destructor.
  virtual ~LaserOdometryNode() = default;

  /**
   * @brief Set the laser pose wrt the robot
   * base_frame frame by retrieving it from tf.
   * @param[in] t. The ros::Time at which to query tf for the transform.
   * @param[in] d. The ros::Duration during which tf is queried.
   */
  void setLaserFromTf(const ros::Time& t = ros::Time(0),
                      const ros::Duration& d = ros::Duration(1));

  /// @brief The main process.
  ///
  /// It can be summarized as follows:
  ///
  ///       - if new reading available
  ///         - plugin process new reading
  ///         - if plugin has new referent frame
  ///           - publish referent frame
  ///         - publish odometry
  ///         - if broadcast on tf
  ///           - broadcast on tf
  ///
  void process();

protected:

  bool configured_   = false; /*!< @brief Whether the node is configured. */
  bool fixed_sensor_ = true;  /*!< @brief Whether the sensor is fixed wrt to the robot base_frame. */
  bool broadcast_tf_ = false; /*!< @brief Whether the node broadcast the pose on tf. */
  bool init_origin_  = false; /*!< @brief Whether to initialize the origin from tf. */
  bool new_scan_     = false; /*!< @brief Whether a new LaserScan was received. */
  bool new_cloud_    = false; /*!< @brief Whether a new PointCloud2 was received. */

  /// @brief Whether to publish a nav_msgs::Odometry msg
  /// or a geometry_msgs::Pose2D msg
  bool publish_odom_ = true;

  /// @brief the message throttling ratio.
  int throttle_ = 1;

  /// @brief How many tries to retrieve the laser pose
  int tf_try_ = 1;

  /// @brief The global frame from which the origin
  /// can base initialized.
  std::string global_frame_;

  /// @brief A pointer to the laser_odometry plugin
  LaserOdometryPtr laser_odom_ptr_;

  /// \brief Current ros::Time accordingly
  /// to the last received message.
  std_msgs::Header::_stamp_type    current_stamp_;

  /// \brief The last received LaserScan
  sensor_msgs::LaserScanConstPtr   latest_scan_;

  /// \brief The last received PointCloud2
  sensor_msgs::PointCloud2ConstPtr latest_cloud_;

  ros::NodeHandle private_nh_;

  /// \brief The \c tf broadcaster
  boost::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ptr_;

  /// @brief Unique subscriber of the node.
  /// @see resetListenerWithType
  ros::Subscriber sub_;

  /// @brief The odometry publisher.
  /// It publishes either a nav_msgs::Odometry msg
  /// or a geometry_msgs::Pose2D msg depending on
  /// publish_odom_ parameter.
  ros::Publisher  pub_odom_;

  /// @brief The delta odometry publisher.
  /// It publishes either a nav_msgs::Odometry msg
  /// or a geometry_msgs::Pose2D msg depending on
  /// publish_odom_ parameter.
  ros::Publisher  pub_odom_inc_;

  /// @brief The Referent reading publisher.
  /// Everytime the referent reading is updated,
  /// it publishes it.
  ros::Publisher  pub_kframe_;

  /// @brief Initializes the node.
  void initialize();

  /// @brief The LaserScan callback.
  /// @see sub_
  /// @see resetListenerWithType
  void LaserCallback(const sensor_msgs::LaserScanConstPtr&   new_scan);

  /// @brief The PointCloud2 callback.
  /// @see sub_
  /// @see resetListenerWithType
  void CloudCallback(const sensor_msgs::PointCloud2ConstPtr& new_cloud);

  /**
   * @brief Whether the node broadcast on tf.
   * @return \c true if broadcast, \c false otherwise.
   */
  bool broadcastTf() const noexcept;

  /**
   * @brief Whether the node broadcast on tf.
   * @param[in] \c true if broadcast, \c false otherwise.
   */
  void broadcastTf(const bool broadcast) noexcept;

  /**
   * @brief First message callback.
   *
   * When receiving the first message it determines
   * if the topic message is of type
   * LaserScan or PointCloud2 and set the proper
   * callback and process scheme accordingly.
   * @param[in] new_s. The first received message.
   */
  void resetListenerWithType(const topic_tools::ShapeShifter::Ptr &new_s);

  /// @brief Broadcast the estimated pose on tf.
  void sendTransform();

  /// @brief Call the appropriate publisher publish method
  /// based on the message type.
  template <typename T>
  void publish(const T& msg) const;

  /// @brief On new-keyframe, publish increment
  /// since last key-frame in sensor frame.
  template <typename T>
  void publish_inc(const T& msg) const;
};

template <typename T>
void LaserOdometryNode::publish(const T& msg) const
{
  if (pub_odom_.getNumSubscribers() > 0)
    pub_odom_.publish(msg);
}

template <typename T>
void LaserOdometryNode::publish_inc(const T& msg) const
{
  if (pub_odom_inc_.getNumSubscribers() > 0 &&
      laser_odom_ptr_->hasNewKeyFrame())
  {
    pub_odom_inc_.publish(msg);
  }
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
