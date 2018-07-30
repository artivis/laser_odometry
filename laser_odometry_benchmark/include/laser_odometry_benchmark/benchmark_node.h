/*
  @file

  @author danielecillis

  @copyright (c) 2018 PAL Robotics SL. All Rights Reserved
*/
#ifndef BENCHMARK_NODE_H
#define BENCHMARK_NODE_H

#include <laser_odometry_core/laser_odometry_core.h>

// The input ROS messages supported
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>

#include <topic_tools/shape_shifter.h>
#include <tf2_ros/transform_broadcaster.h>

// More ROS header
#include <ros/node_handle.h>

struct Error
{
  double translational_error;
  double rotational_error;
};

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

}

class BenchmarkNode
{
public:
  /// @brief Default constructor.
  BenchmarkNode();

  /// @brief Default destructor.
  virtual ~BenchmarkNode() = default;

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

  /// @brief Calculates the error from the last registered pose by comparing in with
  /// the model state pose of the robot
  void error(nav_msgs::OdometryPtr odom_ptr, int i);

  /// @brief Computes the RMSE using the errors collected until the moment the function is
  /// called, and then resets errors_
  void rmse();

protected:
  bool configured_ = false; /*!< @brief Whether the node is configured. */
  bool fixed_sensor_ =
      true; /*!< @brief Whether the sensor is fixed wrt to the robot base_frame. */
  bool broadcast_tf_ = false; /*!< @brief Whether the node broadcast the pose on tf. */
  bool init_origin_ = false;  /*!< @brief Whether to initialize the origin from tf. */
  bool new_scan_ = false;     /*!< @brief Whether a new LaserScan was received. */
  bool new_cloud_ = false;    /*!< @brief Whether a new PointCloud2 was received. */

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

  /// @brief A vector of pointers to the laser_odometry plugins
  /// 0: laser_odometry::LaserOdometryLibPointMatcher
  /// 1: laser_odometry::LaserOdometryPolar
  /// 2: laser_odometry::LaserOdometryCsm
  /// 3: laser_odometry::LaserOdometryRf2o
  std::vector<LaserOdometryPtr> laser_odom_vec_ptr_;

  /// @brief A vector for containing errors with respect to the ground truth
  std::vector<std::vector<Error>> errors_;

  ///@brief A vector that contains the last poses where the odometry plugins
  /// were evaluated. The idea is to calculate the error with respect to the increment
  /// from the last saved position
  std::vector<geometry_msgs::Pose> new_origins_;

  ///@brief A vector of names of the laser_odometry plugins
  std::vector<std::string> names_;

  /// \brief Current ros::Time accordingly
  /// to the last received message.
  std_msgs::Header::_stamp_type current_stamp_;

  /// \brief The last received LaserScan
  sensor_msgs::LaserScanConstPtr latest_scan_;

  /// \brief The last received PointCloud2
  sensor_msgs::PointCloud2ConstPtr latest_cloud_;

  ros::NodeHandle private_nh_;

  /// @brief Unique subscriber of the node.
  /// @see resetListenerWithType
  ros::Subscriber sub_;

  /// @brief The odometry publishers.
  /// They publish either a nav_msgs::Odometry msg
  /// or a geometry_msgs::Pose2D msg depending on
  /// publish_odom_ parameter.
  std::vector<ros::Publisher> pub_odom_vec_;

  /// @brief The delta odometry publishers.
  /// They publish either a nav_msgs::Odometry msg
  /// or a geometry_msgs::Pose2D msg depending on
  /// publish_odom_ parameter.
  std::vector<ros::Publisher> pub_odom_inc_vec_;

  /// @brief The Referent reading publishers.
  /// Everytime the referent readings are updated,
  /// they publish them.
  std::vector<ros::Publisher> pub_kframe_vec_;

  /// @brief The RMSE publisher, which publishes RMSE for all the plugins
  /// at the end of every timeout_
  ros::Publisher pub_rmse_;

  /// @brief The last time the pose in new_origins_ have been calculated. Needed
  /// to calculate when timeout_ is over
  ros::Time last_update_;

  /// @brief Parameter that indicates after how much time RMSE is calculated
  ros::Duration timeout_;

  /// @brief Name of the topic which publishes the ground truth pose
  std::string ground_truth_topic_;

  /// @brief Subscriber to the ground truth pose
  ros::Subscriber ground_truth_sub_;

  /// @brief Ground truth pose, up to date thanks to updateGroundTruth()
  geometry_msgs::PoseStamped ground_truth_pose_;

  /// @brief Initializes the node.
  void initialize();

  /// @brief The LaserScan callback.
  /// @see sub_
  /// @see resetListenerWithType
  void LaserCallback(const sensor_msgs::LaserScanConstPtr& new_scan);

  /// @brief The PointCloud2 callback.
  /// @see sub_
  /// @see resetListenerWithType
  void CloudCallback(const sensor_msgs::PointCloud2ConstPtr& new_cloud);

  /**
   * @brief First message callback.
   *
   * When receiving the first message it determines
   * if the topic message is of type
   * LaserScan or PointCloud2 and set the proper
   * callback and process scheme accordingly.
   * @param[in] new_s. The first received message.
   */
  void resetListenerWithType(const topic_tools::ShapeShifter::Ptr& new_s);

  /// @brief When timeout_ is over, this function saves the estimated odometry
  /// poses and calls "rmse()" to calculate and publish the error
  void setNewOrigin(std::vector<nav_msgs::OdometryPtr> odom_vec_ptr);

  /// @brief Updates the ground_truth_pose_ variable
  void updateGroundTruth(const geometry_msgs::PoseStampedPtr &p);

  /// @brief Checks if timeout_ is over
  bool shouldUpdate(ros::Time t);

  ///@brief Calculates the distance between two quaternions
  double quaternionDistance(const geometry_msgs::Quaternion &a, const geometry_msgs::Quaternion &b);

  ///@brief Calculates the distance between two points
  template <typename Vector3Type>
  double pointDistance(const Vector3Type &a, const Vector3Type &b);

  /// @brief Call the appropriate publisher publish method
  /// based on the message type.
  template <typename T>
  void publish(std::vector<T> msgs) const;

  template <typename T>
  void publish(const T& msg, int i) const;

  /// @brief On new-keyframe, publish increment
  /// since last key-frame in sensor frame.
  template <typename T>
  void publish_inc(std::vector<T> msgs) const;
};

template <typename T>
void BenchmarkNode::publish(std::vector<T> msgs) const
{
  for (unsigned int i = 0; i < pub_odom_vec_.size(); ++i)
  {
    if (pub_odom_vec_[i].getNumSubscribers() > 0)
      pub_odom_vec_[i].publish(msgs[i]);
  }
}

template <typename T>
void BenchmarkNode::publish_inc(std::vector<T> msgs) const
{
  for (unsigned int i = 0; i < pub_odom_inc_vec_.size(); ++i)
  {
    if (pub_odom_inc_vec_[i].getNumSubscribers() > 0 && laser_odom_vec_ptr_[i]->hasNewKeyFrame())
    {
      pub_odom_inc_vec_[i].publish(msgs[i]);
    }
  }
}

template <typename T>
void BenchmarkNode::publish(const T& msg, int i) const
{
  if (msg == nullptr)
    return;

  if (pub_kframe_vec_[i].getNumSubscribers() > 0)
    pub_kframe_vec_[i].publish(msg);
}

} // namespace laser_odometry
#endif  // BENCHMARK_NODE_H
