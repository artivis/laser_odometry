#include <laser_odometry_node/laser_odometry_node.h>
#include <laser_odometry_core/laser_odometry_instantiater.h>
#include <laser_odometry_core/laser_odometry_utils.h>

#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

namespace sm = sensor_msgs;

namespace {

bool getTf(const std::string& source_frame,
           const std::string& target_frame,
           laser_odometry::Transform& tf,
           const ros::Time& t = ros::Time(0),
           const ros::Duration& d = ros::Duration(1.5))
{
  tf2_ros::Buffer tf2_buffer;
  tf2_ros::TransformListener tf2_listener(tf2_buffer);

  geometry_msgs::TransformStamped transform_stamped;
  try{
    transform_stamped = tf2_buffer.lookupTransform(target_frame, source_frame, t, d);
  }
  catch (const tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
    return false;
  }

  Eigen::Affine3d etf = tf2::transformToEigen(transform_stamped);
  tf.matrix() = etf.matrix();

  return true;
}

}

namespace laser_odometry
{

LaserOdometryNode::LaserOdometryNode() :
  private_nh_("~")
{
  initialize();
}

void LaserOdometryNode::initialize()
{
  std::string laser_odometry_type;
  if (!private_nh_.getParam("laser_odometry_type", laser_odometry_type))
  {
    ROS_ERROR("No laser odometry type specified !");
    throw std::runtime_error("No laser odometry type specified !");
  }

  private_nh_.param("broadcast_tf", broadcast_tf_, broadcast_tf_);
  private_nh_.param("init_origin",  init_origin_,  init_origin_);
  private_nh_.param("publish_odom", publish_odom_, publish_odom_);
  private_nh_.param("fixed_sensor", fixed_sensor_, fixed_sensor_);
  private_nh_.param("throttle",     throttle_,     throttle_);
  private_nh_.param("global_frame", global_frame_, std::string("map"));

  int inc_pub_opt = detail::get_underlying<IncrementPublishOptions>(publish_odom_inc_);
  private_nh_.param("publish_delta_option", inc_pub_opt, inc_pub_opt);
  publish_odom_inc_ = detail::to_enum<IncrementPublishOptions>(inc_pub_opt);

  laser_odom_ptr_ = make_laser_odometry(laser_odometry_type);

  if (laser_odom_ptr_ != nullptr)
    configured_ = true;
  else
  {
    ROS_ERROR("Something went wrong.");
    throw std::runtime_error("Something went wrong.");
  }

  setLaserFromTf(ros::Time::now());

  if (init_origin_)
  {
    Transform tf_origin_to_base = Transform::Identity();
    getTf(laser_odom_ptr_->getFrameBase(),
          global_frame_, tf_origin_to_base);

    laser_odom_ptr_->setOrigin(tf_origin_to_base);

    ROS_INFO_STREAM("Initializing origin :\n" << tf_origin_to_base.matrix());
  }

  sub_ = private_nh_.subscribe("topic_in", 1,
                               &LaserOdometryNode::resetListenerWithType, this);

  ROS_INFO("Subscribed to %s", sub_.getTopic().c_str());

  if (publish_odom_)
  {
    pub_odom_ = private_nh_.advertise<nav_msgs::Odometry>("laser_odom", 1);
    pub_odom_inc_  = private_nh_.advertise<nav_msgs::Odometry>("laser_delta_odom", 1);
  }
  else
  {
    pub_odom_ = private_nh_.advertise<geometry_msgs::Pose2D>("laser_odom", 1);
    pub_odom_inc_  = private_nh_.advertise<geometry_msgs::Pose2D>("laser_delta_odom", 1);
  }
}

void LaserOdometryNode::LaserCallback(const sensor_msgs::LaserScanConstPtr& new_scan)
{
  if (new_scan->header.seq % throttle_ != 0) return;

  latest_scan_   = new_scan;
  current_stamp_ = latest_scan_->header.stamp;
  new_scan_      = true;
}

void LaserOdometryNode::CloudCallback(const sensor_msgs::PointCloud2ConstPtr& new_cloud)
{
  if (new_cloud->header.seq % throttle_ != 0) return;

  latest_cloud_  = new_cloud;
  current_stamp_ = latest_cloud_->header.stamp;
  new_cloud_     = true;
}

void LaserOdometryNode::setLaserFromTf(const ros::Time &t, const ros::Duration &d)
{
  Transform tf_base_to_laser = Transform::Identity();

  getTf(laser_odom_ptr_->getFrameLaser(),
        laser_odom_ptr_->getFrameBase(),
        tf_base_to_laser, t, d);

  laser_odom_ptr_->setLaserPose(tf_base_to_laser);

  ROS_INFO_STREAM("Setting laser to base :\n" << tf_base_to_laser.matrix());
}

void LaserOdometryNode::process()
{
  if (!(new_scan_ || new_cloud_) || !configured_) return;

  if (!fixed_sensor_) setLaserFromTf(current_stamp_, ros::Duration(0.005));

  if (publish_odom_)
  {
    nav_msgs::OdometryPtr odom_ptr = boost::make_shared<nav_msgs::Odometry>();
    nav_msgs::OdometryPtr odom_inc_ptr = boost::make_shared<nav_msgs::Odometry>();

    if (new_scan_)
    {
      new_scan_ = false;
      laser_odom_ptr_->process(latest_scan_, odom_ptr, odom_inc_ptr);

      if (laser_odom_ptr_->hasNewKeyFrame())
      {
        sensor_msgs::LaserScanConstPtr kframe;
        laser_odom_ptr_->getKeyFrame(kframe);
        publish(kframe);
      }
    }
    else if (new_cloud_)
    {
      new_cloud_ = false;
      laser_odom_ptr_->process(latest_cloud_, odom_ptr, odom_inc_ptr);

      if (laser_odom_ptr_->hasNewKeyFrame())
      {
        sensor_msgs::PointCloud2ConstPtr kframe;
        laser_odom_ptr_->getKeyFrame(kframe);
        publish(kframe);
      }
    }

    publish(odom_ptr);

    publish_inc(odom_inc_ptr);

    ROS_DEBUG_STREAM("Estimated odom\n" << odom_ptr->pose.pose);
  }
  else
  {
    geometry_msgs::Pose2DPtr pose_2d_ptr = boost::make_shared<geometry_msgs::Pose2D>();
    geometry_msgs::Pose2DPtr pose_2d_inc_ptr = boost::make_shared<geometry_msgs::Pose2D>();

    if (new_scan_)
    {
      new_scan_ = false;
      laser_odom_ptr_->process(latest_scan_, pose_2d_ptr, pose_2d_inc_ptr);
    }
    else if (new_cloud_)
    {
      new_cloud_ = false;
      laser_odom_ptr_->process(latest_cloud_, pose_2d_ptr, pose_2d_inc_ptr);
    }

    publish(pose_2d_ptr);

    publish_inc(pose_2d_inc_ptr);

    ROS_DEBUG_STREAM("Estimated pose_2d\n" << *pose_2d_ptr);
  }

  if (broadcast_tf_) sendTransform();
}

bool LaserOdometryNode::broadcastTf() const noexcept
{
  return broadcast_tf_;
}

void LaserOdometryNode::broadcastTf(const bool broadcast) noexcept
{
  broadcast_tf_ = broadcast;
}

void LaserOdometryNode::resetListenerWithType(const topic_tools::ShapeShifter::Ptr& new_s)
{
  sub_.shutdown();

  if (new_s->getDataType() == "sensor_msgs/LaserScan") {
    sub_ = private_nh_.subscribe("topic_in", 1,
                                 &LaserOdometryNode::LaserCallback, this);

    pub_kframe_ = private_nh_.advertise<sensor_msgs::LaserScan>("key_frame", 1);
  }
  else if(new_s->getDataType() == "sensor_msgs/PointCloud2") {
    sub_ = private_nh_.subscribe("topic_in", 1,
                                 &LaserOdometryNode::CloudCallback, this);

    pub_kframe_ = private_nh_.advertise<sensor_msgs::PointCloud2>("key_frame", 1);
  }
  else {
    ROS_ERROR("Subscribed to topic of type %s !", new_s->getDataType().c_str());
  }
}

void LaserOdometryNode::sendTransform()
{
  if (broadcast_tf_ && configured_)
  {
    Eigen::Affine3d etf;
    etf.matrix() = laser_odom_ptr_->getEstimatedPose().matrix();

    geometry_msgs::TransformStamped tf_msg = tf2::eigenToTransform(etf);

    tf_msg.header.stamp = laser_odom_ptr_->getCurrentTime();
    tf_msg.header.frame_id = laser_odom_ptr_->getFrameOdom();
    tf_msg.child_frame_id  = laser_odom_ptr_->getFrameBase();

    ROS_DEBUG_STREAM("Sending tf:\n" << etf.matrix());

    tf_broadcaster_.sendTransform(tf_msg);
  }
}

} /* namespace laser_odometry */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_odometry_node");

  laser_odometry::LaserOdometryNode node;

  ros::Rate rate(40);

  while (ros::ok())
  {
    const auto start = ros::Time::now();

    ros::spinOnce();

    node.process();

    ROS_DEBUG_STREAM("Processing took : " << (ros::Time::now() - start));

    rate.sleep();
  }

  return EXIT_SUCCESS;
}
