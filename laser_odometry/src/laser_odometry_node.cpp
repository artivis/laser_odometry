#include <laser_odometry/laser_odometry_node.h>
#include <laser_odometry_core/laser_odometry.h>
#include <laser_odometry_core/laser_odometry_utils.h>

namespace sm = sensor_msgs;

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
    return;
  }

  private_nh_.param("broadcast_tf", broadcast_tf_, broadcast_tf_);
  private_nh_.param("init_origin",  init_origin_,  init_origin_);
  private_nh_.param("publish_odom", publish_odom_, publish_odom_);

  laser_odom_ptr_ = LaserOdometryInstantiater::instantiate(laser_odometry_type);

  if (laser_odom_ptr_ != nullptr)
    configured_ = true;
  else
  {
    ROS_ERROR("Something went wrong, exiting.");
    throw std::runtime_error("Something went wrong.");
  }

  tf::Transform base_to_laser = tf::Transform::getIdentity();
  utils::getTf(laser_odom_ptr_->getFrameLaser(),
               laser_odom_ptr_->getFrameBase(),
               base_to_laser);

  laser_odom_ptr_->setLaserPose(base_to_laser);

  if (init_origin_)
  {
    tf::Transform origin_to_base = tf::Transform::getIdentity();
    utils::getTf(laser_odom_ptr_->getFrameFixed(),
                 laser_odom_ptr_->getFrameBase(),
                 origin_to_base);

    laser_odom_ptr_->setOrigin(origin_to_base);
  }

  laser_sub_ = private_nh_.subscribe("scan_in", 1,
                                     &LaserOdometryNode::resetListenerWithType, this);

  ROS_INFO("Subscribed to %s", laser_sub_.getTopic().c_str());

  if (publish_odom_)
    pub_ = private_nh_.advertise<nav_msgs::Odometry>("laser_odom", 1);
  else
    pub_ = private_nh_.advertise<geometry_msgs::Pose2D>("laser_odom", 1);
}

void LaserOdometryNode::LaserCallback(sensor_msgs::LaserScanPtr new_scan)
{
  latest_scan_ = new_scan;
  new_scan_ = true;
}

void LaserOdometryNode::CloudCallback(const sensor_msgs::PointCloud2ConstPtr new_cloud)
{
  latest_cloud_ = new_cloud;
  new_cloud_ = true;

  ROS_WARN_STREAM("Header " << new_cloud->header);
}

void LaserOdometryNode::process()
{
  if (!new_scan_ || !new_cloud_ || !configured_) return;

  if (publish_odom_)
  {
    nav_msgs::OdometryPtr odom_ptr = boost::make_shared<nav_msgs::Odometry>();
    //nav_msgs::OdometryPtr relative_odom_ptr;

    if (new_scan_)
    {
      new_scan_ = false;
      laser_odom_ptr_->process(latest_scan_, odom_ptr);
    }
    else if (new_cloud_)
    {
      new_cloud_ = false;
      laser_odom_ptr_->process(latest_cloud_, odom_ptr);
    }

    publish(odom_ptr);

    ROS_INFO_STREAM("Estimated odom\n" << odom_ptr->pose.pose);
  }
  else
  {
    geometry_msgs::Pose2DPtr pose_2d_ptr = boost::make_shared<geometry_msgs::Pose2D>();
    //nav_msgs::OdometryPtr relative_odom_ptr;

    if (new_scan_)
    {
      new_scan_ = false;
      laser_odom_ptr_->process(latest_scan_, pose_2d_ptr);
    }
    else if (new_cloud_)
    {
      new_cloud_ = false;
      laser_odom_ptr_->process(latest_cloud_, pose_2d_ptr);
    }

    publish(pose_2d_ptr);

    ROS_INFO_STREAM("Estimated pose_2d\n" << *pose_2d_ptr);
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
  laser_sub_.shutdown();

  if (new_s->getDataType() == "sensor_msgs/LaserScan") {
    laser_sub_ = private_nh_.subscribe("scan_in", 1,
                                      &LaserOdometryNode::LaserCallback, this);
  }
  else if(new_s->getDataType() == "sensor_msgs/PointCloud2") {
    laser_sub_ = private_nh_.subscribe("scan_in", 1,
                                      &LaserOdometryNode::CloudCallback, this);
  }
  else {
    ROS_ERROR("Subscribed to topic of unknown type !");
  }
}

void LaserOdometryNode::sendTransform()
{
  if (broadcast_tf_ && configured_)
  {
    tf::StampedTransform transform_msg(laser_odom_ptr_->getEstimatedPose(),
                                       laser_odom_ptr_->getCurrentTime(),
                                       laser_odom_ptr_->getFrameOdom(),
                                       laser_odom_ptr_->getFrameBase());

    geometry_msgs::TransformStamped ttf;
    tf::transformStampedTFToMsg(transform_msg, ttf);

    ROS_INFO_STREAM("Sending tf:\n" << ttf);

    tf_broadcaster_.sendTransform(transform_msg);
  }
}

void LaserOdometryNode::publish(const nav_msgs::OdometryPtr odom_ptr)
{
  pub_.publish(odom_ptr);
}

void LaserOdometryNode::publish(const geometry_msgs::Pose2DPtr pose_2d_ptr)
{
  pub_.publish(pose_2d_ptr);
}

} /* namespace laser_odometry */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_odometry_node");

  laser_odometry::LaserOdometryNode node;

  ros::Rate rate(40);

  while (ros::ok())
  {
    node.process();

    ros::spinOnce();

    rate.sleep();
  }

  return EXIT_SUCCESS;
}
