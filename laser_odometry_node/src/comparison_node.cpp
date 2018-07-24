/*
  @file

  @author danielecillis

  @copyright (c) 2018 PAL Robotics SL. All Rights Reserved
*/
#include "laser_odometry_node/comparison_node.h"
#include <laser_odometry_core/laser_odometry_instantiater.h>
#include <laser_odometry_core/laser_odometry_utils.h>

#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf/tf.h>

#include <math.h>
#include <laser_odometry_node/RMSE.h>
#include <laser_odometry_node/RMSEs.h>

namespace sm = sensor_msgs;

namespace
{
bool getTf(const std::string& source_frame, const std::string& target_frame,
           laser_odometry::Transform& tf, const ros::Time& t = ros::Time(0),
           const ros::Duration& d = ros::Duration(0.5))
{
  tf2_ros::Buffer tf2_buffer;
  tf2_ros::TransformListener tf2_listener(tf2_buffer);

  geometry_msgs::TransformStamped transform_stamped;
  try
  {
    transform_stamped = tf2_buffer.lookupTransform(target_frame, source_frame, t, d);
  }
  catch (const tf2::TransformException& ex)
  {
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
comparisonNode::comparisonNode() : private_nh_("~")
{
  initialize();
}

void comparisonNode::initialize()
{
  private_nh_.param("broadcast_tf", broadcast_tf_, broadcast_tf_);
  private_nh_.param("init_origin", init_origin_, init_origin_);
  private_nh_.param("publish_odom", publish_odom_, publish_odom_);
  private_nh_.param("fixed_sensor", fixed_sensor_, fixed_sensor_);
  private_nh_.param("throttle", throttle_, throttle_);
  private_nh_.param("tf_try", tf_try_, tf_try_);
  private_nh_.param("global_frame", global_frame_, std::string("map"));
  double timeout;
  private_nh_.param<double>("timeout", timeout, 1.0);
  timeout_ = ros::Duration(timeout);
  private_nh_.param<std::string>("ground_truth_topic", ground_truth_topic_, "/ground_truth_pose");
  ground_truth_sub_ =
      private_nh_.subscribe(ground_truth_topic_, 1, &comparisonNode::updateGroundTruth, this);
  std::string string;
  private_nh_.param<std::string>("plugins", string, "LibPointMatcher,Polar,Rf2o,Csm");
  boost::split(names_, string, boost::is_any_of(","));
  std::string prefix = "laser_odometry::LaserOdometry";

  for (std::string s : names_)
    laser_odom_vec_ptr_.push_back(make_laser_odometry(prefix + s));

  for (unsigned int i = 0; i < laser_odom_vec_ptr_.size(); ++i)
  {
    if (laser_odom_vec_ptr_[i] != nullptr)
      configured_ = true;
    else
    {
      ROS_ERROR("Something went wrong.");
      throw std::runtime_error("Something went wrong.");
    }
  }

  setLaserFromTf(ros::Time::now());

  if (init_origin_)
  {
    for (unsigned int i = 0; i < laser_odom_vec_ptr_.size(); ++i)
    {
      Transform tf_origin_to_base = Transform::Identity();
      getTf(laser_odom_vec_ptr_[i]->getFrameBase(), global_frame_, tf_origin_to_base);

      laser_odom_vec_ptr_[i]->setOrigin(tf_origin_to_base);

      ROS_INFO_STREAM("Initializing origin:\n" << tf_origin_to_base.matrix());
    }
  }

  sub_ = private_nh_.subscribe("topic_in", 1, &comparisonNode::resetListenerWithType, this);

  ROS_INFO("Subscribed to %s", sub_.getTopic().c_str());

  std::vector<Error> empty_vector;
  for (unsigned int i = 0; i < laser_odom_vec_ptr_.size(); ++i)
  {
    if (publish_odom_)
    {
      pub_odom_vec_.push_back(
          private_nh_.advertise<nav_msgs::Odometry>("laser_odom_" + names_[i], 1));
      pub_odom_inc_vec_.push_back(
          private_nh_.advertise<nav_msgs::Odometry>("laser_delta_odom_" + names_[i], 1));
    }
    else
    {
      pub_odom_vec_.push_back(
          private_nh_.advertise<geometry_msgs::Pose2D>("laser_odom_" + names_[i], 1));
      pub_odom_inc_vec_.push_back(
          private_nh_.advertise<geometry_msgs::Pose2D>("laser_delta_odom_" + names_[i], 1));
    }
    new_origins_.push_back(ground_truth_pose_.pose);
    errors_.push_back(empty_vector);
  }
  new_origins_.push_back(ground_truth_pose_.pose);

  pub_rmse_ = private_nh_.advertise<laser_odometry_node::RMSEs>("rmse", 1);
}

void comparisonNode::LaserCallback(const sensor_msgs::LaserScanConstPtr& new_scan)
{
  if (new_scan->header.seq % throttle_ != 0)
    return;

  latest_scan_ = new_scan;
  current_stamp_ = latest_scan_->header.stamp;
  new_scan_ = true;
}

void comparisonNode::CloudCallback(const sensor_msgs::PointCloud2ConstPtr& new_cloud)
{
  if (new_cloud->header.seq % throttle_ != 0)
    return;

  latest_cloud_ = new_cloud;
  current_stamp_ = latest_cloud_->header.stamp;
  new_cloud_ = true;
}

void comparisonNode::setLaserFromTf(const ros::Time& t, const ros::Duration& d)
{
  for (unsigned int i = 0; i < laser_odom_vec_ptr_.size(); ++i)
  {
    Transform tf_base_to_laser = laser_odom_vec_ptr_[i]->getLaserPose();

    bool got_tf = false;

    while (tf_try_ && !got_tf)
    {
      got_tf = getTf(laser_odom_vec_ptr_[i]->getFrameLaser(),
                     laser_odom_vec_ptr_[i]->getFrameBase(), tf_base_to_laser, t, d);
      --tf_try_;
    }

    laser_odom_vec_ptr_[i]->setLaserPose(tf_base_to_laser);

    ROS_INFO_STREAM("Setting laser to base :\n" << tf_base_to_laser.matrix());
  }
}

void comparisonNode::process()
{
  if (!(new_scan_ || new_cloud_) || !configured_)
    return;

  if (!fixed_sensor_)
    setLaserFromTf(current_stamp_, ros::Duration(0.005));

  if (publish_odom_)
  {
    std::vector<nav_msgs::OdometryPtr> odom_vec_ptr;
    std::vector<nav_msgs::OdometryPtr> odom_inc_vec_ptr;

    for (unsigned int i = 0; i < laser_odom_vec_ptr_.size(); ++i)
    {
      odom_vec_ptr.push_back(boost::make_shared<nav_msgs::Odometry>());
      odom_inc_vec_ptr.push_back(boost::make_shared<nav_msgs::Odometry>());
      if (new_scan_)
      {
        if (i == static_cast<int>(laser_odom_vec_ptr_.size())-1)
          new_scan_ = false;
        laser_odom_vec_ptr_[i]->process(latest_scan_, odom_vec_ptr[i], odom_inc_vec_ptr[i]);

        /// @todo fixme
        // if (laser_odom_vec_ptr_[i]->hasNewKeyFrame())
        error(odom_vec_ptr[i], i);
        // else
        //    error(odom_vec_ptr[i], i);

        if (laser_odom_vec_ptr_[i]->hasNewKeyFrame())
        {
          sensor_msgs::LaserScanConstPtr kframe;
          laser_odom_vec_ptr_[i]->getKeyFrame(kframe);
          publish(kframe, i);
        }
      }
      else if (new_cloud_)
      {
        if (i == static_cast<int>(laser_odom_vec_ptr_.size())-1)
          new_cloud_ = false;
        laser_odom_vec_ptr_[i]->process(latest_cloud_, odom_vec_ptr[i], odom_inc_vec_ptr[i]);

        if (laser_odom_vec_ptr_[i]->hasNewKeyFrame())
        {
          sensor_msgs::PointCloud2ConstPtr kframe;
          laser_odom_vec_ptr_[i]->getKeyFrame(kframe);
          publish(kframe, i);
        }
      }
    }

    publish(odom_vec_ptr);

    publish_inc(odom_inc_vec_ptr);

    for (unsigned int i = 0; i < odom_inc_vec_ptr.size(); ++i)
    {
      ROS_DEBUG_STREAM("Estimated odom_" + names_[i] + "\n"
                       << odom_inc_vec_ptr[i]->pose.pose);
    }
    if (shouldUpdate(ros::Time(odom_vec_ptr[0]->header.stamp.sec,
                               odom_vec_ptr[0]->header.stamp.nsec)))
      setNewOrigin(odom_vec_ptr);
  }
  else
  {
    std::vector<geometry_msgs::Pose2DPtr> pose_2d_vec_ptr;
    std::vector<geometry_msgs::Pose2DPtr> pose_2d_inc_vec_ptr;

    for (unsigned int i = 0; i < laser_odom_vec_ptr_.size(); ++i)
    {
      pose_2d_vec_ptr.push_back(boost::make_shared<geometry_msgs::Pose2D>());
      pose_2d_inc_vec_ptr.push_back(boost::make_shared<geometry_msgs::Pose2D>());
      if (new_scan_)
      {
        new_scan_ = false;
        laser_odom_vec_ptr_[i]->process(latest_scan_, pose_2d_vec_ptr[i],
                                        pose_2d_inc_vec_ptr[i]);
      }
      else if (new_cloud_)
      {
        new_cloud_ = false;
        laser_odom_vec_ptr_[i]->process(latest_scan_, pose_2d_vec_ptr[i],
                                        pose_2d_inc_vec_ptr[i]);
      }
    }

    publish(pose_2d_vec_ptr);

    publish_inc(pose_2d_inc_vec_ptr);

    for (unsigned int i = 0; i < laser_odom_vec_ptr_.size(); ++i)
      ROS_DEBUG_STREAM("Estimated pose_2d_" + names_[i] + "\n" << *pose_2d_vec_ptr[i]);
  }
}

void comparisonNode::resetListenerWithType(const topic_tools::ShapeShifter::Ptr& new_s)
{
  sub_.shutdown();

  for (unsigned int i = 0; i < laser_odom_vec_ptr_.size(); ++i)
  {
    if (new_s->getDataType() == "sensor_msgs/LaserScan")
    {
      sub_ = private_nh_.subscribe("topic_in", 1, &comparisonNode::LaserCallback, this);

      pub_kframe_vec_.push_back(
          private_nh_.advertise<sensor_msgs::LaserScan>("key_frame_" + names_[i], 1));
    }
    else if (new_s->getDataType() == "sensor_msgs/PointCloud2")
    {
      sub_ = private_nh_.subscribe("topic_in", 1, &comparisonNode::CloudCallback, this);

      pub_kframe_vec_.push_back(
          private_nh_.advertise<sensor_msgs::PointCloud2>("key_frame_" + names_[i], 1));
    }
    else
    {
      ROS_ERROR("Subscribed to topic of type %s !", new_s->getDataType().c_str());
    }
  }
}

template <typename Vector3Type>
double comparisonNode::pointDistance(const Vector3Type& a, const Vector3Type& b)
{
  return (Eigen::Vector3d(a.x, a.y, a.z) - Eigen::Vector3d(b.x, b.y, b.z)).norm();
}

double comparisonNode::quaternionDistance(const geometry_msgs::Quaternion& a,
                                          const geometry_msgs::Quaternion& b)
{
  Eigen::Quaterniond expected_e, actual_e;
  expected_e.x() = a.x;
  expected_e.y() = a.y;
  expected_e.z() = a.z;
  expected_e.w() = a.w;

  actual_e.x() = b.x;
  actual_e.y() = b.y;
  actual_e.z() = b.z;
  actual_e.w() = b.w;
  return expected_e.angularDistance(actual_e);
}

void comparisonNode::error(nav_msgs::OdometryPtr odom_ptr, int i)
{
  std::vector<geometry_msgs::Pose> poses{ odom_ptr->pose.pose, ground_truth_pose_.pose };
  std::vector<tf::Transform> tfs{ tf::Transform(), tf::Transform(), tf::Transform() };
  std::vector<int> vec{ i, static_cast<int>(new_origins_.size() - 1) };

  //  ROS_ERROR_STREAM("Ground_truth  " << ros::Time(ground_truth_pose_.header.stamp.sec,
  //  ground_truth_pose_.header.stamp.nsec)); ROS_ERROR_STREAM(names_[i] + "  " <<
  //  ros::Time(odom_ptr->header.stamp.sec, odom_ptr->header.stamp.nsec));

  for (int j = 0; j < 2; ++j)
  {
    tf::poseMsgToTF(poses[j], tfs[1]);
    tf::poseMsgToTF(new_origins_[vec[j]], tfs[0]);
    tfs[2].mult(tfs[0].inverse(), tfs[1]);
    tf::poseTFToMsg(tfs[2], poses[j]);
    tfs = { tf::Transform(), tf::Transform(), tf::Transform() };
  }
  Error e;
  e.translational_error = pointDistance(poses[0].position, poses[1].position);
  e.rotational_error = quaternionDistance(poses[0].orientation, poses[1].orientation);

  errors_[i].push_back(e);
}

void comparisonNode::rmse()
{
  std::string s = "RMSE\n";
  double sum1 = 0.0;
  double sum2 = 0.0;
  laser_odometry_node::RMSE rmse_msg;
  laser_odometry_node::RMSEs rmse_list;

  std::map<std::string, std::string> spaces;
  spaces["LibPointMatcher"] = "";
  spaces["Polar"] = "          ";
  spaces["Rf2o"] = "           ";
  spaces["Csm"] = "            ";

  for (unsigned int i = 0; i < errors_.size(); ++i)
  {
    for (unsigned int j = 0; j < errors_[i].size(); ++j)
    {
      sum1 += pow(errors_[i][j].translational_error, 2);
      sum2 += pow(errors_[i][j].rotational_error, 2);
    }
    sum1 = sqrt(sum1 / errors_[i].size());
    sum2 = sqrt(sum2 / errors_[i].size());
    s += names_[i] + spaces.at(names_[i]) + ": " + std::to_string((sum1)) + " " +
         std::to_string(sum2) + "\n";
    rmse_msg.transl_error = sum1;
    rmse_msg.rot_error = sum2;
    rmse_msg.name = names_[i];
    rmse_list.list.push_back(rmse_msg);
    errors_[i].clear();
    sum1 = 0.0;
    sum2 = 0.0;
  }
  ROS_WARN_STREAM(s);
  pub_rmse_.publish(rmse_list);
}

void comparisonNode::setNewOrigin(std::vector<nav_msgs::OdometryPtr> odom_vec_ptr)
{
  last_update_ =
      ros::Time(odom_vec_ptr[0]->header.stamp.sec, odom_vec_ptr[0]->header.stamp.nsec);

  for (unsigned int i = 0; i < new_origins_.size() - 1; ++i)
    new_origins_[i] = odom_vec_ptr[i]->pose.pose;

  new_origins_.back() = ground_truth_pose_.pose;

  rmse();
}

void comparisonNode::updateGroundTruth(const geometry_msgs::PoseStampedPtr& p)
{
  ground_truth_pose_ = *p;
}

bool comparisonNode::shouldUpdate(ros::Time t)
{
  if (t - last_update_ >= timeout_)
    return true;
  else
    return false;
}

}  // namespace laser_odometry

int main(int argc, char** argv)
{
  ros::init(argc, argv, "comparison_node");

  laser_odometry::comparisonNode node;

  ros::Rate rate(100);

  while (ros::ok())
  {
    const auto start = ros::Time::now();

    ros::spinOnce();

    node.process();

    ROS_DEBUG_STREAM("Processing took : " << (ros::Time::now() - start));

    rate.sleep();
  }
  return 0;
}

//  ROS_WARN_STREAM("P1^p0 " << names_[i] << ":\n" << pose.position.x << " " <<
//  pose.position.y << " "
//                            << pose.position.z << "\n" << pose.orientation.x << " "
//                            << pose.orientation.y << " " << pose.orientation.z << " "
//                            << pose.orientation.w);
