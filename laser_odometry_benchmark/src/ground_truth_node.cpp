/*
  @file

  @author danielecillis

  @copyright (c) 2018 PAL Robotics SL. All Rights Reserved
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/GetModelState.h>


template <typename Service>
/**
 * @brief getModelState Retrieves the model state position of "name"
 * @param name Name of the Gazebo model
 * @param pose Pose of the Gazebo model
 * @param nh
 * @param srv ROS GetModelState Service
 * @param relative_entity_name
 * @return True if the pose is retrieved correctly
 */
bool getModelState(const std::string &name, geometry_msgs::PoseStamped &pose, ros::NodeHandle& nh,
                   Service &srv, const std::string &relative_entity_name = "")
{
  srv.request.model_name = name;
  srv.request.relative_entity_name = relative_entity_name;
  bool result = true;
  std::string service_name = "/gazebo/get_model_state";
  ros::ServiceClient client = nh.serviceClient<Service>(service_name);
  client.waitForExistence(ros::Duration(5));
  if (!client.call(srv))
  {
    ROS_ERROR_STREAM("Failed to call service " << service_name);
    result = false;
  }
  if (!srv.response.success)
  {
    ROS_ERROR_STREAM("Error calling " << service_name << " " << srv.response.status_message);
    result = false;
  }

  pose.pose = srv.response.pose;
  pose.header.stamp = srv.response.header.stamp;
//  ROS_WARN_STREAM(srv.response.header.stamp.sec << "." << srv.response.header.stamp.nsec);
  return result;
}

/**
 * @brief publishPose Publish the Model State position of the model "robot"
 * @param robot Name of the model
 * @param pub Publisher element
 * @param nh
 */
void publishPose(const std::string &robot, ros::Publisher &pub, ros::NodeHandle &nh)
{
  geometry_msgs::PoseStamped p;
  ros::Rate rate(200);
  gazebo_msgs::GetModelState srv;

  while (ros::ok())
  {
    getModelState(robot, p, nh, srv);
    pub.publish(p);
    rate.sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ground_truth_node");
  ros::NodeHandle nh("~");
  std::string robot;
  nh.param<std::string>("robot", robot, "tiago_steel");

  ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("/ground_truth_pose", 100);

  publishPose(robot, pub, nh);

  return 0;
}
