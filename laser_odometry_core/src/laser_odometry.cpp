#include <laser_odometry_core/laser_odometry.h>

namespace laser_odometry
{

LaserOdometry::LaserOdometry(const std::string& laser_odometry_type)
{
  loadLaserOdometer(laser_odometry_type);
}

bool LaserOdometry::loadLaserOdometer(const std::string& laser_odometry_type)
{
  if (loader_ptr_ == nullptr)
    loader_ptr_ = std::make_shared<LaserOdometerLoader>("laser_odometry_core",
                                                        "laser_odometry::LaserOdometryBase");

  assert(loader_ptr_ != nullptr);

  try {
    laser_odom_ptr_ = loader_ptr_->createInstance(laser_odometry_type);
  }
  catch (pluginlib::PluginlibException& ex){
    ROS_ERROR("The plugin failed to load for some reason.\n\tError: %s", ex.what());
    loaded_ = false;
  }

  if (laser_odom_ptr_ == nullptr){
    ROS_ERROR_STREAM("Error creating laser odometry: " << laser_odometry_type);
    loaded_ = false;
  }
  else
  {
    ROS_DEBUG_STREAM("Succes creating laser odometry: " << laser_odometry_type);

    bool configured = laser_odom_ptr_->configure();

    if (!configured)
    {
      ROS_ERROR_STREAM("Something went wrong while configuring pluging : " << laser_odometry_type);
      loaded_ = false;
    }
    else
      loaded_ = true;
  }

  return loaded_;
}

bool LaserOdometry::broadcastTf() const noexcept
{
  return broadcast_tf_;
}

void LaserOdometry::broadcastTf(const bool broadcast) noexcept
{
  broadcast_tf_ = broadcast;
}

void LaserOdometry::sendTransform()
{
  if (broadcast_tf_)
  {
    tf::StampedTransform transform_msg(laser_odom_ptr_->world_to_base_,
                                       laser_odom_ptr_->current_time_,
                                       laser_odom_ptr_->world_frame_,
                                       laser_odom_ptr_->laser_odom_frame_);

    transform_msg.stamp_ = laser_odom_ptr_->current_time_;

    tf_broadcaster_.sendTransform(transform_msg);
  }
}

} /* namespace laser_odometry */
