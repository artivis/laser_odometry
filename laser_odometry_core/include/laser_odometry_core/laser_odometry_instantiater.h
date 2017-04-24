#ifndef _LASER_ODOMETRY_CORE_LASER_ODOMETRY_H_
#define _LASER_ODOMETRY_CORE_LASER_ODOMETRY_H_

#include <pluginlib/class_loader.h>

#include <laser_odometry_core/singleton.h>
#include <laser_odometry_core/laser_odometry_core.h>

namespace laser_odometry
{

class LaserOdometryInstantiater final : details::Singleton<LaserOdometryInstantiater>
{
protected:

  friend class details::Singleton<LaserOdometryInstantiater>;

  LaserOdometryInstantiater()  = default;
  ~LaserOdometryInstantiater() = default;

public:

  LaserOdometryInstantiater(LaserOdometryInstantiater&) = delete;
  void operator=(LaserOdometryInstantiater&)            = delete;

  static LaserOdometryPtr instantiate(const std::string& laser_odometry_type)
  {
    return LaserOdometryInstantiater::get().instantiate_impl(laser_odometry_type);
  }

protected:

  LaserOdometryPtr instantiate_impl(const std::string& laser_odometry_type)
  {
    bool loaded = false;
    LaserOdometryPtr laser_odom_ptr;

    try {
      laser_odom_ptr = loader.createInstance(laser_odometry_type);
    }
    catch (pluginlib::PluginlibException& ex){
      ROS_ERROR("The plugin failed to load for some reason.\n\tError: %s", ex.what());
      loaded = false;
    }

    if (laser_odom_ptr == nullptr){
      ROS_ERROR_STREAM("Error creating laser odometry: "
                       << laser_odometry_type);
      loaded = false;
    }
    else
    {
      ROS_DEBUG_STREAM("Succes creating laser odometry: "
                       << laser_odometry_type);

      bool configured = laser_odom_ptr->configure();

      if (!configured)
      {
        ROS_ERROR_STREAM("Something went wrong while configuring pluging : "
                         << laser_odometry_type);
        loaded = false;
      }
      else
        loaded = true;
    }

    return laser_odom_ptr;
  }

protected:

  pluginlib::ClassLoader<LaserOdometryBase> loader = {"laser_odometry_core",
                                                      "laser_odometry::LaserOdometryBase"};
};

} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_CORE_LASER_ODOMETRY_H_ */
