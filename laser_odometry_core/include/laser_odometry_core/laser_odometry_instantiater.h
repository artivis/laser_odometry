#ifndef _LASER_ODOMETRY_CORE_LASER_ODOMETRY_H_
#define _LASER_ODOMETRY_CORE_LASER_ODOMETRY_H_

#include <pluginlib/class_loader.h>

#include <laser_odometry_core/singleton.h>
#include <laser_odometry_core/laser_odometry_core.h>

namespace
{

class LaserOdometryInstantiater final
{
public:

  LaserOdometryInstantiater()  = default;
  ~LaserOdometryInstantiater() = default;

  // Not copyable
  LaserOdometryInstantiater(LaserOdometryInstantiater&) = delete;
  void operator=(LaserOdometryInstantiater&)            = delete;

  laser_odometry::LaserOdometryPtr instantiate_impl(const std::string& laser_odometry_type)
  {
    laser_odometry::LaserOdometryPtr laser_odom_ptr;

    try
    {
      laser_odom_ptr = loader.createInstance(laser_odometry_type);
    }
    catch (pluginlib::PluginlibException& ex)
    {
      ROS_ERROR("The plugin failed to load for some reason.\n\tError: %s", ex.what());
    }

    if (laser_odom_ptr == nullptr)
    {
      ROS_ERROR_STREAM("Error creating laser odometry: "
                       << laser_odometry_type);
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
      }
    }

    return laser_odom_ptr;
  }

protected:

  pluginlib::ClassLoader<laser_odometry::LaserOdometryBase> loader =
    {"laser_odometry_core","laser_odometry::LaserOdometryBase"};
};

} /* namespace */

namespace laser_odometry
{

namespace detail
{
using Instantiater = details::Singleton<LaserOdometryInstantiater>;
}

LaserOdometryPtr make_laser_odometry(const std::string& laser_odometry_type)
{
  return detail::Instantiater::get().instantiate_impl(laser_odometry_type);
}

} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_CORE_LASER_ODOMETRY_H_ */
