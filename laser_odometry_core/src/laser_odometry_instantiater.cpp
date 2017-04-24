#include <laser_odometry_core/laser_odometry_instantiater.h>

namespace laser_odometry
{
/// @todo figure out why this doesn't compile

//LaserOdometryPtr LaserOdometryInstantiater::instantiate(const std::string& laser_odometry_type)
//{
//  return LaserOdometryInstantiater::get().instantiate_impl(laser_odometry_type);
//}

//LaserOdometryPtr LaserOdometryInstantiater::instantiate_impl(const std::string& laser_odometry_type)
//{
//  bool loaded = false;
//  LaserOdometryPtr laser_odom_ptr;

//  try {
//    laser_odom_ptr = loader.createInstance(laser_odometry_type);
//  }
//  catch (pluginlib::PluginlibException& ex){
//    ROS_ERROR("The plugin failed to load for some reason.\n\tError: %s", ex.what());
//    loaded = false;
//  }

//  if (laser_odom_ptr == nullptr){
//    ROS_ERROR_STREAM("Error creating laser odometry: "
//                     << laser_odometry_type);
//    loaded = false;
//  }
//  else
//  {
//    ROS_DEBUG_STREAM("Succes creating laser odometry: "
//                     << laser_odometry_type);

//    bool configured = laser_odom_ptr->configure();

//    if (!configured)
//    {
//      ROS_ERROR_STREAM("Something went wrong while configuring pluging : "
//                       << laser_odometry_type);
//      loaded = false;
//    }
//    else
//      loaded = true;
//  }

//  return laser_odom_ptr;
//}

} /* namespace laser_odometry */
