#ifndef _LASER_ODOMETRY_CORE_LASER_ODOMETRY_H_
#define _LASER_ODOMETRY_CORE_LASER_ODOMETRY_H_

#include <pluginlib/class_loader.h>

#include <laser_odometry_core/laser_odometry_core.h>

namespace laser_odometry
{

  class LaserOdometry
  {
    using LaserOdometerLoader = pluginlib::ClassLoader<LaserOdometryBase>;

  public:

    LaserOdometry()          = default;
    virtual ~LaserOdometry() = default;

    LaserOdometry(const std::string& laser_odometry_type);

    template <typename PoseMsgPtr>
    bool process(const sensor_msgs::LaserScanPtr scan,
                 PoseMsgPtr pose_msg_ptr,
                 PoseMsgPtr relative_pose_msg_ptr = nullptr)
    {
      if (!loaded_)
      {
        ROS_ERROR("No laser odometry plugin loaded!");
        return false;
      }

      if (pose_msg_ptr == nullptr) return false;

      assert(laser_odom_ptr_ != nullptr);

//      std::cout << "LaserOdometry::process " << std::endl;

      bool processed = laser_odom_ptr_->process(scan, pose_msg_ptr, relative_pose_msg_ptr);

//      std::cout << "LaserOdometry::process "<< processed << std::endl;

      sendTransform();

//      std::cout << "LaserOdometry::sendTransform " << std::endl;

      return processed;
    }

    void setInitialGuess(const tf::Transform& guess);

    bool loadLaserOdometer(const std::string& laser_odometry_type);

    bool broadcastTf() const noexcept;
    void broadcastTf(const bool broadcast) noexcept;

  protected:

    bool loaded_       = false;
    bool configured_   = false;
    bool broadcast_tf_ = true;

    // @todo metafactory should also work with shared ptr
    std::shared_ptr<LaserOdometerLoader> loader_ptr_;
    LaserOdometryBasePtr laser_odom_ptr_;

    tf::TransformBroadcaster tf_broadcaster_;

    void sendTransform();
  };

  using LaserOdometryPtr = std::shared_ptr<LaserOdometry>;

} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_CORE_LASER_ODOMETRY_H_ */
