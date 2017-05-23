#ifndef _LASER_ODOMETRY_CORE_ROS_MSG_BUFFER_H_
#define _LASER_ODOMETRY_CORE_ROS_MSG_BUFFER_H_

#include <type_traits>

#include <laser_odometry_core/circular_buffer.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace laser_odometry
{

enum class FindStampPolicy
{
  LT = 0,
  GT,
  CLOSEST,
  INTERPOLATE
};

namespace details
{

template<typename T0, typename T1>
struct Helper
{
  static void apply_policy(T0 it)
  {

  }
};

template <typename T>
T* get_pointer(T& t) { return &t; }

/**
 * @brief RosMsgBuffer class.
 *
 * A cirular buffer that eats only objects that
 * have a std_msgs::Header.
 */
template <typename T>
class RosMsgBuffer : public CircularBuffer<T, ros::Duration>
{
//protected:

  using Base = CircularBuffer<T, ros::Duration>;

  using Header = std_msgs::Header;

//  using Stamp = std_msgs::Header::_stamp_type;

  using Base::buffer_;
  using Base::max_size_;

public:

  using Stamp = std_msgs::Header::_stamp_type;
  using Base::CircularBuffer;
  using Base::buffer_size;

  /**
   * @brief default constructor. Size set to zero.
   */
  RosMsgBuffer()
  {
    static_assert(ros::message_traits::HasHeader<T>::value,
                  "Type T has no header !");
  }

  /**
   * @brief default destructor.
   */
  ~RosMsgBuffer() = default;

  Stamp get_first_stamp() const noexcept
  {
    return (Base::empty())? Stamp(0) : buffer_.begin()->header.stamp;
  }

  Stamp get_last_stamp() const noexcept
  {
    return (Base::empty())? Stamp(0) : buffer_.back().header.stamp;
  }

  ros::Duration size() const noexcept override
  {
    return get_dt();
  }

  inline void stamp_policy(const FindStampPolicy policy) noexcept
  {
    stamp_policy_ = policy;
  }

  inline FindStampPolicy stamp_policy() const noexcept
  {
    return stamp_policy_;
  }

  nav_msgs::Odometry get_odom(const Stamp& start, const Stamp& stop)
  {
    if (start < get_first_stamp() ||
        stop > get_last_stamp()   ||
        buffer_.empty())
    {
      ROS_ERROR("NOP");
      return nav_msgs::Odometry();
    }

    // Find the first message that has a stamp > start.
    typename Base::Iterator it_start =
        std::lower_bound(buffer_.begin(), buffer_.end(), start,
                         [](const T& e, const Stamp& s){return e.header.stamp < s;});

    // Find the first message that has a stamp <= stop.
    typename Base::Iterator it_stop =
        std::lower_bound(it_start, buffer_.end(), stop,
                         [](const T& e, const Stamp& s) {return e.header.stamp < s;});

//    std::cout << "it_start " << it_start->header.stamp << std::endl;
//    std::cout << "it_stop "  << it_stop->header.stamp  << std::endl;

    // Apply stamp search policy
    auto new_start = apply_policy(it_start, start);
    auto new_stop  = apply_policy(it_stop,  stop);

//    std::cout << "new_start " << new_start.header.stamp << std::endl;
//    std::cout << "new_stop "  << new_stop.header.stamp  << std::endl;

//    return nav_msgs::Odometry();

    std::list<T*> msgs_to_integrate;
    msgs_to_integrate.push_back(&new_start);

    std::transform(std::next(it_start), std::prev(it_stop),
                   std::back_inserter(msgs_to_integrate),
                   get_pointer<T>);

    msgs_to_integrate.push_back(&new_stop);

    return integrate(msgs_to_integrate);
  }

protected:

  FindStampPolicy stamp_policy_ = FindStampPolicy::CLOSEST;

  ros::Duration get_dt() const noexcept
  {
    return get_last_stamp() - get_first_stamp();
  }

  /**
   * @brief assert_size
   * @return
   */
  virtual bool assert_size() const override
  {
    return get_dt() < max_size_;
  }

  nav_msgs::Odometry apply_policy(typename Base::Iterator it, const Stamp& stamp)
  {
    switch (stamp_policy_)
    {
    case FindStampPolicy::LT:
    {
      return (it!=buffer_.begin())? *std::prev(it) : *it;
      break;
    }
    case FindStampPolicy::GT:
    {
      return *it;
      break;
    }
    case FindStampPolicy::CLOSEST:
    {
      auto it_prev = ((it != buffer_.begin())? std::prev(it) : it);

      if (it_prev == it) return *it;

      return std::min(*it_prev, *it,
                      [&](const T& x, const T& y)
                      {
                        return std::abs((x.header.stamp - stamp).toNSec()) <
                               std::abs((y.header.stamp - stamp).toNSec());
                      });
      break;
    }
    case FindStampPolicy::INTERPOLATE:
    {
      return *it;
      break;
    }
    }
    return *it;
  }

  nav_msgs::Odometry integrate(const std::list<T*>)
  {
    return nav_msgs::Odometry();
  }
};

template <>
nav_msgs::Odometry RosMsgBuffer<nav_msgs::Odometry>::integrate(const std::list<nav_msgs::Odometry*> list)
{
  const auto& start_pose = *(*list.begin());
  const auto& stop_pose  =  (*list.back());

  /// @todo

  nav_msgs::Odometry dt;

  dt.pose.pose.position.x = stop_pose.pose.pose.position.x - start_pose.pose.pose.position.x;
  dt.pose.pose.position.y = stop_pose.pose.pose.position.y - start_pose.pose.pose.position.y;
  dt.pose.pose.position.z = stop_pose.pose.pose.position.z - start_pose.pose.pose.position.z;

  dt.pose.pose.orientation = stop_pose.pose.pose.orientation/* - start_pose.pose.pose.orientation*/;

  return dt;
}

} /* namespace details */
} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_CORE_ROS_MSG_BUFFER_H_ */
