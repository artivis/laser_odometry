#ifndef _LASER_ODOMETRY_CORE_TRAJECTORY_INTERPOLATION_H_
#define _LASER_ODOMETRY_CORE_TRAJECTORY_INTERPOLATION_H_

#include <laser_odometry_core/ros_msg_buffer.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

namespace laser_odometry
{
namespace detail
{
constexpr int count_first_falses() { return 0; }

template <typename... B>
constexpr int count_first_falses(bool b1, B&&... b)
{
  return (b1)? 0 : 1 + count_first_falses(std::forward<B>(b)...);
}

template <typename E, typename... T>
auto get_by_type(std::tuple<T...>& tuple)
-> decltype(std::get<count_first_falses((std::is_same<T, E>::value)...)>(tuple))
{
  return std::get<count_first_falses((std::is_same<T, E>::value)...)>(tuple);
}

/// @todo stuff
template <typename E, typename... T>
auto get_by_type(const std::tuple<T...>& tuple)
-> decltype(std::get<count_first_falses((std::is_same<T, E>::value)...)>(tuple)) const
{
  return std::get<count_first_falses((std::is_same<T, E>::value)...)>(tuple);
}


///////////////

template <std::size_t...Is> struct index_sequence {};

//template <std::size_t N, std::size_t...Is>
//struct build : public build<N - 1, N - 1, Is...> {};

//template <std::size_t...Is>
//struct build<0, Is...> { using type = index_sequence<Is...>; };

//template <std::size_t N>
//using make_index_sequence = typename build<N>::type;



template<typename Seq1, typename Seq2> struct Concatenate;

template<size_t... S1, size_t... S2>
struct Concatenate<index_sequence<S1...>, index_sequence<S2...>> {
    using type = index_sequence<S1..., S2...>;
};

template<size_t first, size_t length>
struct make_range_imp;

template<size_t first, size_t length>
using make_range = typename make_range_imp<first, length>::type;

template<size_t first, size_t length>
struct make_range_imp {
   using type = typename Concatenate<make_range<first, length/2>,
make_range<first+length/2, length-length/2>>::type;
};

template<size_t first>
struct make_range_imp<first, 0> {
   using type = index_sequence<>;
};

template<size_t first>
struct make_range_imp<first, 1> {
   using type = index_sequence<first>;
};



// http://stackoverflow.com/a/32223343

//template <std::size_t... Ints>
//struct index_sequence
//{
//  using type = index_sequence;
//  using value_type = std::size_t;
//  static constexpr std::size_t size() noexcept { return sizeof...(Ints); }
//};

//// --------------------------------------------------------------

//template <class Sequence1, class Sequence2>
//struct _merge_and_renumber;

//template <std::size_t... I1, std::size_t... I2>
//struct _merge_and_renumber<index_sequence<I1...>, index_sequence<I2...>>
//    : index_sequence<I1..., (sizeof...(I1)+I2)...>
//{ };

//// --------------------------------------------------------------

//template <std::size_t N>
//struct make_index_sequence
//    : _merge_and_renumber<typename make_index_sequence<N/2>::type,
//    typename make_index_sequence<N - N/2>::type>
//{ };

//template<> struct make_index_sequence<0> : index_sequence<> { };
//template<> struct make_index_sequence<1> : index_sequence<0> { };

/////////////////////

//template<class F, class...Ts, std::size_t...Is>
//void for_each_in_tuple(const std::tuple<Ts...> & tuple, F func, make_index_sequence<Is...>){
//    using expander = int[];
//    (void)expander { 0, ((void)func(std::get<Is>(tuple)), 0)... };
//}

//template<class F, class...Ts>
//void for_each_in_tuple(const std::tuple<Ts...> & tuple, F func){
//    for_each_in_tuple(tuple, func, make_index_sequence<sizeof...(Ts)>());
//}

} // namespace detail

  template <typename... MsgsType>
  class TrajectoryInterpolator
  {
    template <typename T>
    using Buffer = details::RosMsgBuffer<T>;

    using Buffers = std::tuple<Buffer<MsgsType>...>;

    template <typename... Args>
    using decay_t = typename std::decay<Args...>::type;

    template <typename... Args>
    using DecayBuffer = Buffer<decay_t<Args...>>;

  public:

//    using FindStampPolicy = details::FindStampPolicy;

    using Stamp = std_msgs::Header::_stamp_type;
//    using Stamp = decltype(std::get<0>(msgs_))::Stamp;

    TrajectoryInterpolator()          = default;
    virtual ~TrajectoryInterpolator() = default;

    template <typename T, typename... Args>
    void add(T&& msg, Args&&... msgs)
    {
      detail::get_by_type<DecayBuffer<T>>(msgs_).emplace_back(std::forward<T>(msg));
      add(std::forward<Args>(msgs)...);
    }

    template <typename T>
    nav_msgs::Odometry get(const Stamp& start, const Stamp& stop)
    {
      return detail::get_by_type<DecayBuffer<T>>(msgs_).get_odom(start, stop);
    }

    template <typename T>
    void resize(const ros::Duration& size)
    {
      detail::get_by_type<DecayBuffer<T>>(msgs_).resize(size);
    }

    void resize_all(const ros::Duration& size)
    {
      resize_all_impl(size, detail::make_range<0, sizeof...(MsgsType)>{});
    }

    template <typename T>
    inline void stamp_policy(const FindStampPolicy policy) noexcept
    {
      detail::get_by_type<DecayBuffer<T>>(msgs_).stamp_policy(policy);
    }

    template <typename T>
    inline FindStampPolicy stamp_policy() const noexcept
    {
      return detail::get_by_type<DecayBuffer<T>>(msgs_).stamp_policy();
    }

    inline void stamp_policy_all(const FindStampPolicy policy) noexcept
    {
      stamp_policy_all_impl(policy, detail::make_range<0, sizeof...(MsgsType)>{});
    }

  protected:

    Buffers msgs_;

    void add() {/* End of parameter pack expansion */}

    template <std::size_t... Is>
    void resize_all_impl(const ros::Duration& size, detail::index_sequence<Is...>)
    {
      int unused[] = { 0, ( (void)std::get<Is>
                       (std::forward<Buffers>(msgs_)).resize(size), 0 )... };
      (void)unused; // blocks warnings
    }

    template <std::size_t... Is>
    void stamp_policy_all_impl(const FindStampPolicy policy, detail::index_sequence<Is...>)
    {
      int unused[] = { 0, ( (void)std::get<Is>
                       (std::forward<Buffers>(msgs_)).stamp_policy(policy), 0 )... };
      (void)unused; // blocks warnings
    }
  };

  template <typename... MsgsType>
  using TrajectoryInterpolatorPtr = std::shared_ptr<TrajectoryInterpolator<MsgsType...>>;

} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_CORE_TRAJECTORY_INTERPOLATION_H_ */
