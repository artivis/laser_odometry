/**
 * \file singleton.h
 *
 *  Created on: Aug 31, 2016
 *  \author: Jeremie Deray
 */

#ifndef SINGLETON_H_
#define SINGLETON_H_

#include <memory>
#include <assert.h>

namespace laser_odometry
{
namespace details
{

/**
* \brief A thread-safer? Singleton implementation with
* argument forwarding.
**/
template <class T>
class Singleton
{
  /**
  * \brief Custom deleter to by-pass private destructor issue.
  **/
  struct Deleter;

  using SingletonOPtr = std::unique_ptr<T, Deleter>;

public:

  template <typename... Args>
  static T& get(Args&&... args)
  {
    // c++11 equivalent to `std::make_unique`
    static SingletonOPtr instance_(new T(std::forward<Args>(args)...));

    assert(instance_ != nullptr);

    return *instance_;
  }

  constexpr Singleton(const Singleton&)       = delete;
  //constexpr Singleton(const Singleton&&)      = delete;

  constexpr Singleton& operator=(const Singleton&)  = delete;
  //constexpr Singleton& operator=(const Singleton&&) = delete;

protected:

  Singleton()          = default;
  virtual ~Singleton() = default;
};

template <class T>
struct Singleton<T>::Deleter
{
  void operator()( const T* const /*p*/ )
  {
    // Dunno why
    // `class_loader::MultiLibraryClassLoader::~MultiLibraryClassLoader()`
    // ain't happy with the following uncommented.
    //delete p;
  }
};

} /* namespace details */
} /* namespace laser_odometry */

#endif /* SINGLETON_H_ */
