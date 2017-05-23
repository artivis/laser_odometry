#ifndef _LASER_ODOMETRY_CORE_CIRCULAR_BUFFER_H_
#define _LASER_ODOMETRY_CORE_CIRCULAR_BUFFER_H_

#include <deque>
#include <list>

namespace laser_odometry
{
namespace details
{

//template<class T>
//T begin(std::pair<T, T>& p)
//{
//  return p.first;
//}
//template<class T>
//T end(std::pair<T, T>& p)
//{
//  return p.second;
//}
//template<class Iterator>
//std::reverse_iterator<Iterator> make_reverse_iterator(Iterator it)
//{
//  return std::reverse_iterator<Iterator>(it);
//}
//template<class Range>
//std::pair<std::reverse_iterator<decltype(begin(std::declval<Range>()))>,
//          std::reverse_iterator<decltype(begin(std::declval<Range>()))>>
//make_reverse_range(Range&& r)
//{
//  return std::make_pair(make_reverse_iterator(begin(std::forward<Range>(r))),
//                        make_reverse_iterator(end(std::forward<Range>(r))));
//}

/**
 * @brief CircularBuffer class.
 *
 * FIFO like.
 */
template <typename T, typename Size = std::size_t>
class CircularBuffer
{
protected:

  /**
   * @brief max_size_. The buffer size.
   */
  Size max_size_ = Size(0);

  /**
   * @brief buffer_. The underlying container.
   */
  std::deque<T> buffer_;
//  std::list<T> buffer_;

  using Iterator = typename std::deque<T>::iterator;
  using ConstIterator = typename std::deque<T>::const_iterator;

  using RIterator = typename std::deque<T>::reverse_iterator;
  using RConstIterator = typename std::deque<T>::const_reverse_iterator;

  /**
   * @brief assert_size
   * @return
   */
  virtual bool assert_size() const
  {
    std::cout << "max_size " << max_size_ << std::endl;
    std::cout << "size() " << size() << std::endl;

    return size() <= max_size_;
  }

  void maintain_size()
  {
    while (!assert_size()) buffer_.pop_front();
  }

public:

  /**
   * @brief default constructor. Size set to zero.
   */
  CircularBuffer()  = default;

  /**
   * @brief default destructor.
   */
  ~CircularBuffer() = default;

  /**
   * @brief constructor given the buffer size.
   */
  inline constexpr CircularBuffer(const Size size) noexcept :
    max_size_(size)
  {
    //static_assert(std::is_arithmetic<Size>::value, "Invalid type Size !");
  }

  /**
   * @brief constructor given an initializer list.
   * The buffer size is set accordingly to the number
   * of arguments.
   */
  inline constexpr CircularBuffer(std::initializer_list<T>&& l) :
    max_size_(l.size()), buffer_(l) { }

  /**
   * @brief constructor given objects.
   * The buffer size is set accordingly to the number
   * of objects.
   */
//  template <typename... Args>
//  inline constexpr CircularBuffer(T&& e, Args&&... args) : ??????????????????
//    max_size_(sizeof...(args)), buffer_({e, args...}) { }

  /**
   * @brief operator =
   * @param o, another CircularBuffer<T>
   */
  inline CircularBuffer<T>& operator = (const CircularBuffer<T> o)
  {
    max_size_ = o.max_size_,
    buffer_   = o.buffer_;
    return *this;
  }

  /**
   * @brief buffer_size. Get the size of the buffer.
   * @return buffer_size. The size of the buffer.
   */
  inline constexpr Size buffer_size() const noexcept { return max_size_; }

  /**
   * @brief buffer_size. Get the size of the buffer.
   * @return buffer_size. The size of the buffer.
   */
//  inline void buffer_size(const Size size) noexcept { max_size_ = size; }

  /**
   * @brief size. Number of elements in the buffer.
   * @return Number of elements in the buffer.
   */
  virtual inline Size size() const noexcept { return Size(buffer_.size()); }

  /**
   * @brief empty. Whether the buffer is empty or not.
   * @return bool. True if empty, false otherwise.
   */
  inline bool empty() const noexcept { return buffer_.empty(); }

  /**
   * @brief resize. Resize the size of the buffer.
   * @param size. The size of the buffer.
   *
   * @see std::list::resize()
   */
  inline void resize(const Size size) noexcept
  {
    max_size_ = size;

    maintain_size();
  }

  /**
   * @brief push_back. Push the argument at the back of the container.
   * If the number of elements is greater than the buffer size,
   * objects at the buffer front are popped out.
   * @param e. Object to add at the back of the container.
   *
   * @see size()
   * @see count()
   */
  inline void push_back(const T& e) noexcept
  {
    buffer_.push_back(e);

    maintain_size();
  }

//  /**
//   * @brief push_front. Push the argument at the front of the container.
//   * If the number of elements is greater than the buffer size,
//   * objects at the buffer back are popped out.
//   * @param e. Object to add at the front of the container.
//   *
//   * @see size()
//   * @see count()
//   * @see push_back()
//   */
//  inline void push_front(const T& e) noexcept
//  {
//    buffer_.push_front(e);

//    maintain_size();
//  }

  /**
   * @brief emplace_back. Create an object at the back of the container from the arguments.
   * If the number of elements is greater than the buffer size,
   * objects at the buffer front are popped out.
   * @param args. Arguments to create a new object.
   *
   * @see size()
   * @see count()
   */
  template <typename... Args>
  inline void emplace_back(Args&&... args)
  {
    buffer_.emplace_back(std::forward<Args>(args)...);

    maintain_size();
  }

  /**
   * @brief front.
   *
   * @return (const_)reference to the first element.
   */
  inline auto front() -> decltype(buffer_.front())
  {
    return buffer_.front();
  }

  /**
   * @brief back.
   *
   * @return (const_)reference to the last element.
   */
  inline auto back() -> decltype(buffer_.back())
  {
    return buffer_.back();
  }

  /**
   * @brief operator [].
   * @param i. Index of the element to return.
   * @return The 'i'th element of the buffer.
   */
  inline auto operator[](const std::size_t i) -> decltype(buffer_[i])
  {
    return buffer_[i];
  }

  /**
   * @brief operator [].
   * @param i. Index of the element to return.
   * @return The 'i'th element of the buffer.
   */
  inline auto operator[](const std::size_t i) const -> decltype(buffer_[i])
  {
    return buffer_[i];
  }

  /**
   * @brief begin.
   * @return iterator to the buffer head.
   */
  inline auto begin() -> decltype(buffer_.begin())
  {
    return buffer_.begin();
  }

  /**
   * @brief begin.
   * @return iterator to the buffer head.
   */
  inline auto begin() const -> decltype(buffer_.begin())
  {
    return buffer_.begin();
  }

  /**
   * @brief end.
   * @return iterator to the buffer tail.
   */
  inline auto end() -> decltype(buffer_.end())
  {
    return buffer_.end();
  }

  /**
   * @brief end.
   * @return iterator to the buffer tail.
   */
  inline auto end() const -> decltype(buffer_.end())
  {
    return buffer_.end();
  }

  /**
   * @brief clear. Clear the buffer.
   */
  inline void clear()
  {
    buffer_.clear();
  }
};

} /* namespace details */
} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_CORE_CIRCULAR_BUFFER_H_ */
