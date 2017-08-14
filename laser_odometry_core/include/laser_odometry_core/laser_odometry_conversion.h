#ifndef _LASER_ODOMETRY_CORE_LASER_ODOMETRY_CONVERSION_H_
#define _LASER_ODOMETRY_CORE_LASER_ODOMETRY_CONVERSION_H_

namespace laser_odometry {
namespace conversion {

template <class Input, class Ouput>
void toRos(const Input&, Ouput&);

template <class Input, class Ouput>
void fromRos(const Input&, Ouput&);

} /* namespace conversion */
} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_CORE_LASER_ODOMETRY_CONVERSION_H_ */
