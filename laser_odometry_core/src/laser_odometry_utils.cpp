#include <laser_odometry_core/laser_odometry_utils.h>

namespace laser_odometry
{
namespace utils
{

void tfFromXYTheta(const double x, const double y, const double theta, Transform& t)
{
  Eigen::AngleAxis<Scalar> rollAngle(0,    Eigen::Vector3d::UnitX());
  Eigen::AngleAxis<Scalar> pitchAngle(0,   Eigen::Vector3d::UnitY());
  Eigen::AngleAxis<Scalar> yawAngle(theta, Eigen::Vector3d::UnitZ());

  Eigen::Quaternion<Scalar> q = rollAngle * pitchAngle * yawAngle;

  t = q;
  t.translation() = Eigen::Matrix<Scalar, 3, 1>(x, y, 0);
}

template <>
inline bool all_positive<float>(const std::vector<float> vec);

template <>
inline bool all_positive<double>(const std::vector<double> vec);

} /* namespace utils */
} /* namespace laser_odometry */
