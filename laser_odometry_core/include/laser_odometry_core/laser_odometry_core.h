#ifndef _LASER_ODOMETRY_CORE_LASER_ODOMETRY_H_
#define _LASER_ODOMETRY_CORE_LASER_ODOMETRY_H_

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <property_bag/property_bag.h>

namespace laser_odometry
{

  class LaserOdometryBase{

  public:

    LaserOdometryBase();

    virtual ~LaserOdometryBase() = default;

    virtual bool configure(const property_bag::PropertyBag &/*parameters*/) = 0;

    virtual bool process(const sensor_msgs::LaserScan& /*scan*/,
                         geometry_msgs::Pose2DPtr /*pose*/) = 0;

    virtual bool process(const sensor_msgs::LaserScan& /*scan*/,
                         geometry_msgs::PoseWithCovarianceStampedPtr /*pose*/) = 0;

    virtual void clear(){
      /* @todo */
      configured_ = false;
    }

  protected:

    bool configured_;
  };

  typedef boost::shared_ptr<LaserOdometryBase> LaserOdometryBasePtr;

} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_CORE_LASER_ODOMETRY_H_ */
