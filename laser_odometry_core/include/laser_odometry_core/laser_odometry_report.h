#ifndef _LASER_ODOMETRY_CORE_LASER_ODOMETRY_REPORT_H_
#define _LASER_ODOMETRY_CORE_LASER_ODOMETRY_REPORT_H_

#include "laser_odometry_core/laser_odometry_core.h"

namespace laser_odometry
{

  struct LaserOdometryBase::ProcessReport
  {
    ProcessReport() = default;
    explicit ProcessReport(bool p) : processed(p) { }
    explicit ProcessReport(bool p, bool k) : processed(p), key_frame(k) { }

    const bool processed = false;
    const bool key_frame = true;

    const bool operator()() const noexcept
    {
      return processed;
    }

    static ProcessReport ErrorReport()
    {
      static const ProcessReport error_report{false, false};
      return error_report;
    }
  };

} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_CORE_LASER_ODOMETRY_REPORT_H_ */
