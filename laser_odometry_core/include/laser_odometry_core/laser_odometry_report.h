#ifndef _LASER_ODOMETRY_CORE_LASER_ODOMETRY_REPORT_H_
#define _LASER_ODOMETRY_CORE_LASER_ODOMETRY_REPORT_H_

#include "laser_odometry_core/laser_odometry_core.h"

namespace laser_odometry
{

  /**
   * @brief A (very)-simple report of the scan matching process.
   */
  struct LaserOdometryBase::ProcessReport
  {
    ProcessReport() = default;

    /**
     * @brief Constructor given one \c bool arg.
     * @param[in] p. Whether the matching was successful or not.
     */
    explicit ProcessReport(bool p) : processed(p) { }

    /**
     * @brief Constructor given two \c bool args.
     * @param[in] p. Whether the matching was successful or not.
     * @param[in] k. evaluated reading has become the new referent reading.
     */
    explicit ProcessReport(bool p, bool k) : processed(p), key_frame(k) { }

    /// @brief Whether the matching was successful or not.
    const bool processed = false;

    /// @brief Whether the evaluated reading has become the new
    /// referent reading.
    const bool key_frame = true;

    /**
     * @brief Conversion to \c bool operator.
     * @return Whether the matching was successful or not.
     */
    bool operator()() const noexcept
    {
      return processed;
    }

    /**
     * @brief A default error report.
     */
    static ProcessReport ErrorReport()
    {
      static const ProcessReport error_report{false, false};
      return error_report;
    }
  };

} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_CORE_LASER_ODOMETRY_REPORT_H_ */
