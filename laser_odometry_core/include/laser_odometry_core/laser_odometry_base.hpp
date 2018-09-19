#ifndef _LASER_ODOMETRY_CORE_LASER_ODOMETRY_BASE_HPP_
#define _LASER_ODOMETRY_CORE_LASER_ODOMETRY_BASE_HPP_

// Un-comment for IDE highlights
//#include <laser_odometry_core/laser_odometry_base.h>

namespace laser_odometry {

template <typename Msg>
void LaserOdometryBase::updateKf(Msg&& msg)
{
  updateKfMsg(std::forward<Msg>(msg));
  updateKfTransform();
}

template <typename PoseMsgT, typename IncrementMsgT>
void LaserOdometryBase::fillMsgs(PoseMsgT&& pose_msg_ptr,
                                 IncrementMsgT&& increment_msg_ptr)
{
  fillMsg(std::forward<PoseMsgT>(pose_msg_ptr));
  fillIncrementMsg(std::forward<IncrementMsgT>(increment_msg_ptr));
}

template <typename Msg>
inline LaserOdometryBase::ProcessReport
LaserOdometryBase::process(Msg&& msg)
{
  if (msg == nullptr)
  {
    ROS_WARN("Laser odometry process function received a nullptr input message!");
    return ProcessReport::ErrorReport();
  }

  ros::WallTime start = ros::WallTime::now();

  has_new_kf_   = false;
  current_time_ = msg->header.stamp;

  // first message
  if (!initialized_)
  {
    initialized_ = initialize(std::forward<Msg>(msg));

    initializeFrames();

    ROS_INFO_STREAM_COND(initialized_, "LaserOdometry Initialized!");

    return ProcessReport{true, true};
  }

  preProcessing();

  // the predicted change of the laser's position, in the laser frame
  const Transform increment_prior_in_laser = getIncrementPriorInLaserFrame();

  // The actual computation
  const bool processed = processImpl(std::forward<Msg>(msg), increment_prior_in_laser);

  assertIncrement();
  assertIncrementCovariance();

  posePlusIncrement(processed);

  has_new_kf_ = isKeyFrame(getIncrementSinceKeyFrame());

  if (has_new_kf_)
  {
    // generate a keyframe
    updateKf(std::forward<Msg>(msg));

    isKeyFrame();
  }
  else
    isNotKeyFrame();

  postProcessing();

  execution_time_ = ros::WallTime::now() - start;

  return ProcessReport{processed, has_new_kf_};
}

template <typename Msg, typename PoseMsgT, typename IncrementMsgT>
LaserOdometryBase::ProcessReport
LaserOdometryBase::process(Msg&& msg,
                           PoseMsgT&& pose_msg,
                           IncrementMsgT&& pose_increment_msg)
{
  const auto report = process(std::forward<Msg>(msg));

  fillMsgs(std::forward<PoseMsgT>(pose_msg),
           std::forward<IncrementMsgT>(pose_increment_msg));

  return report;
}

} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_CORE_LASER_ODOMETRY_BASE_HPP_ */
