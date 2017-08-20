#ifndef _LASER_ODOMETRY_CORE_LASER_ODOMETRY_BASE_H_
#define _LASER_ODOMETRY_CORE_LASER_ODOMETRY_BASE_H_

#include <laser_odometry_core/laser_odometry_transform.h>

// The input ROS messages supported
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

// More ROS header
#include <ros/node_handle.h>

namespace laser_odometry
{
  using TransformWithCovariancePtr = boost::shared_ptr<TransformWithCovariance>;

  /**
   * @enum OdomType
   * @brief The type of odometry computed.
   */
  enum class OdomType : std::size_t
  {
    Unknown,
    Odom2D,
    Odom2DCov,
    Odom3D,
    Odom3DCov
  };

  /**
   * @class LaserOdometryBase.
   * @brief Base class for the laser_odometry plugins.
   *
   * This class represents an abstract scan/pointcloud matcher for
   * odometry computation.
   *
   * It is though as a key-frame based scan/pointcloud matcher,
   * meaning that the reference reading is only updated upon
   * isKeyFrame() function returning true.
   *
   * The derived class is required to implement a limited number
   * of functions (see below) while most of the common operations are implemented
   * in this base class (e.g. frame composition).
   *
   *
   * Given a new reading, the overall algorithm takes place in LaserOdometryBase::process
   * and can be summarized as follows:
   *
   * process :
   *
   *       - if first reading
   *           - initialize                         [O]
   *           - return
   *
   *       - preProcessing                          [O]
   *
   *       - getIncrementPrior
   *
   *                    increment_in_base_prior_ *IF*
   *                 /  set using setIncrementPrior
   *          return |
   *                 \  predict()  otherwise        [O]
   *
   *       - processImpl                            [X]
   *
   *       - posePlusIncrement                      [X]
   *
   *       - is_keyframe = isKeyFrame               [O]
   *
   *       - if is_keyframe is true
   *
   *          - isKeyFrame                          [O]
   *
   *       - if is_keyframe is \b not true
   *
   *          - isNotKeyFrame                       [O]
   *
   *       - postProcessing                         [O]
   *
   * [X] Symbol marks the necessity for the base class to override the function.<br>
   * [O] Symbol marks optionality for the base class to override the function.
   *
   */
  class LaserOdometryBase
  {
  public:

    /// @brief A brief report of the matching.
    struct ProcessReport;

  public:

    /**
     * @brief Default constructor.
     */
    LaserOdometryBase()          = default;

    /**
     * @brief Default destructor.
     */
    virtual ~LaserOdometryBase() = default;

    /**
     * @brief Compute the 2D odometry given a sensor_msgs::LaserScan.
     * @param[in] scan_msg. The input LaserScan.
     * @return ProcessReport. A brief summary of the scan matching process.
     *
     * @see ProcessReport
     */
    ProcessReport process(const sensor_msgs::LaserScanConstPtr& scan_msg);

    /**
     * @brief Compute the 2D odometry given a sensor_msgs::PointCloud2.
     * @param[in] cloud_msg. The input PointCloud2.
     * @return ProcessReport. A brief summary of the scan matching process.
     *
     * @see ProcessReport
     */
    ProcessReport process(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

    /**
     * @brief Compute the 2D odometry given a sensor_msgs::LaserScan.
     * @param[in] scan_msg. The input LaserScan.
     * @param[out] pose. The estimated 2D pose.
     * @param[out] pose_increment. The estimated 2D pose increment.
     * @return ProcessReport. A brief summary of the scan matching process.
     *
     * @see ProcessReport
     */
    template <typename PoseMsgT, typename IncrementMsgT>
    ProcessReport process(const sensor_msgs::LaserScanConstPtr& scan_msg,
                          PoseMsgT&& pose_msg,
                          IncrementMsgT&& pose_increment_msg = nullptr);

    /**
     * @brief Compute the 2D odometry given a sensor_msgs::PointCloud2.
     * @param[in] scan_msg. The input PointCloud2.
     * @param[out] pose. The estimated 2D pose.
     * @param[out] pose_increment. The estimated 2D pose increment.
     * @return ProcessReport. A brief summary of the scan matching process.
     *
     * @note Ouput message type supported:
     *
     * - TransformWithCovariancePtr
     * - geometry_msgs::Pose2DPtr
     * - nav_msgs::OdometryPtr
     *
     * @see ProcessReport
     */
    template <typename PoseMsgT, typename IncrementMsgT>
    ProcessReport process(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                          PoseMsgT&& pose_msg,
                          IncrementMsgT&& pose_increment_msg = nullptr);

  protected:

    /**
     * @brief Function to be implemented by the derived plugin.
     * It is the core (implementation) of the LaserOdometryBase::process function.
     * The base class implementation throws a std::runtime_error in case the function
     * is called while not being overloaded.
     * @param[in] laser_msg. The input LaserScan.
     * @param[in] prediction. A prediction regarding the displacement increment.
     * @return Whether or not the scan matching was successful.
     *
     * @note [Necessary] To be implemented in the derived class.
     */
    virtual bool processImpl(const sensor_msgs::LaserScanConstPtr& laser_msg,
                             const Transform& prediction);

    /**
     * @brief Function to be implemented by the derived plugin.
     * It is the core (implementation) of the LaserOdometryBase::process function.
     * The base class implementation throws a std::runtime_error in case the function
     * is called while not being overloaded.
     * @param[in] cloud_msg. The input PointCloud2.
     * @param[in] prediction. A prediction regarding the displacement increment.
     * @return Whether or not the pointcloud matching was successful.
     *
     * @note [Necessary] To be implemented in the derived class.
     */
    virtual bool processImpl(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                             const Transform& prediction);

  public:

    /**
     * @brief Return the current estimated pose.
     * @return A const-reference Transform.
     */
    const Transform& getEstimatedPose() const noexcept;

    /**
     * @brief Reset the matcher.
     * The base class implemetation resets all transforms to
     * Identity and the key-reading to nullptr.
     */
    virtual void reset();

    /**
     * @brief Hard reset the matcher.
     * The base class implemetation resets all transforms to
     * Identity and the key-reading to nullptr.
     */
    virtual void hardReset();

    /**
     * @brief Configure the base class various parameters.
     * It internally calls configureImpl() to allow
     * the derived class to have it own configuration.
     * @return Whether the class is configured or not.
     *
     * @see configureImpl.
     */
    bool configure();

    /**
     * @brief Whether the class is configured or not.
     * @return Whether the class is configured or not.
     */
    bool configured() const noexcept;

    /* Guetter / Setter */

    /**
     * @brief Const-reference to the origin frame.
     * @return Const-reference to the origin frame.
     */
    const Transform& getOrigin() const;

    /**
     * @brief Set the origin frame.
     * @param[in] origin. The origin frame.
     */
    void setOrigin(const Transform& origin);

    /**
     * @brief Const-reference to the increment prior in the base_frame
     * of the upcoming matching.
     * It is the prior of the pose increment in the base_frame
     * from the last processed scan to the current one.
     * @return Const-reference to the increment prior of the upcoming matching.
     */
    const Transform& getIncrementPrior() const;

    /**
     * @brief Set the increment prior in the base_frame
     * of the upcoming matching.
     * It is the prior of the pose increment in the base_frame
     * from the last processed scan to the current one.
     * @param[in] guess. The increment prior of the upcoming matching.
     */
    void setIncrementPrior(const Transform& increment_in_base_prior);

    /**
     * @brief Const-reference to the laser pose wrt the robot base_frame.
     * @return Const-reference to the laser pose wrt the robot base_frame.
     */
    const Transform& getLaserPose() const;

    /**
     * @brief Set the laser pose wrt the robot base frame.
     * @param[in] base_to_laser.
     */
    void setLaserPose(const Transform& base_to_laser);

    /// @brief the robot base frame name.
    /// @return the robot base frame name.
    const std::string& getFrameBase()  const noexcept;

    /// @brief the robot laser frame name.
    /// @return the robot laser frame name.
    const std::string& getFrameLaser() const noexcept;

    /// @brief the global fixed frame name.
    /// @return the global fixed frame name.
    const std::string& getFrameFixed() const noexcept;

    /// @brief the global odometry frame name.
    /// @return the global odometry frame name.
    const std::string& getFrameOdom()  const noexcept;

    /// @brief Set the robot base frame name.
    /// @param[in] frame.
    void setFrameBase(const std::string& frame);

    /// @brief Set the robot laser frame name.
    /// @param[in] frame.
    void setFrameLaser(const std::string& frame);

    /// @brief Set the global fixed frame name.
    /// @param[in] frame.
    void setFrameFixed(const std::string& frame);

    /// @brief Set the global odometry frame name.
    /// @param[in] frame.
    void setFrameOdom(const std::string& frame);

    /**
     * @brief Get the time stamp of the latest reading evaluated.
     * @return Const-reference ros::Time.
     */
    const ros::Time& getCurrentTime() const noexcept;

    /**
     * @brief Return the execution time of the last
     * process call.
     * @return Const-reference to process execution time.
     */
    const ros::WallDuration& getExecutionTime() const noexcept;

    /**
     * @brief setType. Set the plugin name.
     * @note This function is only meant to be
     * used by the Instantiater.
     * @param type. The plugin name.
     */
    void setType(const std::string& type);

    /**
     * @brief getType. Get the plugin name.
     * @return The plugin name.
     */
    std::string getType() const noexcept;

    /**
     * @brief The type of odometry computed by the plugin.
     * @return The type of odometry computed by the plugin.
     * If not overrided, default is OdomType::Unknown
     *
     * @see OdomType
     */
    virtual OdomType odomType() const noexcept;

    /**
     * @brief Whether the reference reading has been updated or not.
     * @return Whether the reference reading has been updated or not.
     */
    bool hasNewKeyFrame() const noexcept;

    /**
     * @brief Get the referent LaserScan.
     * @param[in] the sensor_msgs::LaserScan key-frame.
     */
    void getKeyFrame(sensor_msgs::LaserScanConstPtr& key_frame_msg) const noexcept;

    /**
     * @brief Set the referent LaserScan.
     * @param[in] the sensor_msgs::LaserScan key-frame.
     */
    void setKeyFrame(const sensor_msgs::LaserScanConstPtr& key_frame_msg);

    /**
     * @brief Get the referent PointCloud2.
     * @param[in] the sensor_msgs::PointCloud2 key-frame.
     */
    void getKeyFrame(sensor_msgs::PointCloud2ConstPtr& key_frame_msg) const noexcept;

    /**
     * @brief Set the referent PointCloud2.
     * @param[in] the sensor_msgs::PointCloud2 key-frame.
     */
    void setKeyFrame(const sensor_msgs::PointCloud2ConstPtr& key_frame_msg);

  protected:

    bool configured_   = false; /// @brief Whether the matcher is configured.
    bool initialized_  = false; /// @brief Whether the matcher is initialized.
    bool has_new_kf_   = false; /// @brief Whether the matcher has a new referent reading.

    /// @brief The type of this odometer. Overrided by the derived plugin.
    std::string laser_odometry_type_ = "laser_odometry_core::LaserOdometryBase";

    /// @brief The execution time of the last process call.
    ros::WallDuration execution_time_;

    /// @brief The default increment covariance diagonal.
    std::vector<Scalar> default_cov_diag_ = std::vector<Scalar>(6, default_cov_diag_val);

    /// @brief The estimated pose covariance.
    Covariance pose_covariance_;

    /// @brief The estimated pose increment covariance.
    Covariance increment_covariance_;

    ros::NodeHandle private_nh_ = ros::NodeHandle("~");

    std::string base_frame_       = "base_link";        /// @brief The robot base frame name.
    std::string laser_frame_      = "base_laser_link";  /// @brief The robot laser frame name.
    std::string fixed_frame_      = "map";              /// @brief The global fixed frame name.

    /// @brief The global odometry frame name.
    /// This frame name is only used in the published message.
    std::string laser_odom_frame_ = "laser_odom";

    /// @brief Tranform from base_frame to laser_frame
    Transform base_to_laser_ = Transform::Identity();

    /// @brief Tranform from laser_frame to base_frame
    /// == base_to_laser_^-1
    Transform laser_to_base_ = Transform::Identity();

    /// @brief The relative transform in the laser_frame.
    /// @note This is the transform the derived class should fills.
    Transform increment_ = Transform::Identity();

    /// @brief The increment in the base_frame.
    Transform increment_in_base_ = Transform::Identity();

    /// @brief Guessed/predicted tranform
    /// from reference_'reading' to
    /// current_'reading' in the base_frame.
    Transform increment_in_base_prior_ = Transform::Identity();

    /// @brief Tranform from fixed_frame
    /// to base_frame, where fixed_frame
    /// is the origin of the integration.
    /// == fixed_to_base_kf_ * increment_in_base_.
    Transform fixed_to_base_ = Transform::Identity();

    /// @brief Tranform from fixed_frame to
    /// the last keyfame frame.
    /// == fixed_to_base * increment_in_base_.
    Transform fixed_to_base_kf_ = Transform::Identity();

    /// @brief An optional user defined
    /// transform from that maps the
    /// integration origin to another
    /// reference than Identity
    /// Default: Identity
    Transform fixed_origin_ = Transform::Identity();

    /// @brief Tranform from the fixed_origin frame
    /// to the base_frame.
    /// It is the integrated robot odometry.
    /// == fixed_origin_ * fixed_to_base_.
    Transform fixed_origin_to_base_ = Transform::Identity();

    /// @brief  The referent LaserScan.
    sensor_msgs::LaserScanConstPtr   reference_scan_;

    /// @brief  The referent PointCloud2.
    sensor_msgs::PointCloud2ConstPtr reference_cloud_;

    /// @brief Current ros::Time accordingly
    /// to the last processed message.
    ros::Time current_time_;

    /**
     * @brief Called within configure(), it allow the derived class
     * to perform its configuration step.
     * @return Whether the class is configured or not.
     *
     * @note The base class returns \b true by default.
     */
    virtual bool configureImpl();

    /**
     * @brief Call upon the first LaserScan reception it allows the derived class
     * to perfom some reading-dependent initializations.
     * @param[in] The first LaserScan reading.
     * @return Whether the class is initialized.
     *
     * @note The base class returns \b true by default.
     */
    virtual bool initialize(const sensor_msgs::LaserScanConstPtr&   scan_msg);

    /**
     * @brief Call upon the first PointCloud2 reception it allows the derived class
     * to perfom some reading-dependent initializations.
     * @param[in] The first PointCloud2 reading.
     * @return Whether the class is initialized.
     *
     * @note The base class returns \b true by default.
     */
    virtual bool initialize(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

    /**
     * @brief Perform a prediction of the pose increment for the upcoming matching.
     * @param[in] The last estimated pose increment in the \b world_frame_ frame.
     * @return A prediction of the upcoming pose increment.
     *
     * @note The base class returns Transform::Identity() by default.
     */
    virtual Transform predict(const Transform& increment);

    /**
     * @brief Allows the derived class to perform some pre-processing
     * before the actual matching.
     *
     * @note The base class does nothing by default.
     */
    virtual void preProcessing();

    /**
     * @brief getIncrementPriorInKeyFrame. Return the increment prior
     * in the last key-frame frame by using either the increment_in_base_prior_
     * set by user (if set) (default) or using predict().
     * @return The increment prior in the last key-frame frame.
     *
     * @see setIncrementPrior
     * @see predict
     */
    Transform getIncrementPriorInKeyFrame();

    /**
     * @brief getIncrementPriorInLaserFrame. Return the increment prior
     * taking into acount the last key-frame in the laser frame.
     * @return The increment prior in the last laser frame.
     *
     * @see getIncrementPriorInKeyFrame
     * @see setIncrementPrior
     * @see predict
     */
    Transform getIncrementPriorInLaserFrame();

    /**
     * @brief posePlusIncrement. Update the estimated current pose
     * with the evaluated increment if process() succeeded.
     * @param processed.
     */
    void posePlusIncrement(const bool processed);

    /**
     * @brief Allows the derived class to perform some post-processing
     * after the actual matching.
     *
     * @note The base class does nothing by default.
     */
    virtual void postProcessing();

    /**
     * @brief Evaluate if the current reading should become the new
     * referent reading for the matching.
     * @param[in] The estimated pose increment in the base_frame_ frame.
     * @return Whether the currently evaluated reading is the new referent.
     *
     * @note The base class returns \b true by default.
     */
    virtual bool isKeyFrame(const Transform& increment);

    /**
     * @brief Allows the derived class to perform some actions if the
     * currently evaluated reading is the new referent.
     */
    virtual void isKeyFrame();

    /**
     * @brief Allows the derived class to perform some actions if the
     * currently evaluated reading is \b NOT the new referent.
     */
    virtual void isNotKeyFrame();

    /**
     * @brief resetDefaultCovariance. Reset Covariances
     * to their default values. e.g. on key-frame creation.
     */
    virtual void resetCovarianceDefault();

    /**
     * @brief Fills the message with the estimated pose.
     */
    template <typename T>
    void fillMsg(T&& msg_ptr);

    /**
     * @brief Fills the message with the estimated pose increment.
     */
    template <typename T>
    void fillIncrementMsg(T&& msg_ptr);

    /**
     * @brief Fills the messages with both the pose
     * and the estimated pose increment.
     */
    template <typename PoseMsgT, typename IncrementMsgT>
    void fillMsgs(PoseMsgT&& pose_msg_ptr, IncrementMsgT&& increment_msg_ptr)
    {
      fillMsg(std::forward<PoseMsgT>(pose_msg_ptr));
      fillIncrementMsg(std::forward<IncrementMsgT>(increment_msg_ptr));
    }
  };

  /// @brief A base-class pointer.
  typedef boost::shared_ptr<LaserOdometryBase> LaserOdometryPtr;

} /* namespace laser_odometry */

#include <laser_odometry_core/laser_odometry_report.h>

namespace laser_odometry {

template <typename PoseMsgT, typename IncrementMsgT>
LaserOdometryBase::ProcessReport
LaserOdometryBase::process(const sensor_msgs::LaserScanConstPtr& scan_msg,
                           PoseMsgT&& pose_msg,
                           IncrementMsgT&& pose_increment_msg)
{
  const auto report = process(scan_msg);

  fillMsgs(std::forward<PoseMsgT>(pose_msg),
           std::forward<IncrementMsgT>(pose_increment_msg));

  return report;
}

template <typename PoseMsgT, typename IncrementMsgT>
LaserOdometryBase::ProcessReport
LaserOdometryBase::process(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                           PoseMsgT&& pose_msg,
                           IncrementMsgT&& pose_increment_msg)
{
  const auto report = process(cloud_msg);

  fillMsgs(std::forward<PoseMsgT>(pose_msg),
           std::forward<IncrementMsgT>(pose_increment_msg));

  return report;
}

} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_CORE_LASER_ODOMETRY_BASE_H_ */
