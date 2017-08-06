#ifndef _LASER_ODOMETRY_CORE_LASER_ODOMETRY_BASE_H_
#define _LASER_ODOMETRY_CORE_LASER_ODOMETRY_BASE_H_

// The input ROS messages supported
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

// The output ROS messages supported
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
//#include <geometry_msgs/PoseWithCovarianceStamped.h>

// More ROS header
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

namespace laser_odometry
{
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
   *       - predict                                [O]
   *
   *       - process_impl                           [X]
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

    /// @brief The covariance message type.
    /// @see geometry_msgs::PoseWithCovariance::_covariance_type.
    using covariance_msg_t = geometry_msgs::PoseWithCovariance::_covariance_type;

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
     * @brief Compute the 2D odometry given a LaserScan.
     * @param[in] scan_msg. The input LaserScan.
     * @param[out] pose_msg. The estimated 2D pose.
     * @param[out] pose_increment_msg. The estimated 2D pose increment.
     * @return ProcessReport. A brief summary of the scan matching process.
     *
     * @see ProcessReport
     */
    ProcessReport process(const sensor_msgs::LaserScanConstPtr& scan_msg,
                          geometry_msgs::Pose2DPtr pose_msg,
                          geometry_msgs::Pose2DPtr pose_increment_msg = nullptr);

    /**
     * @brief Compute the 3D (actually 2D) odometry given a LaserScan.
     * @param[in] scan_msg. The input LaserScan.
     * @param[out] odom_msg. The estimated 3D odometry (actually 2D).
     * @param[out] odom_increment_msg. The estimated 3D odometry increment.
     * @return ProcessReport. A brief summary of the scan matching process.
     *
     * @see ProcessReport
     */
    ProcessReport process(const sensor_msgs::LaserScanConstPtr& scan_msg,
                          nav_msgs::OdometryPtr odom_msg,
                          nav_msgs::OdometryPtr odom_increment_msg = nullptr);

    /**
     * @brief Compute the 2D odometry given a PointCloud2.
     * @param[in] cloud_msg. The input PointCloud2.
     * @param[out] pose_msg. The estimated 2D pose.
     * @param[out] pose_increment_msg. The estimated 2D pose increment.
     * @return ProcessReport. A brief summary of the pointcloud matching process.
     *
     * @see ProcessReport
     */
    ProcessReport process(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                          geometry_msgs::Pose2DPtr pose_msg,
                          geometry_msgs::Pose2DPtr pose_increment_msg = nullptr);

    /**
     * @brief Compute the 3D odometry given a \c PointCloud2.
     * @param[in] cloud_msg. The input \c PointCloud2.
     * @param[out] pose_msg. The estimated 3D odometry.
     * @param[out] pose_increment_msg. The estimated 3D odometry increment.
     * @return ProcessReport. A brief summary of the pointcloud matching process.
     *
     * @see ProcessReport
     */
    ProcessReport process(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                          nav_msgs::OdometryPtr odom_msg,
                          nav_msgs::OdometryPtr odom_increment_msg = nullptr);

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
    virtual bool process_impl(const sensor_msgs::LaserScanConstPtr& laser_msg,
                              const tf::Transform& prediction);

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
    virtual bool process_impl(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                              const tf::Transform& prediction);

  public:

    /**
     * @brief Return the current estimated pose.
     * @return A const-reference tf::Transform.
     */
    const tf::Transform& getEstimatedPose() const noexcept;

    /**
     * @brief Reset the matcher.
     * The base class implemetation resets all transforms to
     * Identity and the key-reading to nullptr.
     */
    virtual void reset();

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

    /// \brief Reference to the origin frame.
    /// \return Reference to the origin frame.
    tf::Transform& getOrigin();
    /// \return Const-reference to the origin frame.
    const tf::Transform& getOrigin() const;

    /**
     * @brief Set the origin frame.
     * @param[in] origin. The origin frame.
     */
    void setOrigin(const tf::Transform& origin);

    /// \brief Reference to the initial prediction transform of the upcoming matching.
    /// \return Reference to the initial prediction transform of the upcoming matching.
    tf::Transform& getInitialGuess();

    /// \brief Const-reference to the initial prediction transform of the upcoming matching.
    /// \return Const-reference to the initial prediction transform of the upcoming matching.
    const tf::Transform& getInitialGuess() const;

    /**
     * @brief Set the initial prediction of the upcoming matching.
     * @param[in] guess. The initial prediciton.
     */
    void setInitialGuess(const tf::Transform& guess);

    /// \brief Reference to the laser pose wrt the robot base frame.
    /// \return Reference to the laser pose wrt the robot base frame.
    tf::Transform& getLaserPose();

    /// \brief Const-reference to the laser pose wrt the robot base frame.
    /// \return Const-reference to the laser pose wrt the robot base frame.
    const tf::Transform& getLaserPose() const;

    /**
     * @brief Set the laser pose wrt the robot base frame.
     * @param[in] base_to_laser.
     */
    void setLaserPose(const tf::Transform& base_to_laser);

    /// \brief the robot base frame name.
    /// \return the robot base frame name.
    const std::string& getFrameBase()  const noexcept;

    /// \brief the robot laser frame name.
    /// \return the robot laser frame name.
    const std::string& getFrameLaser() const noexcept;

    /// \brief the global fixed frame name.
    /// \return the global fixed frame name.
    const std::string& getFrameFixed() const noexcept;

    /// \brief the global odometry frame name.
    /// \return the global odometry frame name.
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

    /// @brief The type of odometry computed by the plugin.
    /// @return The type of odometry computed by the plugin.
    /// If not overrided, default is OdomType::Unknown
    /// @see OdomType
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

    bool configured_   = false; /*!< @brief Whether the matcher is configured. */
    bool initialized_  = false; /*!< @brief Whether the matcher is initialized. */
    bool has_new_kf_   = false; /*!< @brief Whether the matcher has a new referent reading. */

    covariance_msg_t pose_covariance_;  /*!< @brief The estimated pose covariance. */
    covariance_msg_t increment_covariance_; /*!< @brief The estimated pose increment covariance. */

    ros::NodeHandle private_nh_ = ros::NodeHandle("~");

    std::string base_frame_       = "base_link";        /*!< @brief The robot base frame name. */
    std::string laser_frame_      = "base_laser_link";  /*!< @brief The robot laser frame name. */
    std::string fixed_frame_      = "map";              /*!< @brief The global fixed frame name. */

    /// \brief The global odometry frame name.
    /// This frame name is only used in the published message.
    std::string laser_odom_frame_ = "odom";

    /// \brief Tranform from base_frame to laser_frame
    tf::Transform base_to_laser_;

    /// \brief Tranform from laser_frame to base_frame
    /// == base_to_laser_^-1
    tf::Transform laser_to_base_;

    /// @brief The relative transform in the laser_frame.
    /// @note This is the transform the derived class should fills.
    tf::Transform increment_;

    /// \brief The relative transform in the base_frame.
    tf::Transform relative_tf_;

    /// \brief Guessed/predicted tranform
    /// from reference_'reading' to
    /// current_'reading' in the base_frame.
    tf::Transform guess_relative_tf_;

    /// \brief Tranform from fixed_frame
    /// to base_frame, where fixed_frame
    /// is the origin of the integration.
    /// == fixed_to_base_kf_ * relative_tf_.
    tf::Transform fixed_to_base_;

    /// \brief Tranform from fixed_frame to
    /// the last keyfame frame.
    /// == fixed_to_base * relative_tf_.
    tf::Transform fixed_to_base_kf_;

    /// \brief An optional user defined
    /// transform from that maps the
    /// integration origin to another
    /// reference than Identity
    /// Default: Identity
    tf::Transform fixed_origin_;

    /// \brief Tranform from the fixed_origin frame
    /// to the base_frame.
    /// It is the integrated robot odometry.
    /// == fixed_origin_ * fixed_to_base_.
    tf::Transform fixed_origin_to_base_;

    /// \brief  The referent LaserScan.
    sensor_msgs::LaserScanConstPtr   reference_scan_;

    /// \brief  The referent PointCloud2.
    sensor_msgs::PointCloud2ConstPtr reference_cloud_;

    /// \brief Current ros::Time accordingly
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
     * @note The base class returns tf::Transform::getIdentity() by default.
     */
    virtual tf::Transform predict(const tf::Transform& increment);

    /**
     * @brief Allows the derived class to perform some pre-processing
     * before the actual matching.
     *
     * @note The base class does nothing by default.
     */
    virtual void preProcessing();

    /**
     * @brief Allows the derived class to perform some post-processing
     * after the actual matching.
     *
     * @note The base class does nothing by default.
     */
    virtual void postProcessing();

    /**
     * @brief Evaluate if the current reading should become the new
     * referent reading form the matching.
     * @param[in] The estimated pose increment in the world_frame_ frame.
     * @return Whether the currently evaluated reading is the new referent.
     *
     * @note The base class returns \b true by default.
     */
    virtual bool isKeyFrame(const tf::Transform& increment);

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
     * @brief Fills the published message with the estimated pose increment.
     */
    template <typename T>
    void fillMsg(T& msg_ptr);

    /**
     * @brief Fills the published message with the estimated pose increment.
     */
    template <typename T>
    void fillIncrementMsg(T& msg_ptr);
  };

  /// @brief A base-class pointer.
  typedef boost::shared_ptr<LaserOdometryBase> LaserOdometryPtr;

  template <>
  inline void LaserOdometryBase::fillMsg<geometry_msgs::Pose2DPtr>(geometry_msgs::Pose2DPtr& msg_ptr)
  {
    if (msg_ptr == nullptr) return;

    msg_ptr->x = fixed_origin_to_base_.getOrigin().getX();
    msg_ptr->y = fixed_origin_to_base_.getOrigin().getY();
    msg_ptr->theta = tf::getYaw(fixed_origin_to_base_.getRotation());
  }

  template <>
  inline void LaserOdometryBase::fillMsg<nav_msgs::OdometryPtr>(nav_msgs::OdometryPtr& msg_ptr)
  {
    if (msg_ptr == nullptr) return;

    msg_ptr->header.stamp    = current_time_;
    msg_ptr->header.frame_id = laser_odom_frame_;
    msg_ptr->child_frame_id  = base_frame_;

    msg_ptr->pose.pose.position.x = fixed_origin_to_base_.getOrigin().getX();
    msg_ptr->pose.pose.position.y = fixed_origin_to_base_.getOrigin().getY();
    msg_ptr->pose.pose.position.z = fixed_origin_to_base_.getOrigin().getZ();

    tf::quaternionTFToMsg(fixed_origin_to_base_.getRotation(),
                          msg_ptr->pose.pose.orientation);

    //msg_ptr->pose.covariance  = pose_covariance_;
    //msg_ptr->twist.covariance = pose_twist_covariance_;
  }

  template <>
  inline void LaserOdometryBase::fillIncrementMsg<geometry_msgs::Pose2DPtr>(geometry_msgs::Pose2DPtr& msg_ptr)
  {
    if (msg_ptr == nullptr) return;

    msg_ptr->x = increment_.getOrigin().getX();
    msg_ptr->y = increment_.getOrigin().getY();
    msg_ptr->theta = tf::getYaw(increment_.getRotation());
  }

  template <>
  inline void LaserOdometryBase::fillIncrementMsg<nav_msgs::OdometryPtr>(nav_msgs::OdometryPtr& msg_ptr)
  {
    if (msg_ptr == nullptr) return;

    msg_ptr->header.stamp    = current_time_;
    msg_ptr->header.frame_id = "last_key_frame"; /// @todo this frame does not exist. Should it?
    msg_ptr->child_frame_id  = base_frame_;

    msg_ptr->pose.pose.position.x = increment_.getOrigin().getX();
    msg_ptr->pose.pose.position.y = increment_.getOrigin().getY();
    msg_ptr->pose.pose.position.z = increment_.getOrigin().getZ();

    tf::quaternionTFToMsg(increment_.getRotation(),
                          msg_ptr->pose.pose.orientation);

    msg_ptr->pose.covariance  = increment_covariance_;
    //msg_ptr->twist.covariance = increment_twist_covariance_;
  }

} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_CORE_LASER_ODOMETRY_BASE_H_ */
