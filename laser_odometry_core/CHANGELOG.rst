^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package laser_odometry_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* add fillIncrementMsg
* fix msg_ptr z oups
* rename twist_cov -> inc_cov
* rename correction\_ -> increment\_ & :lipstick:
* fix macro
* fix macro
* Contributors: Jeremie Deray

0.0.2 (2017-05-23)
------------------
* missing deps
* typo & clearer var name
* Contributors: Jeremie Deray

0.0.1 (2017-05-23)
------------------
* LICENSE pkg.xml
* cov is twist cov, comment pose cov
* :lipstick:
* replace kword world->fixed
* base rm usuless bool
* node doc
* ProcessReport doc
* :lipstick:
* add twist_covariance\_
* delete usuless func
* base doc
* readme
* fix topic in and default world = map
* reduce default getTf duration
* fix warning unused
* core add getKeyFrame
* add hasNewKeyFrame
* Base:process no more virtual
* core install rule
* add isNotKeyFrame & fixes
* base::odomType do not throw but warn and return unknown
* reset correction
* isKeyFrame(correction\_) && confImpl returns true && doc
* getTf do not set tf if it couldn't get it
* add isKeyFrame() && comments && :lipstick:
* wip toward Base handling common comp
* message filling made template
* wip toward Base handling the common comp
* add pre/postProcessing
* use Covariance type rather than vector
* fix warning
* add isKeyFrame in base & func reordering
* mv Base class decl to base.h -> core.h simple includer
* process returns ProcessReport rather than bool
* add ProcessReport pimpl
* use fillOdomMsg & fix child_frame_id
* return by const ref
* add odomType
* getTf add opt duration & fix err msg
* use proper singleton scheme
* rename instantiater
* getTf with optional stamp
* fix core build
* fillOdomMsg uses current_time\_ & add expressFromLaserToBase
* rm reference_scan & :lipstick:
* rm main node from core
* core cmake
* process() aint pur virtual anymore but throws & add process(pcl2)
* node init origin
* rename get/set FrameWorld -> FrameFixed
* fix getTf uninitialized
* core CMake add node
* add node launch file
* core add Node class
* replace class LaserOdometry -> LaserOdometryInstantiater
* core getter/setter & fillPose*Msg & odom frame = odom
* one-liner tfFromXYTheta
* fix tf pub
* yup changed core api again
* utils getTf from msg
* comments
* tf init I
* :lipstick:
* core api retrieve relative_tf & setInitialGuess
* new process api & mv common stuff to base class
* getTf setIdentity
* core friend loader
* core add loader/holder class
* laser_odometry_core fix deps
* core lib
* Base configure & predict & broadcast
* add utils
* _core fix constructor & process(PoseWithCovarianceStampedPtr)
* upload laser_odometry_core
* Contributors: Jeremie Deray, artivis
