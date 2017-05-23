^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package laser_odometry
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.2 (2017-05-23)
------------------

0.0.1 (2017-05-23)
------------------
* added metapkg, renamed laser_odometry to laser_odometry_node
* node add missing global frame for origin init
* replace kword world->fixed
* node doc
* Fix installation rules
* fix origin frame retrieval
* fix topic in and default world = map
* reduce default getTf duration
* node default pub odom & not tf & topic_in
* laser_odometry add simple launch file and update conf file
* fix setLaserFromTf
* add node default yaml
* node pub kframe
* clean laser_odom launch
* laser_odometry install rule
* node add timer on process()
* add cheap throttling
* made publish template
* node :lipstick:
* use proper singleton scheme
* rename instantiater
* protect a bunch of functions
* throw if no laser odometry type specified
* setLaserFromTf with optional stamp
* add possibility of a not fixed sensor<->base frame
* fix node launch
* hacky easy sub to either laser_scan or pcl2 & fix include
* add laser_odometry to hold the main node
* Contributors: Jeremie Deray, Procópio Stein, artivis, davidfernandez
