# laser_odometry

## A [`pluginlib`](http://wiki.ros.org/pluginlib)-based package for **laser-scan matching**

---

[![GHA][badge-ci-img]][badge-ci]

## Package Summary

A unified Laser Odometry scheme which aims at providing a single, simple interface for different scan matcher algorithms.
With a single node to start, selecting an scan-matching algorithm is as simple as setting a parameter in a launch file !

- Maintainer status: maintained
- Maintainer: Jeremie Deray <deray.jeremie@gmail.com>
- Author: Jeremie Deray <deray.jeremie@gmail.com>
- License: Apache-2.0
- Bug / feature tracker: https://github.com/artivis/laser_odometry/issues
- Source: git https://github.com/artivis/laser_odometry.git (branch: master)

---

## Plugins

Available plugins:

- [laser_odometry_csm](https://github.com/artivis/laser_odometry_csm) A plugin for [csm](https://github.com/AndreaCensi/csm)
- [laser_odometry_libpointmatcher](https://github.com/artivis/laser_odometry_libpointmatcher) A plugin for [libpointmatcher](https://github.com/ethz-asl/libpointmatcher)
- [laser_odometry_polar](https://github.com/artivis/laser_odometry_polar) A plugin for the [polar scan matcher](https://github.com/ccny-ros-pkg/scan_tools/tree/indigo/polar_scan_matcher)
- [laser_odometry_rf2o](https://github.com/artivis/laser_odometry_rf2o) A plugin for the [rf2o scan matcher](https://github.com/artivis/rf2o_laser_odometry)
- [laser_odometry_srf](https://github.com/artivis/laser_odometry_srf) A plugin for the [srf scan matcher](https://github.com/artivis/srf_laser_odometry)

## Notes

- This package is still under heavy developement thus the API is **not** completely stable yet.
- For ROS distro pre-Kinetic, to properly visualize the `nav_msgs/Odometry` message published by the node with `Rviz` (that is, with its covariance), it is recommended to use the [rviz_plugin_covariance](https://github.com/laas/rviz_plugin_covariance) (E.g. [wiki Screenshot](https://github.com/artivis/laser_odometry/wiki/Screenshot)).

## Documentation

- The Doxygen documention is available online at [codedocs.xyz/artivis/laser_odometry](https://codedocs.xyz/artivis/laser_odometry/index.html).
- Some notes about the node topics and parameters can be found in the [Parameters & Topics](https://github.com/artivis/laser_odometry/wiki/Parameters-and-topics) wiki page.
- The overall execution scheme is summarized in the [pseudo-code](https://github.com/artivis/laser_odometry/wiki/Overall-execution-pseudo-code) wiki page.
- To develop your own plugin, please refer to [the plugin instructions](https://github.com/artivis/laser_odometry/wiki/Create-a-plugin) wiki page.

[//]: # (URLs)

[badge-ci-img]: https://github.com/artivis/laser_odometry/workflows/ci/badge.svg?branch=master
[badge-ci]: https://github.com/artivis/laser_odometry/workflows/ci/badge.svg?branch=master