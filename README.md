# laser_odometry
## A [`pluginlib`](http://wiki.ros.org/pluginlib)-based package for **laser-scan matching**
---

## Package Summary
A unified Laser Odometry scheme which aims at providing a single, simple interface for different scan matcher algorithms.  
With a single node to start, selecting an scan-matching algorithm is as simple as setting a parameter in a launch file !

- Maintainer status: maintained
- Maintainer: Jeremie Deray <jeremie.deray@pal-robotics.com>
- Author: Jeremie Deray <jeremie.deray@pal-robotics.com>
- License: Apache-2.0
- Bug / feature tracker: https://github.com/artivis/laser_odometry/issues
- Source: git https://github.com/artivis/laser_odometry.git (branch: master)

<!-- [![Build Status](https://travis-ci.org/artivis/laser_odometry.svg?branch=master)](https://travis-ci.org/artivis/laser_odometry) -->
---
## Build Summary
| Indigo            | Jade              | kinetic            |
|-------------------|-------------------|--------------------|
| [![Indigo][1]][5] | [![Jade][2]][5]   | [![kinetic][3]][5] |

[1]: https://travis-matrix-badges.herokuapp.com/repos/artivis/laser_odometry/branches/master/1
[2]: https://travis-matrix-badges.herokuapp.com/repos/artivis/laser_odometry/branches/master/4
[3]: https://travis-matrix-badges.herokuapp.com/repos/artivis/laser_odometry/branches/master/7
[5]: https://travis-ci.org/artivis/laser_odometry

## Plugins
Available plugins:

-   [laser_odometry_csm](https://github.com/artivis/laser_odometry_csm) A plugin for [csm](https://github.com/AndreaCensi/csm)
-   [laser_odometry_libpointmatcher](https://github.com/artivis/laser_odometry_libpointmatcher) A plugin for [libpointmatcher](https://github.com/ethz-asl/libpointmatcher)
-   [laser_odometry_polar](https://github.com/artivis/laser_odometry_polar) A plugin for the [polar scan matcher](https://github.com/ccny-ros-pkg/scan_tools/tree/indigo/polar_scan_matcher)
-   [laser_odometry_rf2o](https://github.com/artivis/laser_odometry_rf2o) A plugin for the [rf2o scan matcher](https://github.com/artivis/rf2o_laser_odometry)

Under development plugins:

-   None at this time.

<!-- -  [laser_odometry_gpar](https://github.com/artivis/laser_odometry_gpar) -->

## Notes
- This package is still under heavy developement thus the API is **not** completely stable yet.
- To properly visualize the `nav_msgs/Odometry` message published by the node with `Rviz`, it is recommended to use the [rviz_plugin_covariance](https://github.com/laas/rviz_plugin_covariance) (E.g. [wiki Screenshot](https://github.com/artivis/laser_odometry/wiki/Screenshot)).

## Documentation
The Doxygen documention is available online at [codedocs.xyz/artivis/laser_odometry](https://codedocs.xyz/artivis/laser_odometry/index.html).  
The overall execution scheme is summarized in the [pseudo-code](https://github.com/artivis/laser_odometry/wiki/Overall-execution-pseudo-code) wiki page.  
To develop your own plugin, please refer to [the plugin instructions](https://github.com/artivis/laser_odometry/wiki/Create-a-plugin) wiki page.

## Upcoming
- More plugins, mostly correlation-based & feature-based scan matching.
- A comparison framework to easily compare plugins.
