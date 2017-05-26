# laser_odometry : A [`pluginlib`](http://wiki.ros.org/pluginlib)-based package for **laser-scan matching**.
---

[![Build Status](https://travis-ci.org/artivis/laser_odometry.svg?branch=master)](https://travis-ci.org/artivis/laser_odometry)

Aims at providing a single, simple interface for different scan matcher algorithms.

With a single node to start, selecting an scan-matching algorithm is as simple as setting a parameter in a launch file !

Available plugins:

-  [laser_odometry_csm](https://github.com/artivis/laser_odometry_csm) A plugin for [csm](https://github.com/AndreaCensi/csm)

Under development plugins:

- `laser_odometry_libpointmatcher` A plugin for [libpointmatcher](https://github.com/ethz-asl/libpointmatcher)
- `laser_odometr_polar` A plugin for the [polar scan matcher](https://github.com/ccny-ros-pkg/scan_tools/tree/indigo/polar_scan_matcher)
- `laser_odometry_rf2o` A plugin for the [rf2o odometry](http://wiki.ros.org/rf2o_laser_odometry)

<!-- -  [laser_odometry_libpointmatcher](https://github.com/artivis/laser_odometry_libpointmatcher) -->
<!-- -  [laser_odometry_gpar](https://github.com/artivis/laser_odometry_gpar) -->
<!-- -  [laser_odometr_polar](https://github.com/artivis/laser_odometr_polar) -->
<!-- -  [laser_odometr_rf2o](https://github.com/artivis/laser_odometr_rf2o) -->

To develop your own plugin, please refer to [the plugin instructions](https://github.com/artivis/laser_odometry/wiki/Create-a-plugin).
