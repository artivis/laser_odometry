# laser_odometry : A [`pluginlib`](http://wiki.ros.org/pluginlib)-based package for **laser-scan matching**.
---

Aims at providing a single, simple interface for different scan matcher algorithms.

With a single node to start, selecting an scan-matching algorithm is as simple as setting a parameter in a launch file !

Available plugins:

-  [laser_odometry_csm](https://github.com/artivis/laser_odometry_csm)
-  [laser_odometry_libpointmatcher](https://github.com/artivis/laser_odometry_libpointmatcher)
-  [laser_odometry_gpar](https://github.com/artivis/laser_odometry_gpar)

To develop your own plugin, please refer to [the plugin instructions](laser_odometry_core/README.md).
  
#### *@todo*:

- [ ] documentation
- [x] publish key-frames
