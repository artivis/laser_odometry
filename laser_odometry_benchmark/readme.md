**Laser Odometry Comparison**

The laser_odometry package uses laser scans in order to find out which is the position of the robot. It implements four kind of laser odometries:
1. Canonical Scan Matcher (CSM);
2. Iterative Closest Point library (LibPointMatcher);
3. Polar Scan Matcher;
4. RF2O Scan Mathcer.
The differences among such methods reside in how the algorithms compute the robot pose. We can see an example of trajectories after a simple navigation task:

![traj](/images/traj-crop.png)

From this picture seems like RF2O is quite good and also CSM has a good but not perfect performance. 
A bunch of pictures is not enough to estimate the accuracy of the laser odometry implementations, therefore, in order to establish which is the best, we need to define a proper evaluation method:
A central role is played by the ground truth, which has to be the best and more thrustworthy localization algorithm possible. 
Every approach has its own perks and flaws: one might perform better in closed and narrow places, another one might be perfect for open-space environment or for obstacle with particular shape.
Everytime there is an error with respect to the real pose, such error accumulates over time, bringing the position to diverge completely after few meters of walk.
About the first consideration, in a real environment we can employ objects like motion controllers to track efficiently the robot position; instead, during simulations, we can simply rely on the Gazebo Model State pose.
For what’s concerning the other two points, it would be appropriate to divide the robot trajectory in parts, and evaluate every part calculating the offset from the current pose to the last saved one. In this way, no error accumulates over time with the risk of diverging and every time we change environment during an experiment, the history of the odometry will not influence the ongoing measurements in the new situation.
Finally, a good evaluation method has also a good scoring method. We chose RMSE, i.e. Root Mean Squared Error, because it tells us how concentrated is the data around the optimal values, represented by the ground truth localization. 
These are the ideas behind the package, and now we will see details about the code implementation.

Initially, from the launch file it is possible to set these set of variables:
- topic_in: the name of the input scans topic;
- ground_truth_topic: the name of the topic that publishes the ground truth;
- timeout: interval of time in seconds between two calculations of the RMSE;
- plugins: the list of the laser odometry plugins you want to use;
- gazebo: set to true, it abilitates the launch of a node that publishes the Gazebo Model State pose of the robot as ground truth pose.

The launch file in the package laser_odometry_benchmark launches the comparison_node, where all the requested odometries and the support variables are properly initialized. 
Thanks to the time-stamps of the odometry messages (every time a new scan is published on the topic_in topic, the robot pose is updated and published) is possible to determine when timeout is over and measure the offsets from the last saved poses contained in the variable new_origins_. Subsequently, the obtained value is published and the new_origins_ variable is updated with the current poses. In the following image, it’s possible to see a graph of the RMSE error after the navigation executed above, plotted on PlotJuggler:

![rmse plot](/images/plot-rmse.png)

We can see now another example of trajectories after a slighlty more complex navigation: 

![other trajs](/images/other-traj-crop.png)
