# amcl_sewer

This module has been included for performing the localization of the SIAR platform while navigating through the sewer network. 

It uses Adaptive Monte-Carlo Localization with two different weights. 

First, it calculates the distance of the particle with the graph (i.e. distance to closest edge). Particles that are farther 
from the graph are less likely to represent the state of the SIAR.

Second, whenever the manhole detector detects a manhole the weights are changed to the distance to the closest manhole.
In this way, we can precisely locate the SIAR platform in the sewer network.

## Launching the experiments

In the "launch" subdirectory, two launch files are given for easilly executing the experiments described in the submitted paper:

- *amcl_experiment_1.launch*. By default, it searches for then bag of Experiment 1 in "\~/siar_2017-01-17-10-25-40.bag" (can be changed with the parameter bag_file) and produces the stats file: "\~/stats_experiment_1.txt". 

- *amcl_experiment_1.launch*. By default, it searches for then bag of Experiment 1 in "\~/siar_2017-01-17-11-17-28.bag" (can be changed with the parameter bag_file) and produces the stats file: "\~/stats_experiment_2.txt". 

## amcl_node

This node executes the localization. 

The parameters are (default values given inside parentheses):



### ROS Parameters

* *base_frame_id*: (base_link) The name of the base link of the platform
* *odom_frame_id*: (odom) The reference frame of the odometry
* *global_frame_id*: (map) Global reference frame of the AMCL
* *update_rate*: (10) Update rate of the AMCL 
* *min_particles*: (300) Minimum size of the particle set
* *max_particles*: (600) Maximum size of the particle set
* *odom_x_mod*: (0.2) Odometry noise in x coord
* *odom_y_mod*: (0.2) Odometry noise in y coord
* *odom_a_mod*: (0.2) Odometry noise in yaw
* *initial_x*: (0.0) Initial position of the robot (x)
* *initial_y*: (0.0) Initial position of the robot (y)
* *initial_a*: (0.0) Initial position of the robot (yaw)
* *initial_x_dev*: (0.3) Std_dev of the initial position of the robot (x)
* *initial_y_dev*: (0.3) Std_dev of the initial position of the robot (y)
* *initial_a_dev*: (0.2) Std_dev of the initial position of the robot (yaw)
* *initialize*: (false) If true, automatically initializes the filter with the provided initial data
* *update_min_d*: (0.5) Minimum displacement for performing prediction + update
* *update_min_a*: (0.2) Minimum angular displacement for performing prediction + update
* *resample_interval*: (0) Number (-1) of updates before resampling
* *manhole_detect_topic*: (/manhole) Topic (type Bool) that is true if a manhole is detected
* *sewer_graph_file*: (...) Location of the Sewer Graph File (see example in test subdirectory of sewer_graph)
* *min_manhole_detected*: (10) Minimum number of detections before an update for changing to Manhole weighting
* *edgeDev* (1.0) Std_dev from the sewer_graph
* *forkDev* (3.0) Std_dev from the sewer_graph in non-straight sections
* *fork_dist* (3.0) If a fork node is closer than this distance from the mean position of the particle set, we consider the SIAR inside a fork
* *manholeDev* (0.6) Std_dev for the manhole weighting
* *manholeThres* (0.15) Additive weight for manhole weighting
* *stats_file* (~/manhole_stats.txt) Saves statistics whenever passing below a manhole (which has to be manually labeled)
* *traj_file* (~/traj_.txt) Saves the mean position of the particle set in after each prediction or update steps



### Published topics

* */amcl_node/particle_cloud* (geometry_msgs/PoseArray) pose of the particles
* */amcl_node/sewer_graph* (visualization_msgs/Marker) Sewer Graph Visualization
* */gps/fix* (sensor_msgs/NatSatFix) Location of the (0,0) local position in global coordinates (ENU transform between local and global)


### Subscribed topics

* *(manhole_detect_topic)* (std_msgs/Bool) Output of the manhole detector module (if true --> SIAR is below a manhole)
* */ground_truth* (std_msgs/Bool) If true we are really below a manhole (hand labeled)
* */amcl_node/initial_pose* (geometry_msgs/PoseWithCovarianceStampedConstPtr)

