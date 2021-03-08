# topological-montecarlo-localization

This metapackage contains most of the development for localization of the SIAR platform inside a sewer network. 

This localization system has been published in the Sensor journal in the submission entitle "A robust localization system for inspection robots in sewer networks ", by D. Alejo, F. Caballero and L. Merino. https://www.mdpi.com/1424-8220/19/22/4946

## Running the experiments

We include the ROS package "sensor_sims" that contains the scripts for automatically reproduce the results presented in the paper.

## Composition

It is composed by the following packages:

* *amcl_sewer* A customized version of the AMCL algorithm adapted to the particularities of the sewer network.
* *manhole_detector* A keras-based CNN detector trained with over 45k images for detecting manholes with the up-facing camera of the robot
* *sensor_sims* Contains scripts for automatically obtaining the results presented in the submitted sensor paper.
* *sewer_graph* Uses the simple_graph library to acquire the sewer network topology from file and store it in memory
* *simple_graph* A lightweight library for storing sparse graphs
* *wall_detector* A module for estimating the relative orientation of the robot with respect to a sewer gallery.

For more details about these packages, please refer to their respective internal README.md.

## Compilation

A rosinstall file is included for installing the required dependencies (see dependencies section).

In order to build the package, clone it inside the *src* directory of your Catkin workspace and compile it by using *catkin_make* as usual.

## Dependencies

- libgeotranz3  (tested in version 3.3)
- libkml-1.2.0 (or higher)
- Rgbd-odom package. Can be downloaded from: https://github.com/robotics-upo/rgbd_odom.git
- Keras python library (https://keras.io/) for neural networks. *Please refer to the README.md of manhole_detector package* for its correct configuration.
- Function ROS package. https://github.com/robotics-upo/functions
- Plane detector package. https://github.com/robotics-upo/plane_detector
