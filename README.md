# localization_siar

This metapackage contains most of the development for localization of the SIAR platform inside a sewer network. 

This localization system is presented in the paper "RGBD-based Robot Localization in Sewer Networks" of D. Alejo, F. Caballero and L. Merino that has been accepted for publication in the 2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2017).

The video attachment to the paper in HD can be found at:

-  https://youtu.be/N_tUhdBN7Z4

The bags for reproducing the presented experiments can be found at:

- *Experiment 1*: http://robotics.upo.es/~daletei/SIAR/Dataset/2017-01-17_sewers_jan/siar_2017-01-17-10-25-40.bag
- *Experiment 2*: http://robotics.upo.es/~daletei/SIAR/Dataset/2017-01-17_sewers_jan/siar_2017-01-17-11-17-28.bag


It is composed by the following packages:

* *amcl_sewer* A customized version of the popular AMCL algorithm adapted to the particularities of the sewer network.
* *functions* Some general-purpose functions and classes regarding to file management, handling with RealVectors and many more
* *manhole_detector* A keras-based CNN detector trained with over 19k images for detecting manholes with the up-facing camera of the robot
* *sewer_graph* Uses the simple_graph library to acquire the sewer network topology from file and store it in memory
* *simple_graph* A lightweight library for storing sparse graphs

For more details about these packages, please refer to their internal README.md.

## Compilation

Before compiling, please check the dependencies

In order to build the package, clone it inside the *src* directory of your Catkin workspace and compile it by using *catkin_make* as usual.

## Dependencies

- libgeotranz3  (tested in version 3.3)
- libkml-1.2.0 (or higher)
- Rgbd-odom package. Can be downloaded from: https://github.com/robotics-upo/rgbd_odom.git
- Keras python library (https://keras.io/) for neural networks. *Please refer to the README.md of manhole_detector package* for its correct configuration.

