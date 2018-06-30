# manhole_detector

This package contains scripts for CNN detecting manhole, faking the detector according to sequences of images that are below a sewer (hand labeled)
and generating data for training the CNN according to the handmade labels.

## Dependencies

It makes use of the keras python library for using neural networks for detection. Please refer to https://keras.io/ for installation. 

It requires the hdf5 and h5py for loading the trained network

## Configuring Keras

Keras should be configured with the following parameters. Please edit "~/.keras/keras.json" file.

- "backend": "theano"
- "image_dim_ordering": "th"

## cnn_detector.py

This node uses the CNN for generating a bool message that will be true whenever a manhole is detected

### Use:

 cnn_detector.py \<camera name\> \<cnn_file\>
 
 The cnn_file is saved in the test subdirectory. The camera name is usually "/up"
 
### Published topics

* */manhole* (std_msgs/Bool) Indicates whether SIAR is below a manhole or not
* */manhole_marker* (visualization_msgs/Marker) A marker that will be published if the manhole is detected

### Subscribed topics

* */{camera_name}/depth_registered/image_raw" (sensor_msgs/Image) THe input depth image that is used as input of the CNN 

 
## fake_detector.py

This node loads the handmade label file that identifies the sequence number of the images that are below a manhole and uses it to generate
a bool message indicating it.

In the subdirectory test/ you can find the label files used in Experiments 1 & 2. V2 of the input label files are shortened and give best results. Groundtruth files are even shorter with instants in which the robot is just under the manhole. These are used for generating the localization statistics of the paper "RGBD-based Robot Localization in Sewer Networks" of D. Alejo, F. Caballero and L. Merino that has been accepted for publication in the 2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2017).


### Use:
 fake_detector.py \<camera name\> \<label_file\>
 
### Published topics

* */manhole* (std_msgs/Bool) Indicates whether SIAR is below a manhole or not

### Subscribed topics

* */{camera_name}/depth_registered/image_raw" (sensor_msgs/Image) THe input depth image for checking its sequence number

## generate_dataset_learning

Loads a bag file and generates four data files with downsampled depth (80x60) images. A label file is necessary in order to distinguish images under manhole and images without manhole:

1 - "positive_depth": All depth images that are below a manhole according to the label file. It will include each positive image and its 180º deg rotation in order to enlarge the test data set and make it invariant to the track direction.

2 - "positive_rgb: All RGB images that are below a manhole according to the label file. It will include each positive image and its 180º deg rotation in order to enlarge the test data set and make it invariant to the track direction.

3 - "negative_depth": All depth images that are not below a manhole according to the label file

4 - "negative_rgb": All RGB images that are not below a manhole according to the label file

### Use:

 * *generate_dataset_learning*  \<bag file\> \<label_file\> \[\<camera_name\>\] \[\<skip the starting n images\>\] 
 
 (camera_name defaults to "/up")
