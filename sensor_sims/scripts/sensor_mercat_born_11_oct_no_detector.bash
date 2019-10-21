#!/bin/bash
# $1 --> The first uav number
# $2 --> The last number of uavs
# $3 --> The path to the repository (resolution)
# $4 --> Number of tests to be generated in each folder
# $5 --> Base input file

orig_folder=$PWD

if [ $# -lt 3 ]; then
  echo "Usage: $0 <first_exe_number> <last_exe_number> <path_to_ros_setup_bash> [<path_to_dataset>]"
  exit 1
fi

DATE_BAG=2017-10-11-11-05-03

bag_file=$4/siar_${DATE_BAG}.bag
ground_file=$4/input_vector_DATE_BAG.txt
directory_out=$4/output/${DATE_BAG}/detector

source $3

# Initial Parameters_11_oct 
initial_x=-12.671
initial_y=-105.37
initial_a=-2.3
bag_file=/windows/Dataset/2017-10-11/siar_2017-10-11-11-05-03_filtered.bag
ground_file=/windows/Dataset/2017-10-11/input_vector_2017-10-11-11-05-03_ground_truth.txt
start=92
duration=2350
odom_a_mod=0.12
odom_a_noise=0.04
odom_x_mod=0.4


# # With yaw estimation --> yaw_estimator --> true
CONTADOR=$1
directory_out=/home/chur/Dataset/2017-10-11/no_estimation_no_vo
mkdir -p $directory_out
roscd amcl_sewer/launch
cp amcl_bag.launch $directory_out
cd ../scripts
cp $0 $directory_out
until [ $CONTADOR -gt $2 ]; do
  # Roslaunch with multiple parameters
  roslaunch amcl_sewer amcl_bag.launch play_bag:=false ground_truth:=$ground_file\
  ground_truth_out:=${directory_out}/stats_$CONTADOR.txt \
  trajectory_file:=${directory_out}/traj_$CONTADOR.txt \
  trajectory_file_python:=${directory_out}/traj_python_$CONTADOR.txt \
  yaw_estimator:=false inspection:=false \
  odom_a_mod:=$odom_a_mod odom_a_noise:=$odom_a_noise odom_x_mod:=$odom_x_mod odom_y_mod:=$odom_x_mod \
  min_particles:=300 max_particles:=400 \
  camera:=/up initial_x:=$initial_x initial_y:=$initial_y initial_a:=$initial_a rgbd_odom:=false &
  
  #end of roslaunch
  
  let pid1=$!
  rosbag play $bag_file -s $start --clock -r 1.5 -d 5 -u $duration
  rosnode kill -a
  wait ${pid1}
  let CONTADOR+=1
done


cd $orig_folder
