#!/bin/bash
# $1 --> The first uav number
# $2 --> The last number of uavs
# $3 --> The path to the repository (resolution)
# $4 --> Number of tests to be generated in each folder
# $5 --> Base input file

CONTADOR=$1
orig_folder=$PWD

if [ $# -lt 3 ]; then
  echo "Usage: $0 <first_exe_number> <last_exe_number> <path_to_ros_setup_bash> [<path_to_dataset>]"
  exit 1
fi

DATE_BAG=2018-06-12-10-54-51

bag_file=$4/siar_${DATE_BAG}.bag
ground_file=$4/input_vector_DATE_BAG.txt
directory_out=$4/output/${DATE_BAG}/montecarlo

source $3

# Initial Parameters_11_oct 
initial_x=96.24 #Initial position (start 980)
initial_y=-182.72
initial_a=2.47
 
start=0
duration=2005
odom_a_mod=0.12
odom_a_noise=0.04
odom_x_mod=0.2

# # With yaw estimation --> yaw_estimator --> true
CONTADOR=$1

mkdir -p $directory_out
until [ $CONTADOR -gt $2 ]; do
  # Roslaunch with multiple parameters
  roslaunch amcl_sewer amcl_bag.launch play_bag:=false ground_truth:=$ground_file\
  ground_truth_out:=${directory_out}/stats_$CONTADOR.txt \
  trajectory_file:=${directory_out}/traj_$CONTADOR.txt \
  trajectory_file_python:=${directory_out}/traj_python_$CONTADOR.txt \
  yaw_estimator:=true inspection:=false angle_dev:=0.06 \
  emit_tf_base:=true base_frame_id:=base_link_vo odom_frame_id:=odom_vo \
  odom_a_mod:=$odom_a_mod odom_a_noise:=$odom_a_noise odom_x_mod:=$odom_x_mod odom_y_mod:=$odom_x_mod \
  min_particles:=300 max_particles:=400 \
  camera:=/up initial_x:=$initial_x initial_y:=$initial_y initial_a:=$initial_a rgbd_odom:=false \
  graph_file:=pedralbes_graph_reduced &
  
  #end of roslaunch
  
  let pid1=$!
  rosbag play $bag_file -s $start --clock -r 0.9 -d 2 -u $duration
  rosnode kill -a
  wait ${pid1}
  let CONTADOR+=1
done


cd $orig_folder
