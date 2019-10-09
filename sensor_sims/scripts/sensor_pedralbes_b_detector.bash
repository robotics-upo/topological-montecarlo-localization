#!/bin/bash
# $1 --> The first uav number
# $2 --> The last number of uavs
# $3 --> The path to the repository (resolution)
# $4 --> Number of tests to be generated in each folder
# $5 --> Base input file

CONTADOR=$1
orig_folder=$PWD

if [ $# -ne 2 ]; then
  echo "Usage: $0 <first_exe_number> <last_exe_number>"
  exit 1
fi

source ~/siar_ws/devel/setup.bash

# Initial Parameters_11_oct 
initial_x=96.24 #Initial position (start 980)
initial_y=-182.72
initial_a=2.47
 
bag_file=/windows/Dataset/2018-06-12_sewers_pedralbes/siar_2018-06-12-10-54-51_short_filtered.bag
ground_file=/windows/Dataset/2018-06-12_sewers_pedralbes/input_vector_ground_truth_b.txt
start=0
duration=2005
odom_a_mod=0.12
odom_a_noise=0.04
odom_x_mod=0.2

# # With yaw estimation --> yaw_estimator --> true
CONTADOR=$1
directory_out=/home/chur/Dataset/pedralbes/detector
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
  yaw_estimator:=true inspection:=false angle_dev:=0.06 \
  emit_tf_base:=true base_frame_id:=base_link_vo odom_frame_id:=odom_vo \
  odom_a_mod:=$odom_a_mod odom_a_noise:=$odom_a_noise odom_x_mod:=$odom_x_mod odom_y_mod:=$odom_x_mod \
  min_particles:=300 max_particles:=400 \
  camera:=/up initial_x:=$initial_x initial_y:=$initial_y initial_a:=$initial_a rgbd_odom:=false \
  graph_file:=pedralbes_graph_reduced &
  
  #end of roslaunch
  
  let pid1=$!
  rosbag play $bag_file -s $start --clock -r 2 -d 2 -u $duration
  rosnode kill -a
  wait ${pid1}
  let CONTADOR+=1
done


cd $orig_folder
