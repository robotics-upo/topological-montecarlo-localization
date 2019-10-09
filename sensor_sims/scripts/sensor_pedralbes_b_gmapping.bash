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
directory_out=/home/chur/Dataset/pedralbres/gmapping
mkdir -p $directory_out
start=0
duration=2005
odom_a_mod=0.12
odom_a_noise=0.04
odom_x_mod=0.2

# # With yaw estimation --> yaw_estimator --> true
CONTADOR=$1
roscd sensor_sims/launch
cp gmapping_bag.launch $directory_out
cd ../scripts
cp $0 $directory_out
until [ $CONTADOR -gt $2 ]; do
  # Roslaunch with multiple parameters
  roslaunch sensor_sims gmapping_bag.launch play_bag:=false ground_truth:=$ground_file\
  ground_truth_out:=${directory_out}/stats_$CONTADOR.txt \
  trajectory_file:=${directory_out}/traj_$CONTADOR.txt \
  trajectory_file_python:=${directory_out}/traj_python_$CONTADOR.txt \
  base_frame_id:=base_link odom_frame_id:=odom \
  camera:=/up initial_x:=$initial_x initial_y:=$initial_y initial_a:=$initial_a rgbd_odom:=true \
  graph_file:=pedralbes_graph &
  
  #end of roslaunch
  
  let pid1=$!
  rosbag play $bag_file -s $start --clock -r 1.5 -d 2 -u $duration
  rosnode kill -a
  wait ${pid1}
  let CONTADOR+=1
done


cd $orig_folder
