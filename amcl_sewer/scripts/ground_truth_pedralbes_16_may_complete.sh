#! /bin/bash
# $1 --> The first test number
# $2 --> The last number of tests

CONTADOR=$1
orig_folder=$PWD

if [ $# -ne 2 ]; then
  echo "Usage: $0 <first_exe_number> <last_exe_number>"
  exit 1
fi

initial_x=119.6  # Initial position (start 0 original file)
initial_y=-178.5
initial_a=0.844
 
bag_file=/home/chur/Dataset/2018-05-18_Pedralbes/siar_2018-05-18-10-28-30_filtered.bag
bag_out_file=/home/chur/Dataset/2018-05-18_Pedralbes/siar_gt
ground_file=/home/chur/Dataset/2018-05-18_Pedralbes/input_vector_ground_truth.txt
start=0
odom_a_mod=0.2
odom_a_noise=0.1
odom_x_mod=0.2


# # With yaw estimation --> yaw_estimator --> true
CONTADOR=$1
directory_out=/home/chur/Dataset/2018-05-18_Pedralbes/ground_truth/
mkdir -p $directory_out
cd /home/chur/test_ws/src/topological-montecarlo-localization/amcl_sewer/launch
cp amcl_bag_ground_truth.launch $directory_out
cd ../scripts
cp $0 $directory_out
until [ $CONTADOR -gt $2 ]; do
  # Roslaunch with multiple parameters
  roslaunch amcl_sewer amcl_bag_ground_truth.launch play_bag:=false ground_truth:=$ground_file\
  ground_truth_out:=${directory_out}/stats_$CONTADOR.txt \
  trajectory_file:=${directory_out}/traj_$CONTADOR.txt \
  trajectory_file_python:=${directory_out}/traj_python_$CONTADOR.txt \
  yaw_estimator:=false \
  odom_a_mod:=$odom_a_mod odom_a_noise:=$odom_a_noise odom_x_mod:=$odom_x_mod odom_y_mod:=$odom_x_mod \
  camera:=/up initial_x:=$initial_x initial_y:=$initial_y initial_a:=$initial_a rgbd_odom:=false \
  graph_file:=pedralbes_graph_reduced_2 &
  
  #end of roslaunch
  
  let pid1=$!
  roslaunch amcl_sewer bag.launch filename:=$bag_out_file &
  let pid2=$!
  rosbag play $bag_file -s $start --clock -r 0.1
  rosnode kill -a
  wait ${pid1}
  let CONTADOR+=1
done


cd $orig_folder
