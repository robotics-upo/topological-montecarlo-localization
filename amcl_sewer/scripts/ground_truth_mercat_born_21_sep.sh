#! /bin/bash
# $1 --> The first test number
# $2 --> The last number of tests

CONTADOR=$1
orig_folder=$PWD

if [ $# -ne 2 ]; then
  echo "Usage: $0 <first_exe_number> <last_exe_number>"
  exit 1
fi

# Initial Parameters_21_sept. 
initial_x=100.52
initial_y=-197.05
initial_a=-0.562
bag_file=/home/chur/Dataset/2017-09-21_full_track_surface/siar_2017-09-21-12-17-39_filtered.bag
bag_out_file=/home/chur/Dataset/2017-09-21_full_track_surface/siar_gt
ground_file=/home/chur/Dataset/2017-09-21_full_track_surface/input_vector_2017-09-21_ground_truth.txt
start=20
end=3260
odom_a_mod=0.2
odom_x_mod=0.3


# # With yaw estimation --> yaw_estimator --> true
CONTADOR=$1
directory_out=/home/chur/Dataset/2017-09-21_full_track_surface/ground_truth
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
  camera:=/front initial_x:=$initial_x initial_y:=$initial_y initial_a:=$initial_a rgbd_odom:=false &
  
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
