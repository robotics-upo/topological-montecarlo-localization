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

# Initial Parameters_17_oct 
initial_x=102.2
initial_y=-198.55
initial_a=2.4788
bag_file=/windows/Dataset/2017-10-17/siar_2017-10-17-10-12-16_filtered_gt.bag
ground_file=/windows/Dataset/2017-10-17/input_vector_2017-10-17-10-12-16_ground_truth.txt
start=0
odom_a_mod=0.15
odom_a_noise=0.04
odom_x_mod=0.45


# # With yaw estimation --> yaw_estimator --> true
CONTADOR=$1
directory_out=/home/chur/Dataset/2017-10-17/fused
mkdir -p $directory_out
cd /home/chur/siar_ws/src/topological-montecarlo-localization/amcl_sewer/launch
cp amcl_bag_ground_truth.launch $directory_out
cd ../scripts
cp $0 $directory_out
until [ $CONTADOR -gt $2 ]; do
  # Roslaunch with multiple parameters
  roslaunch amcl_sewer amcl_bag_fused.launch fused:=true angleDev:=0.06 play_bag:=false ground_truth:=$ground_file\
  ground_truth_out:=${directory_out}/stats_$CONTADOR.txt \
  trajectory_file:=${directory_out}/traj_$CONTADOR.txt \
  trajectory_file_python:=${directory_out}/traj_python_$CONTADOR.txt \
  yaw_estimator:=true inspection:=false \
  odom_a_mod:=$odom_a_mod odom_a_noise:=$odom_a_noise odom_x_mod:=$odom_x_mod odom_y_mod:=$odom_x_mod \
  min_particles:=200 max_particles:=500 \
  camera:=/up initial_x:=$initial_x initial_y:=$initial_y initial_a:=$initial_a rgbd_odom:=false &
  
  #end of roslaunch
  
  let pid1=$!
    rosbag play $bag_file -s $start --clock -r 0.8 -d 1 
  rosnode kill -a
  wait ${pid1}
  let CONTADOR+=1
done


cd $orig_folder
