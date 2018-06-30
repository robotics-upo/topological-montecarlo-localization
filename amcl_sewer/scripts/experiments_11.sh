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

# Initial Parameters_17_jan 
initial_x=64.058
initial_y=-250.36
initial_a=0.7893873
bag_file=/home/chur/Dataset/2017-01-17_sewers_jan/siar_2017-01-17-11-17-28.bag
ground_file=/home/chur/Dataset/2017-10-17_Demo_Sewers/input_vector_2017-10-17-10-12-16_ground_truth.txt
start=390
odom_a_mod=0.12
odom_a_noise=0.05
odom_x_mod=0.3


# # With yaw estimation --> yaw_estimator --> true
CONTADOR=$1
directory_out=/home/chur/stats/stats17_jan_wheel_mod_${odom_x_mod}_mod_a_${odom_a_mod}_noise_${odom_a_noise}
mkdir -p $directory_out
cd /home/chur/test_ws/src/siar_packages/localization_siar/amcl_sewer/launch
cp amcl_bag.launch $directory_out
cd ../scripts
cp $0 $directory_out
until [ $CONTADOR -gt $2 ]; do
  # Roslaunch with multiple parameters
  roslaunch amcl_sewer amcl_bag.launch play_bag:=false ground_truth:=$ground_file \
  ground_truth_out:=${directory_out}/stats_$CONTADOR.txt \
  yaw_estimator:=false detector:=true \
  odom_a_mod:=$odom_a_mod odom_a_noise:=$odom_a_noise odom_x_mod:=$odom_x_mod odom_y_mod:=$odom_x_mod \
  camera:=/front initial_x:=$initial_x initial_y:=$initial_y initial_a:=$initial_a rgbd_odom:=false &
  
  #end of roslaunch
  
  let pid1=$!
  rosbag play $bag_file -s $start --clock -r 2
  rosnode kill -a
  wait ${pid1}
  let CONTADOR+=1
done

# # With yaw estimation --> yaw_estimator --> true
CONTADOR=$1
directory_out=/home/chur/stats/stats17_jan_wheel_yaw_mod_${odom_x_mod}_mod_a_${odom_a_mod}_noise_${odom_a_noise}
mkdir -p $directory_out
cd /home/chur/test_ws/src/siar_packages/localization_siar/amcl_sewer/launch
cp amcl_bag.launch $directory_out
cd ../scripts
cp $0 $directory_out
until [ $CONTADOR -gt $2 ]; do
  # Roslaunch with multiple parameters
  roslaunch amcl_sewer amcl_bag.launch play_bag:=false ground_truth:=$ground_file \
  ground_truth_out:=${directory_out}/stats_$CONTADOR.txt \
  yaw_estimator:=true detector:=true \
  odom_a_mod:=$odom_a_mod odom_a_noise:=$odom_a_noise odom_x_mod:=$odom_x_mod odom_y_mod:=$odom_x_mod \
  camera:=/front initial_x:=$initial_x initial_y:=$initial_y initial_a:=$initial_a rgbd_odom:=false &
  


cd $orig_folder
