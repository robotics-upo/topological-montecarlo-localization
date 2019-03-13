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

# Initial Parameters ARSI 2
initial_x=131.431
initial_y=-185.395
initial_a=-2.387
bag_file=/home/chur/Dataset/ARSI/Ribera/ARSI_2017_10_16_CarrerRibera.bag
ground_file=/home/chur/Dataset/ARSI/Ribera/input_vector_ground_truth.txt
start=0
odom_a_mod=0.05
odom_x_mod=0.1


# # With yaw estimation --> yaw_estimator --> true
CONTADOR=$1
directory_out=/home/chur/Dataset/ARSI/Ribera/ground_truth
mkdir -p $directory_out
cd /home/chur/test_ws/src/topological-montecarlo-localization/amcl_sewer/launch
cp amcl_bag_ground_truth_arsi.launch $directory_out
cd ../scripts
cp $0 $directory_out
until [ $CONTADOR -gt $2 ]; do
  # Roslaunch with multiple parameters
  roslaunch amcl_sewer amcl_bag_ground_truth_arsi.launch play_bag:=false ground_truth:=$ground_file\
  ground_truth_out:=${directory_out}/stats_$CONTADOR.txt \
  trajectory_file:=${directory_out}/traj_$CONTADOR.txt \
  odom_a_mod:=$odom_a_mod odom_a_noise:=$odom_a_noise odom_x_mod:=$odom_x_mod odom_y_mod:=$odom_x_mod \
  camera:=/front initial_x:=$initial_x initial_y:=$initial_y initial_a:=$initial_a &
  
  #end of roslaunch
  
  let pid1=$!
  rosbag play $bag_file -s $start --clock -r 2
  rosnode kill -a
  wait ${pid1}
  let CONTADOR+=1
done


cd $orig_folder
