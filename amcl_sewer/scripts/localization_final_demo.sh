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

# Initial Parameters garcia Faria
initial_x=-191.796798706 # Initial position 
initial_y=-142.06
initial_a=0.8

bag_file=/home/chur/Dataset/2018-12-13-Final-Demo/siar_2018-12-13-11-43-26.bag
ground_file=/home/chur/Dataset/2018-12-13-Final-Demo/ground_truth.txt
start=1030
rate=2
odom_a_mod=0.2
odom_a_noise=0.1
odom_x_mod=0.2


# # With yaw estimation --> yaw_estimator --> true
CONTADOR=$1
directory_out=/home/chur/Dataset/2018-12-13-Final-Demo/ground_truth/
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
  yaw_estimator:=false \
  odom_a_mod:=$odom_a_mod odom_a_noise:=$odom_a_noise odom_x_mod:=$odom_x_mod odom_y_mod:=$odom_x_mod \
  camera:=/front initial_x:=$initial_x initial_y:=$initial_y initial_a:=$initial_a rgbd_odom:=false \
  graph_file:=Pg_Garcia_Faria.graph &
  
  #end of roslaunch
  let pid1=$!
  rosbag play $bag_file -s $start --clock -r $rate
  rosnode kill -a
  wait ${pid1}
  let CONTADOR+=1
done


cd $orig_folder
