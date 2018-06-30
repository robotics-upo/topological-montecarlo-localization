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

# Initial Parameters_21_sept. 
initial_x=100.52
initial_y=-197.05
initial_a=-0.562
bag_file=/home/chur/Dataset/2017-09-21_full_track_surface/siar_2017-09-21-12-17-39.bag
ground_file=/home/chur/Dataset/2017-09-21_full_track_surface/input_vector_2017-09-21_ground_truth.txt
start=560
end=3260
odom_a_mod=0.05
odom_x_mod=0.1

# Now with wheel odometry
# 

# directory_out=/home/chur/stats/stats21_sept_wheel_mod_${odom_x_mod}_mod_a_${odom_a_mod}_noise_${odom_a_mod}
# mkdir -p $directory_out
#  until [ $CONTADOR -gt $2 ]; do  
#    roslaunch amcl_sewer amcl_bag.launch play_bag:=false ground_truth:=$ground_file \
#    ground_truth_out:=${directory_out}/stats_$CONTADOR.txt \
#   odom_a_mod:=$odom_a_mod odom_a_noise:=$odom_a_mod odom_x_mod:=$odom_x_mod odom_y_mod:=$odom_x_mod \
#   initial_x:=$initial_x initial_y:=$initial_y initial_a:=$initial_a rgbd_odom:=false &
#   let pid1=$!
#   rosbag play $bag_file -s $start --clock -r 2.2 -u $end
#   rosnode kill -a
#   wait ${pid1}
#   let CONTADOR+=1
# done

# Visual Odometry
# CONTADOR=$1
# 

# directory_out=/home/chur/stats/stats21_sept_wheel_mod_${odom_x_mod}_mod_a_${odom_a_mod}_noise_${odom_a_mod}
# mkdir -p $directory_out
#  until [ $CONTADOR -gt $2 ]; do  
#    roslaunch amcl_sewer amcl_bag.launch play_bag:=false ground_truth:=$ground_file \
#    ground_truth_out:=${directory_out}/stats_$CONTADOR.txt \
#   odom_a_mod:=$odom_a_mod odom_a_noise:=$odom_a_mod odom_x_mod:=$odom_x_mod odom_y_mod:=$odom_x_mod \
#   initial_x:=$initial_x initial_y:=$initial_y initial_a:=$initial_a rgbd_odom:=true &
#   let pid1=$!
#   rosbag play $bag_file -s $start --clock -r 2.2 -u $end
#   rosnode kill -a
#   wait ${pid1}
#   let CONTADOR+=1
# done
# # NOTE: For adjusting noise settings
# #   odom_a_mod:=0.25 odom_a_noise:=0.075 odom_x_mod:=0.5 odom_y_mod:=0.5 \
# 
# # With yaw estimation
CONTADOR=$1
directory_out=/home/chur/stats/stats21_sept_yaw_wheel_mod_${odom_x_mod}_mod_a_${odom_a_mod}_noise_${odom_a_mod}
mkdir -p $directory_out
cd /home/chur/test_ws/src/siar_packages/localization_siar/amcl_sewer/launch
cp amcl_bag.launch $directory_out
cd ../scripts
cp experiment_full_track_vo_vs_wheel.sh $directory_out
until [ $CONTADOR -gt $2 ]; do
  # Roslaunch with multiple parameters
  roslaunch amcl_sewer amcl_bag.launch play_bag:=false ground_truth:=$ground_file \
  ground_truth_out:=${directory_out}/stats_$CONTADOR.txt \
  yaw_estimator:=true \
  odom_a_mod:=$odom_a_mod odom_a_noise:=$odom_a_mod odom_x_mod:=$odom_x_mod odom_y_mod:=$odom_x_mod \
  camera:=/front initial_x:=$initial_x initial_y:=$initial_y initial_a:=$initial_a rgbd_odom:=false &
  
  #end of roslaunch
  
  let pid1=$!
  rosbag play $bag_file -s $start --clock -r 2 -u 3260
  rosnode kill -a
  wait ${pid1}
  let CONTADOR+=1
done

# Visual Odometry
# 
# CONTADOR=$1
# odom_x_mod=0.3
# odom_y_mod=0.2
# directory_out=/home/chur/stats/stats21_sept_vo_yaw_mod_${odom_x_mod}_mod_a_${odom_a_mod}_noise_0
# mkdir -p $directory_out
#  until [ $CONTADOR -gt $2 ]; do  
#    roslaunch amcl_sewer amcl_bag.launch play_bag:=false ground_truth:=$ground_file \
#    ground_truth_out:=${directory_out}/stats_$CONTADOR.txt \
#   odom_a_mod:=$odom_a_mod odom_a_noise:=0 odom_x_mod:=$odom_x_mod odom_y_mod:=$odom_x_mod \
#   yaw_estimator:=true \
#   initial_x:=$initial_x initial_y:=$initial_y initial_a:=$initial_a rgbd_odom:=true &
#   let pid1=$!
#   rosbag play $bag_file -s $start --clock -r 0.9 -u $end
#   rosnode kill -a
#   wait ${pid1}
#   let CONTADOR+=1
# done

cd $orig_folder