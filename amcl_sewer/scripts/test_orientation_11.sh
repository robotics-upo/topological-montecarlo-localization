# $1 --> The first uav number
# $2 --> The last number of uavs
# $3 --> The path to the repository (resolution)
# $4 --> Number of tests to be generated in each folder
# $5 --> Base input file

# Initial position: MH Comercial 3
# initial_x=42.2586
# initial_y=-137.46	
# initial_a=2.7
#start=1200
#end=400

# Initial position: MH Comercial 1
initial_x=100.878
initial_y=-197.30 
initial_a=2.4
start=742
end=1100

bag_file=/home/chur/Dataset/2017-01-17_sewers_jan/siar_2017-01-17-11-17-28.bag
ground_file=/home/chur/Dataset/2017-01-17_sewers_jan/input_vector_2017-01-17-11_ground_truth
odom_a_mod=0.075
odom_x_mod=0.1
directory_out=/home/chur/stats

roslaunch amcl_sewer amcl_bag.launch play_bag:=false ground_truth:=$ground_file \
ground_truth_out:=${directory_out}/stats_test_$CONTADOR.txt \
odom_a_mod:=$odom_a_mod odom_a_noise:=$odom_a_mod odom_x_mod:=$odom_x_mod odom_y_mod:=$odom_x_mod \
yaw_estimator:=true \
initial_x:=$initial_x initial_y:=$initial_y initial_a:=$initial_a rgbd_odom:=false &
let  pid1=$!
sleep 2
rosbag play ${bag_file} -s $start -u $end --clock -r 1.5
rosnode kill -a
wait ${pid1}
