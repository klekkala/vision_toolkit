#!/bin/bash
source ~/catkin_ws/devel/setup.bash
rosparam set use_sim_time true

bag_file=$1

roslaunch lego_loam run.launch &

pid1=$!
sleep 10
rosbag play "$bag_file" --topic /velodyne_points /cam1/imu --clock &

pid2=$!

wait $pid2

if [ $? -eq 0 ]; then
    kill $pid1
    exit 0
else
    exit 1
fi
