#!/bin/bash
source /opt/ros/noetic/setup.bash


roscore &

pid1=$!
sleep 10

input_file="/lab/tmpig23b/vision_toolkit/data/bag_dump/All_sessions.txt"
output_dir="/lab/tmpig23b/vision_toolkit/data/bag_dump"


# Read input file line by line
while IFS=$'\t' read -r date_and_session bag_file bag_files; do
    # Extract date and session
    date=$(echo "$date_and_session" | cut -d'/' -f1)
    session=$(echo "$date_and_session" | cut -d'/' -f2)
    bag_path="/data/$date/cam1/$(basename "$bag_file")"
    echo $bag_path

    mkdir -p "/$output_dir/$date/$session/all_pcl/"
    rosrun pcl_ros bag_to_pcd "$bag_path" /velodyne_points "/lab/tmpig23b/vision_toolkit/data/bag_dump/$date/$session/all_pcl"

    sleep 10

    if [ -z "$(ls -A "/lab/tmpig23b/vision_toolkit/data/bag_dump/$date/$session/all_pcl/")" ]; then
    echo "$bag_path" >> error_pcl.txt
fi
done < "$input_file"
exit 0

