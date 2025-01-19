#!/bin/bash
if [ -z "$1" ]; then
    echo "Usage: $0 parameter_path"
    exit 1
fi

script_dir=$(dirname "$(realpath "$0")")
date=$(basename "$1")

bag_count=0
LEGO_FOLDER="/lab/tmpig23b/vision_toolkit/data/bag_dump/$1/$2/all_lego"
ODOM_FOLDER="/lab/tmpig23b/vision_toolkit/data/bag_dump/$1/$2/all_odom"

if [ ! -d "$LEGO_FOLDER" ]; then
    echo "Folder '$LEGO_FOLDER' does not exist. Creating it now."
    mkdir -p "$LEGO_FOLDER"
fi

if [ ! -d "$ODOM_FOLDER" ]; then
    echo "Folder '$ODOM_FOLDER' does not exist. Creating it now."
    mkdir -p "$ODOM_FOLDER"
fi

# Loop through each bag file and execute the script
for bag_file in $(ls -v /lab/tmpig23b/vision_toolkit/data/blocks/$1/$2/cam1/*.bag); do
    $script_dir/auto_LeGO.sh "$bag_file"
    sleep 30
    if [ ! -s /tmp/odometry.txt ]; then
        echo "$1 $bag_file" >> LeGO_error.txt
    else
        mv /tmp/odometry.txt "$ODOM_FOLDER/odometry_$bag_count.txt"
        mv /tmp/cornerMap.pcd "$LEGO_FOLDER/cornerMap_$bag_count.pcd"
        mv /tmp/trajectory.pcd "$LEGO_FOLDER/trajectory_$bag_count.pcd"
        mv /tmp/surfaceMap.pcd "$LEGO_FOLDER/surfaceMap_$bag_count.pcd"
    fi

    sleep 20
    ((bag_count+=1))
done


