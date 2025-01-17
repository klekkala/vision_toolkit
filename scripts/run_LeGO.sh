#!/bin/bash
if [ -z "$1" ]; then
    echo "Usage: $0 parameter_path"
    exit 1
fi

script_dir=$(dirname "$(realpath "$0")")
date=$(basename "$1")

bag_count=0

# Loop through each bag file and execute the script
for bag_file in $(ls -v /lab/tmpig23b/vision_toolkit/data/blocks/$1/$2/cam1/*.bag); do
    $script_dir/auto_LeGO.sh "$bag_file"
    sleep 30
    if [ ! -s /tmp/odometry.txt ]; then
        echo "$1 $bag_file" >> LeGO_error.txt
    else
        mv /tmp/odometry.txt "/lab/tmpig23b/vision_toolkit/data/bag_dump/$date/$2/all_odom/odometry_$bag_count.txt"
        mv /tmp/cornerMap.pcd "/lab/tmpig23b/vision_toolkit/data/bag_dump/$date/$2/all_lego/cornerMap_$bag_count.pcd"
        mv /tmp/trajectory.pcd "/lab/tmpig23b/vision_toolkit/data/bag_dump/$date/$2/all_lego/trajectory_$bag_count.pcd"
        mv /tmp/surfaceMap.pcd "/lab/tmpig23b/vision_toolkit/data/bag_dump/$date/$2/all_lego/surfaceMap_$bag_count.pcd"
    fi

    sleep 20
    ((bag_count+=1))
done


