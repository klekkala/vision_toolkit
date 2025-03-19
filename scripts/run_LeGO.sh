#!/bin/bash
if [ -z "$1" ]; then
    echo "Usage: $0 parameter_path"
    exit 1
fi

script_dir=$(dirname "$(realpath "$0")")
date=$(basename "$1")

session=0
OUTPUT_DIR="/lab/tmpig23b/vision_toolkit/data/bag_dump/$1"

# Loop through each bag file and execute the script
for bag_file in $(ls -v /data/$1/cam1/*.bag); do
    $script_dir/auto_LeGO.sh "$bag_file"
    
    sleep 30
    if [ ! -s /tmp/odometry.txt ]; then
        echo "Error while processing $date/$session" 
        echo "$1 $bag_file" >> LeGO_error.txt
    else
        echo "Success! Moving files to $OUTPUT_DIR/$session"
        if [ ! -d "$OUTPUT_DIR/$session/all_odom" ]; then
            echo "Folder '$OUTPUT_DIR/$session/all_odom' does not exist. Creating it now."
            mkdir -p "$OUTPUT_DIR/$session/all_odom"
        fi

        if [ ! -d "$OUTPUT_DIR/$session/all_lego" ]; then
            echo "Folder '$OUTPUT_DIR/$session/all_lego' does not exist. Creating it now."
            mkdir -p "$OUTPUT_DIR/$session/all_lego"
        fi

        mv /tmp/odometry.txt "$OUTPUT_DIR/$session/all_odom/odometry.txt"
        mv /tmp/cornerMap.pcd "$OUTPUT_DIR/$session/all_lego/cornerMap.pcd"
        mv /tmp/trajectory.pcd "$OUTPUT_DIR/$session/all_lego/trajectory.pcd"
        mv /tmp/surfaceMap.pcd "$OUTPUT_DIR/$session/all_lego/surfaceMap.pcd"
    fi

    sleep 20
    ((session++))
done


