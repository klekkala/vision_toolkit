#!/bin/bash

if [ -z "$1" ]; then
    echo "Usage: $0 parameter_path"
    exit 1
fi


./auto_LeGO.sh "$1"
sleep 20
if [ ! -s /tmp/odometry.txt ]; then
    echo "$1" >> LeGO_error.txt
else

    mkdir -p "/lab/tmpig13b/kiran/bag_dump/$3/$2/all_odom/$4/"
    mkdir -p "/lab/tmpig13b/kiran/bag_dump/$3/$2/all_lego/$4/"
    mv /tmp/odometry.txt "/lab/tmpig13b/kiran/bag_dump/$3/$2/all_odom/$4/"
    mv /tmp/cornerMap.pcd "/lab/tmpig13b/kiran/bag_dump/$3/$2/all_lego/$4/"
    mv /tmp/trajectory.pcd "/lab/tmpig13b/kiran/bag_dump/$3/$2/all_lego/$4/"
    mv /tmp/surfaceMap.pcd "/lab/tmpig13b/kiran/bag_dump/$3/$2/all_lego/$4/"
fi



