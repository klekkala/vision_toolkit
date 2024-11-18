#!/bin/bash

if [ -z "$1" ]; then
    echo "Usage: $0 parameter_path"
    exit 1
fi


./vision_toolkit/scripts/auto_LeGO.sh "$1"
sleep 20
if [ ! -s /tmp/odometry.txt ]; then
    echo "$1" >> LeGO_error.txt
    echo "tmp/odometry.txt not found"
else
    mv /tmp/odometry.txt "/lab/tmpig23b/navisim/data/bag_dump/$3/$2/all_odom/"
    mv /tmp/cornerMap.pcd "/lab/tmpig23b/navisim/data/bag_dump/$3/$2/all_lego/"
    mv /tmp/trajectory.pcd "/lab/tmpig23b/navisim/data/bag_dump/$3/$2/all_lego/"
    mv /tmp/surfaceMap.pcd "/lab/tmpig23b/navisim/data/bag_dump/$3/$2/all_lego/"
fi