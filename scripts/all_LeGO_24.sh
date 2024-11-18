#!/bin/bash

# Input file
input_file="/lab/tmpig23b/navisim/data/bag_dump/All_sessions.txt"


# Read input file line by line
while IFS=$'\t' read -r date_and_session bag_file bag_files; do
    # Extract date and session
    date=$(echo "$date_and_session" | cut -d'/' -f1)
    session=$(echo "$date_and_session" | cut -d'/' -f2)

    bag_path="/lab/tmpig23b/navisim/data/bag_dump/$date/bags/cam1/$(basename "$bag_file")"
    # bag_path = "/data/$date/cam1/*.bag"
    echo $bag_path
    ./vision_toolkit/scripts/run_LeGO_parameter.sh "$bag_path" "$session" "$date"
    sleep 20
done < "$input_file"
