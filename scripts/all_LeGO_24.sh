#!/bin/bash

# Input file
input_file="/lab/tmpig13b/kiran/bag_dump/All_sessions.txt"


# Read input file line by line
while IFS=$'\t' read -r date_and_session bag_file bag_files; do
    # Extract date and session
    date=$(echo "$date_and_session" | cut -d'/' -f1)
    session=$(echo "$date_and_session" | cut -d'/' -f2)

    bag_path="/data/$date/cam1/$(basename "$bag_file")"
    echo $bag_path
    ./run_LeGO_parameter.sh "$bag_path" "$session" "$date"
    sleep 20
done < "$input_file"
