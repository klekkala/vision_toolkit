#!/bin/bash
if [ $# -eq 0 ]; then
  echo "Error: Please provide an argument."
  exit 1
fi

./extract/bag_.sh "$1"
./scripts/all_pcl.sh "$1"
./scripts/split_blocks/run_LeGO_with_blocks.sh "$1" 

PROJECTION_FOLDER="$HOME/3d2d_ann"
# Need to sync camera for syn_data
"$PROJECTION_FOLDER/sync_cam.sh" "$1"
"$PROJECTION_FOLDER/gpt4/extract_and_gpt4.sh" "$1"

.//scripts/run_divide.sh "$1"

  