#!/bin/bash
if [ $# -eq 0 ]; then
  echo "Error: Please provide an argument."
  exit 1
fi

SRC_PATH="${2:-/data}"
SAVE_PATH="${3:-/lab/tmpig23b/vision_toolkit/data/bag_dump}"

./extract/bag_.sh "$1" $SRC_PATH $SAVE_PATH
./scripts/run_LeGO.sh "$1" $SRC_PATH $SAVE_PATH

PROJECTION_FOLDER="$HOME/3d2d_ann"
# Need to sync camera for syn_data
"$PROJECTION_FOLDER/sync_cam.sh" "$1"
"$PROJECTION_FOLDER/gpt4/extract_and_gpt4.sh" "$1"

./scripts/run_divide.sh "$1"

  