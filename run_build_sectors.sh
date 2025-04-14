#!/bin/bash
if [ $# -eq 0 ]; then
  echo "Error: Please provide an argument."
  exit 1
fi

DATE=$1
SESSION=$2
WINDOW_SIZE=20
SRC_DIR='/lab/tmpig23b/vision_toolkit/data/bag_dump'
OUT_DIR='/lab/kiran/vision_toolkit/output/sectors'

./scripts/sequence2sector/seq2sector.sh $DATE $SESSION $WINDOW_SIZE $SRC_DIR $OUT_DIR
./scripts/elevation_map/run_build_elevation_occupancy_map.sh $DATE $SESSION $OUT_DIR 