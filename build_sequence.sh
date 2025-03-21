#!/bin/bash
if [ $# -eq 0 ]; then
  echo "Error: Please provide an argument."
  exit 1
fi

DATE=$1
SESSION=$2
WINDOW_SIZE=20
SRC_DIR='/lab/tmpig23b/vision_toolkit/data/bag_dump'
SEQ_OUT='.'

./scripts/sector2seq/sector2sequence.sh $DATE $SESSION $WINDOW_SIZE $SRC_DIR $SEQ_OUT
./scripts/sequence_graph/run_compute_sequence_graph.sh $DATE $SESSION $OUT_DIR $OUT_DIR
./scripts/elevation_map/plot_elevation_map.sh $DATE $SESSION $OUT_DIR 