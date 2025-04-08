#!/bin/bash
if [ $# -eq 0 ]; then
  echo "Error: Please provide an argument."
  exit 1
fi

DATE=$1
SESSION=$2
WINDOW_size=$3
SRC_DIR=$4
OUT_DIR=$5

script_dir=$(dirname "$(realpath "$0")")
python "$script_dir"/seq2sector.py --date $DATE --session $SESSION --window_size $WINDOW_size --src_dir $SRC_DIR --out_dir $OUT_DIR