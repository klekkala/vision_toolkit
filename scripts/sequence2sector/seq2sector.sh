#!/bin/bash
if [ $# -eq 0 ]; then
  echo "Error: Please provide an argument."
  exit 1
fi

DATE=$1
SESSION=$2
WINDOW_SIZE=$3
SRC_DIR=$4
OUT_DIR=$5

script_dir=$(dirname "$(realpath "$0")")
python "$script_dir"/seq2sector.py --date $DATE --session $SESSION --window_size $WINDOW_SIZE --src_dir $SRC_DIR --out_dir $OUT_DIR
python "$script_dir"/run_seq2sector_imgs.py --src "$SRC_DIR" --target "$OUT_DIR" --date "$DATE" --session "$SESSION"
python "$script_dir"/run_seq2sector_pcl.py --src "$SRC_DIR" --target "$OUT_DIR" --date "$DATE" --session "$SESSION"