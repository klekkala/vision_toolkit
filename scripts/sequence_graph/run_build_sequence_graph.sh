#!/bin/bash
DATE="$1"
SESSION="$2"
SRC="$3"
OUT="$4"
# DISPLAY="$5"

script_dir=$(dirname "$(realpath "$0")")
python "$script_dir"/compute_sequence_graph.py --path "$SRC" --date "$DATE" --session "$SESSION" --out "$OUT"


