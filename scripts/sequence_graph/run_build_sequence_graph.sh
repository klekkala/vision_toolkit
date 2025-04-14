#!/bin/bash
SEQ="$1"
SRC="$2"
OUT="$3"
# DISPLAY="$5"

script_dir=$(dirname "$(realpath "$0")")
python "$script_dir"/build_sequence_graph.py --path "$SRC" --sequences="$SEQ" --out "$OUT"


