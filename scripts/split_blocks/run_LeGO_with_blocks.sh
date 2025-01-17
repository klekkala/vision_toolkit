#!/bin/bash
if [ $# -eq 0 ]; then
  echo "Error: Please provide an argument."
  exit 1
fi

script_dir=$(dirname "$(realpath "$0")")
OUT_DIR="/lab/tmpig23b/vision_toolkit/data/blocks/"

index=0
for entry in $(ls -1v /data/$1/cam1/*.bag); do
    
    if [ "${entry: -4}" != ".bag" ]; then
        continue
    fi
    echo "${entry}"
    # python3 $script_dir/split_blocks.py --date "$1" --input "$entry" --output "$OUT_DIR" --session "$index"
    "$script_dir/../run_LeGO.sh" "$1" "$index"
    ((index++))
done
