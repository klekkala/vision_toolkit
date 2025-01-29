#!/bin/bash

DATE="$1"
script_dir=$(dirname "$(realpath "$0")")
OUT_DIR="/lab/tmpig23b/vision_toolkit/data/blocks/"
ODOM_DIR="/lab/tmpig23b/vision_toolkit/data/block_pcl/"
SEQ_GRAPH_DIR="/lab/tmpig23b/vision_toolkit/data/sequence_graph/"

python "$script_dir"/compute_sequence_graph.py --date $DATE --odo $ODOM_DIR$DATE --output SEQ_GRAPH_DIR


