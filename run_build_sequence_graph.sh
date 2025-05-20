#!/bin/bash
SEC_DIR='/lab/kiran/vision_toolkit/output/sectors/'
OUT_DIR='./output/sequence_graph'
SEQ="2023_03_29/0,2023_03_29/1,2023_03_29/2,2023_03_29/3,2023_03_29/4,2023_03_29/5,2023_03_29/6"

./scripts/sequence_graph/run_build_sequence_graph.sh $SEQ $SEC_DIR $OUT_DIR
