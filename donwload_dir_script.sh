#!/bin/bash
if [ $# -eq 0 ]; then
  echo "Error: Please provide an argument."
  exit 1
fi

python3 download_dir.py --path "student@iGpu24:/lab/tmpig13b/kiran/bag_dump/$1/$2/$3" --date "$1" --session "$2" --dir "$3"

# ssh student@iGpu24 "ls -1v /lab/tmpig13b/kiran/bag_dump/$1/$2/$3" | while read entry; do
#     echo "${entry}"
#     # if [ "${entry: -4}" != ".bag" ]; then
#     # #     continue
#     # # fi
#     # python3 download_bag.py --bag "student@iGpu10:${entry}" --cam "$i" --save $1

# done