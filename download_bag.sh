#!/bin/bash
if [ $# -eq 0 ]; then
  echo "Error: Please provide an argument."
  exit 1
fi

for ((i = 1; i<= 5; i++))
do
  index=0
  ssh student@iGpu10 "ls -1v /data/$1/cam$i/*.bag" | while read entry; do
      if [ "${entry: -4}" != ".bag" ]; then
          continue
      fi
      python3 download_bag.py --bag "student@iGpu10:${entry}" --cam "$i" --date $1
      ((index++))
  done
done