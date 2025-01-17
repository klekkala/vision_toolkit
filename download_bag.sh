#!/bin/bash

# Check if an argument is provided
if [ $# -eq 0 ]; then
  echo "Error: Please provide an argument."
  exit 1
fi

# Loop through cameras (adjust range if needed)
for ((i = 1; i <= 1; i++)); do
  index=0
  # Use ssh to list bag files on the remote server
  ssh student@iGpu10 "ls -1v /data/$1/cam$i/*.bag" | while read -r entry; do
      # Check if the file ends with .bag
      if [[ "${entry: -4}" != ".bag" ]]; then
          continue
      fi
      # Run the Python script with the specified arguments
      python3 download_bag.py --bag "student@iGpu10:${entry}" --cam "$i" --session "$1" --date $1
      
      # Increment the index
      index=$((index + 1))
  done
done