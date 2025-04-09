if [ $# -eq 0 ]; then
  echo "Error: Please provide an argument."
  exit 1
fi

script_dir=$(dirname "$(realpath "$0")")
SRC_PATH="${2:-/data}"
SAVE_PATH="${3:-/lab/tmpig23b/vision_toolkit/data/bag_dump}"

for ((i = 1; i<= 5; i++))
do
  index=0
  for entry in $(ls -1v $SRC_PATH/$1/cam$i/*.bag); do

      if [ "${entry: -4}" != ".bag" ]; then
          continue
      fi
      echo "Extracting images from ${entry}"
      python3 $script_dir/read_bag_.py --bag "${entry}" --cam "$i" --date $1 --session $index --save_path $SAVE_PATH
      ((index++))
  done
done
