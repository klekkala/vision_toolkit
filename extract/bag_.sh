if [ $# -eq 0 ]; then
  echo "Error: Please provide an argument."
  exit 1
fi

script_dir=$(dirname "$(realpath "$0")")

for ((i = 1; i<= 5; i++))
do
  index=0
  for entry in $(ls -1v /data/$1/cam$i/*.bag); do

      if [ "${entry: -4}" != ".bag" ]; then
          continue
      fi
      echo "${entry}"
      python3 $script_dir/read_bag_.py --bag "${entry}" --cam "$i" --date $1 --session $index
      ((index++))
  done
done
