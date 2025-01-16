if [ $# -eq 0 ]; then
  echo "Error: Please provide an argument."
  exit 1
fi

for ((i = 1; i<= 5; i++))
do
  for entry in $(ls -1v /data/$1/cam$i/*.bag); do
      if [ "${entry: -4}" != ".bag" ]; then
          continue
      fi
      echo "${entry}"
      python3 ./vision_toolkit/extract/read_bag_.py --bag "${entry}" --cam "$i" --date $1
  done
done
