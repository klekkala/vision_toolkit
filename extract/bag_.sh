if [ $# -eq 0 ]; then
  echo "Error: Please provide an argument."
  exit 1
fi

for ((i = 1; i<= 5; i++))
do
  index=0
  for entry in $(ls -1v /lab/tmpig23b/navisim/data/bags/$1/$index/cam$i/*.bag); do
      if [ "${entry: -4}" != ".bag" ]; then
          continue
      fi
      echo "${entry}"
      python3 ./vision_toolkit/extract/read_bag_.py --bag "${entry}" --cam "$i" --save $1 --num $index
      ((index++))
  done
done
