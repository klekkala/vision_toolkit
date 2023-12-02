source ./_carla-ros-bridge/catkin_ws/devel/setup.bash


if [ $# -eq 0 ]; then
  echo "Error: Please provide an argument."
  exit 1
fi

for entry in /data/$1/cam1/*; do
    if [ "${entry: -4}" != ".bag" ]; then
        continue
    fi
    echo "${entry}"
    python3 ./read_bag_.py --bag "$entry" --cam 1 --save $1
  done

for entry in /data/$2/cam1/*; do
    if [ "${entry: -4}" != ".bag" ]; then
        continue
    fi
    echo "${entry}"
    python3 ./read_bag_.py --bag "$entry" --cam 2 --save $1
  done


for entry in /data/$3/cam1/*; do
    if [ "${entry: -4}" != ".bag" ]; then
        continue
    fi
    echo "${entry}"
    python3 ./read_bag_.py --bag "$entry" --cam 3 --save $1
  done

for entry in /data/$1/cam4/*; do
    if [ "${entry: -4}" != ".bag" ]; then
        continue
    fi
    echo "${entry}"
    python3 ./read_bag_.py --bag "$entry" --cam 4 --save $1
  done

for entry in /data/$1/cam5/*; do
    if [ "${entry: -4}" != ".bag" ]; then
        continue
    fi
    echo "${entry}"
    python3 ./read_bag_.py --bag "$entry" --cam 5 --save $1
  done

