if [ "$#" -ne 1 ]; then
  echo "Please provide a date to be used"
  exit 1
fi

input="/lab/tmpig13b/jiwon/bags/$1/"
output="/lab/tmpig13b/jiwon/blocks/$1/"

python split_block.py -i "$input" -o "$output" --date "$1"