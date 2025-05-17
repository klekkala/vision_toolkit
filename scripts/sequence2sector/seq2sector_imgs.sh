if [ $# -eq 0 ]; then
  echo "Error: Please provide an argument."
  exit 1
fi

DATE=$1
SESSION=$2
WINDOW_size=$3

OUT_SEC=$4
SRC_IMG=$5
OUT_IMG=$6

python run_seq2sector_imgs.py --src "$SRC_IMG" --target "$OUT_IMG" --date "$DATE" --session "$SESSION"



