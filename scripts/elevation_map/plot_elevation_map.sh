DATE="$1"
SESSION="$2"
SRC="$3"
SEQUENCE=0

script_dir=$(dirname "$(realpath "$0")")
python "$script_dir"/build_elevation_occupancy.py --path "$SRC" --date "$DATE" --session "$SESSION" --sequence "$SEQUENCE"