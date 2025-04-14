DATE="$1"
SESSION="$2"
SRC="$3"

script_dir=$(dirname "$(realpath "$0")")

for SECTOR_DIR in "$SRC/$DATE/$SESSION"/*/; do
    SECTOR=$(basename "$SECTOR_DIR")
  
    python3 "$script_dir"/build_elevation_occupancy.py --path "$SRC" --date "$DATE" --session "$SESSION" --sector "$SECTOR"
    python3 "$script_dir"/plot_map.py --path "$SRC" --date "$DATE" --session "$SESSION" --sector "$SECTOR"
done
