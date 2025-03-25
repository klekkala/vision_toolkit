DATE="$1"
SESSION="$2"
# SRC="$3"
SRC='/lab/kiran/vision_toolkit/'

script_dir=$(dirname "$(realpath "$0")")

for SECTOR in 0
do
    python "$script_dir"/build_elevation_occupancy.py --path "$SRC" --date "$DATE" --session "$SESSION" --sector "$SECTOR"
    python "$script_dir"/plot_map.py --path "$SRC" --date "$DATE" --session "$SESSION" --sector "$SECTOR"
done
