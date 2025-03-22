DATE="$1"
SESSION="$2"
# SRC="$3"
SRC='/lab/kiran/vision_toolkit/'
# SEQUENCE=2

script_dir=$(dirname "$(realpath "$0")")

for sector in 0 1 2 3 4
do
    python "$script_dir"/build_elevation_occupancy.py --path "$SRC" --date "$DATE" --session "$SESSION" --sector "$sector"
    python "$script_dir"/plot_map.py --path "$SRC" --date "$DATE" --session "$SESSION" --sector "$sector"
done
