import argparse
import os
import shutil
import json
from pathlib import Path
from collections import defaultdict
from tqdm import tqdm

def extract_timestamp_seconds(filename: str) -> int:
    try:
        return int(filename.split("_")[1].split(".")[0]) // 1_000_000_000
    except (IndexError, ValueError):
        return None

def group_images_by_camera(img_dir: Path):
    camera_images = defaultdict(list)
    for file in img_dir.glob("*.jpg"):
        if file.name.startswith("cam") and "_" in file.name:
            ts = extract_timestamp_seconds(file.name)
            if ts is not None:
                cam_id = file.stem.split("_")[0]  # e.g., "cam5"
                camera_images[cam_id].append((ts, file.name))
    return camera_images

def ensure_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)

def save_images_to_sector(sector_idx, images, img_dir, output_dir, sector_map):
    sector_name = f"sector{sector_idx}"
    sector_path = output_dir / sector_name / 'imgs'
    ensure_dir(sector_path)

    for _, fname in tqdm(images, desc=f"[{sector_name}]", leave=False):
        src = img_dir / fname
        dst = sector_path / fname
        shutil.copy2(src, dst)
        sector_map[sector_name].append(fname)

def assign_images_to_sectors(img_dir: Path, output_dir: Path, window_size: int):
    camera_images = group_images_by_camera(img_dir)
    sector_map = defaultdict(list)  # {"sector0": [img1, img2], "sector1": [...]}
    sector_bounds = {}  # {(cam_id, sector_idx): (start_time, end_time)}

    for cam_id, images in camera_images.items():
        images.sort()
        if not images:
            continue

        start_time = images[0][0]
        end_time = images[-1][0]
        sector_idx = 0
        current_start = start_time

        while current_start <= end_time:
            sector_name = f"sector{sector_idx}"
            window_images = [(ts, fn) for ts, fn in images if current_start <= ts < current_start + window_size]
            if window_images:
                save_images_to_sector(sector_idx, window_images, img_dir, output_dir, sector_map)
                sector_bounds[(cam_id, sector_idx)] = (current_start, current_start + window_size)
            current_start += window_size
            sector_idx += 1

    return sector_map

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--src', type=str, required=True)
    parser.add_argument('--date', type=str, required=True)
    parser.add_argument('--session', type=str, default="0")
    parser.add_argument('--target', type=str, required=True)
    parser.add_argument('--window', type=int, default=20, help='Time window in seconds per sector')
    args = parser.parse_args()

    img_dir = Path(args.src) / args.date / args.session / "all_imgs"
    output_dir = Path(args.target) / args.date / args.session
    json_path = Path(args.target) / args.date / args.session / "sector_data.json"

    print(f"Processing per-camera time-window images from: {img_dir}")
    print(f"Saving output to: {output_dir}")

    sector_map = assign_images_to_sectors(img_dir, output_dir, args.window)

    with open(json_path, "w") as f:
        json.dump(sector_map, f, indent=2)

    print(f"\n✅ Sector map saved to {json_path}")

if __name__ == "__main__":
    main()