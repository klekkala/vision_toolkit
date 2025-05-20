import argparse
import os
import shutil
import json
from pathlib import Path
from collections import defaultdict
from tqdm import tqdm


def split_pcd_by_time(pcd_dir, output_dir, window=20):
    """
    Split PCD files in `pcd_dir` into sectors based on timestamp and save them in `output_dir`.
    Each sector contains PCDs that fall within a `window` seconds interval.
    Returns a dictionary mapping sector names to lists of filenames.
    """
    pcd_files = sorted(Path(pcd_dir).glob("*.pcd"))

    # Extract timestamps from filenames
    timestamps = []
    for f in pcd_files:
        try:
            ts = float(f.stem)  # filename without extension
            timestamps.append((ts, f))
        except ValueError:
            print(f"⚠️ Skipping invalid file name: {f.name}")

    if not timestamps:
        print("❌ No valid .pcd files found.")
        return {}

    # Sort by timestamp and find start time
    timestamps.sort(key=lambda x: x[0])
    start_time = timestamps[0][0]

    # Group files by sector
    sector_map = defaultdict(list)
    for ts, file_path in timestamps:
        sector_idx = int((ts - start_time) // window)
        sector_name = f"sector{sector_idx}"
        sector_dir = Path(output_dir) / sector_name / 'pcl'
        sector_dir.mkdir(parents=True, exist_ok=True)

        dest = sector_dir / file_path.name
        shutil.copy(file_path, dest)
        sector_map[sector_name].append(file_path.name)

    print(f"✅ Split complete. Created {len(sector_map)} sectors.")
    return dict(sector_map)  # Convert to regular dict for JSON serialization


def main():
    parser = argparse.ArgumentParser(description="Split PCD files into time-based sectors.")
    parser.add_argument('--src', type=str, required=True, help='Source root directory')
    parser.add_argument('--date', type=str, required=True, help='Date folder (e.g., 2023-01-01)')
    parser.add_argument('--session', type=str, default="0", help='Session folder name')
    parser.add_argument('--target', type=str, required=True, help='Target directory to save split sectors')
    parser.add_argument('--window', type=int, default=20, help='Time window in seconds per sector')
    args = parser.parse_args()

    pcl_dir = Path(args.src) / args.date / args.session / "all_pcl"
    output_dir = Path(args.target) / args.date / args.session
    json_path = Path(args.target) / args.date / args.session  / "sector_pcl_data.json"

    print(f"📂 Processing PCDs from: {pcl_dir}")
    print(f"📁 Saving split sectors to: {output_dir}")

    sector_map = split_pcd_by_time(pcl_dir, output_dir, args.window)

    with open(json_path, "w") as f:
        json.dump(sector_map, f, indent=2)

    print(f"📝 Sector map saved to: {json_path}")


if __name__ == "__main__":
    main()