import argparse
import os
import rosbag
import sys
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument('--dir', type=str)
parser.add_argument('--path', type = str)
parser.add_argument('--date', type=str)
parser.add_argument('--session', type=int)
args = parser.parse_args()

save_path = f'/lab/tmpig23b/navisim/data/bag_dump/{args.date}/{args.session}/'
os.makedirs(save_path, exist_ok=True)

def fetch_remote_dir(remote_path, local_path):
    # Use scp to copy the remote file locally
    try:
        subprocess.check_call(['scp', '-r', remote_path, local_path])
    except subprocess.CalledProcessError as e:
        with open('error_log.txt', 'a') as error_file:
            error_file.write(f"Failed to copy directory {remote_path}\n")
        sys.exit(1)

# Check if the bag file is remote (starts with ssh://)
fetch_remote_dir(args.path, save_path)