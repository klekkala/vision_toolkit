import argparse
import os
import rosbag
import sys
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument('--frame', type=int)
parser.add_argument('--date', type=str)
parser.add_argument('--bag', type=str)
parser.add_argument('--cam', type=str)
parser.add_argument('--session', type=str)
parser.add_argument('--check', action='store_true')
args = parser.parse_args()

save_path = f'/lab/tmpig23b/navisim/data/bags/{args.date}/{args.session}/cam{args.cam}'
os.makedirs(save_path, exist_ok=True)
print(args.bag)
def fetch_remote_file(remote_path, local_path):
    # Use scp to copy the remote file locally
    try:
        subprocess.check_call(['scp', remote_path, local_path])
    except subprocess.CalledProcessError as e:
        with open('error_log.txt', 'a') as error_file:
            error_file.write(f"Failed to copy {remote_path}\n")
        sys.exit(1)

try:
    # Check if the bag file is remote (starts with ssh://)
    if 'student@iGpu10' in args.bag:
        # Create a local copy path
        local_bag = os.path.join(save_path, os.path.basename(args.bag))

        if not os.path.exists(local_bag):
            fetch_remote_file(args.bag, local_bag)
        bag = rosbag.Bag(local_bag)
    else:
        # Local file case
        bag = rosbag.Bag(args.bag)
except rosbag.bag.ROSBagException as e:
    with open('error_log.txt', 'a') as error_file:
        error_file.write(f"{args.bag}\n")
    sys.exit(1)
