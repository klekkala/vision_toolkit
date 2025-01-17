import rosbag
import os
import argparse
import subprocess
import tqdm
from rospy import Time

from datetime import timedelta


def split_rosbag(input_bag_path, output_bag_path, split_duration=600):
    """
    Split a ROS bag into smaller blocks of a given duration.

    Args:
        input_bag_path (str): Path to the input ROS bag file.
        output_bag_prefix (str): Prefix for the output bag files.
        split_duration (int): Duration in seconds for each split bag. Default is 600 secods (10 min)
    """
    output_bag_name = input_bag_path.split('/')[-1].replace('.bag', '')

    with rosbag.Bag(input_bag_path, 'r') as inbag:
        start_time = inbag.get_start_time()
        end_time = inbag.get_end_time()
        current_time = start_time
        bag_index = 0
        
        while current_time < end_time:
            split_start_time = current_time
            split_end_time = min(current_time + split_duration, end_time)
            new_output_bag = os.path.join(output_bag_path, f"{output_bag_name}_block_{bag_index}.bag")
            
            with rosbag.Bag(new_output_bag, 'w') as outbag:
                for topic, msg, t in inbag.read_messages(
                        start_time=Time(split_start_time), 
                        end_time=Time(split_end_time)):
                    outbag.write(topic, msg, t)
            
            print(f"Created {new_output_bag} from {split_start_time} to {split_end_time}.")
            current_time = split_end_time
            bag_index += 1

def get_bags(path, date, session):
    all_bags = {}
    for cam_num in range(1, 6):
        bag_dir = os.path.join(path, date, session, f'cam{cam_num}')
        bags_per_cam = [os.path.join(bag_dir, f) for f in os.listdir(bag_dir) if f.endswith(".bag")]
        all_bags[cam_num] = bags_per_cam

    return all_bags
        

def get_sesession(path, date):
    return [f for f in os.listdir(os.path.join(path, date))]

def download_bags_if_not_exists(path, date, script_name = 'download_bag.sh'):
    script_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', script_name)
    is_path_exists = os.path.exists(os.path.join(path, date))
    args = [date]
    if is_path_exists:
        return
    
    try:
        print('downloading bag')
        # Use subprocess.run to execute the shell script
        result = subprocess.run(
            [script_path, *args],  # Pass script and arguments
            check=True,           # Raise an exception on error
            text=True,            # Return output as text (Python 3.7+)
            stdout=subprocess.PIPE,  # Capture standard output
            stderr=subprocess.PIPE   # Capture standard error
        )
    except subprocess.CalledProcessError as e:
        print("Error running script:")
        print(e.stderr)


def parse_arguments():
    parser = argparse.ArgumentParser(description="Split a ROS bag into smaller chunks.")
    parser.add_argument(
        "-i", "--input", 
        required=True, 
        help="Path to the input ROS bag file."
    )
    parser.add_argument(
        "-o", "--output", 
        required=False, 
        help="Prefix for the output bag files."
    )
    parser.add_argument(
        "--duration", 
        type=int, 
        default=600, 
        help="Duration of each split bag in seconds (default: 600 seconds)."
    )
    parser.add_argument(
        '--date',
        type=str,

    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_arguments()
    input_path = args.input
    output_path = args.output
    date = args.date

    download_bags_if_not_exists(input_path, date)
    sessions = get_sesession(input_path, date)

    for session_idx in tqdm(range(0, len(sessions)), desc='Handling sequences'):
        all_bags_dir = get_bags(input_path, date, sessions[session_idx])
        
        for cam_num, dirs in tqdm(all_bags_dir.items(), desc = 'Handling bags of each camera'):
            output_bag_path = os.path.join(output_path, date, session, f'cam{cam_num}', 'blocks')
            is_path_exists = os.path.exists(output_bag_path)
            
            if not is_path_exists:
                os.mkdir(output_bag_path)
            
            for bag in dirs:
                split_rosbag(bag, output_bag_path)

#python split_bag.py -i /lab/tmpig23b/navisim/data/bags -o /lab/tmpig23b/navisim/data/bags --date 2023_03_11