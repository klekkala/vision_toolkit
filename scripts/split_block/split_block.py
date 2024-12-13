import argparse
import rosbag
import rospy
import os
from datetime import timedelta

def split_rosbag(input_bag_path, date, output_path, split_duration=600):
    """
    Split a ROS bag into smaller bags of a given duration.

    Args:
        input_bag_path (str): Path to the input ROS bag file.
        output_bag_prefix (str): Prefix for the output bag files.
        split_duration (int): Duration in seconds for each split bag.
    """

    bags = get_all_bags(input_bag_path)
    

    for key, bag_list in bags.items():
        for bag in bag_list:
            bag_name = bag.split('/')[-1].replace('.bag', '')
            output_bag_name = f'{key}_{bag_name}'

            with rosbag.Bag(bag, 'r') as inbag:
                start_time = inbag.get_start_time()
                end_time = inbag.get_end_time()
                current_time = start_time
                bag_index = 0
                
                while current_time < end_time:
                    split_start_time = current_time
                    split_end_time = min(current_time + split_duration, end_time)
                    block_name = f"{output_bag_name}_{bag_index}.bag"
                    
                    with rosbag.Bag(os.path.join(output_path, block_name), 'w') as outbag:
                        for topic, msg, t in inbag.read_messages(
                                start_time=rospy.Time(split_start_time), 
                                end_time=rospy.Time(split_end_time)):
                            outbag.write(topic, msg, t)
                    
                    print(f"Created {output_bag_name} from {split_start_time} to {split_end_time}.")
                    current_time = split_end_time
                    bag_index += 1

def get_all_bags(input_bag_path):
    cams = [os.path.join(input_bag_path, f) for f in os.listdir(input_bag_path)]
    
    bags = {}
    for cam_id, path in enumerate(cams):
        bag_path = [os.path.join(path, f) for f in os.listdir(path)]
        id = f'cam{cam_id+1}'
        if not cam_id in bags:
            bags[id] = []
        bags[id].extend(bag_path)
    
    return bags


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Split a ROS bag into smaller chunks.")
    parser.add_argument(
        "-i", "--input", 
        required=True, 
        help="Path to the input ROS bag file."
    )
    parser.add_argument(
        "-o", "--output", 
        required=True, 
        help="Prefix for the output bag files."
    )
    parser.add_argument(
        "--date", 
        required=True, 
        help="Date for the input bag files."
    )
    parser.add_argument(
        "-d", "--duration", 
        type=int, 
        default=600, 
        help="Duration of each split bag in seconds (default: 600 seconds)."
    )

    
    args = parser.parse_args()
    if not os.path.exists(args.output):
        os.makedirs(args.output)

    split_rosbag(args.input, args.date, args.output, args.duration)


    