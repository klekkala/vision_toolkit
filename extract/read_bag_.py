import cv2
import numpy as np
import rosbag
import pandas as pd
import argparse
from cv_bridge import CvBridge
import os
import sys
parser = argparse.ArgumentParser()
parser.add_argument('--frame', type=int)
parser.add_argument('--bag', type=str)
parser.add_argument('--cam', type=str)
parser.add_argument('--date', type=str)
parser.add_argument('--check', action='store_true')

args = parser.parse_args()
save_path = "/lab/tmpig23b/navisim/data/bag_dump/"+args.date+"/all_imgs/"
os.makedirs(save_path, exist_ok=True)

try:
    bag = rosbag.Bag(args.bag)
except rosbag.bag.ROSBagException as e:
    with open('error_log.txt', 'a') as error_file:
        error_file.write(f"{args.bag}\n")
    sys.exit(1)
bridge = CvBridge()


print(bag.get_type_and_topic_info().topics)
img_topic=None
if f'/cam{args.cam}/color/image_raw/compressed' in bag.get_type_and_topic_info().topics.keys():
    img_topic = f'/cam{args.cam}/color/image_raw/compressed'
elif f'/cam{args.cam}/color/image_raw' in bag.get_type_and_topic_info().topics.keys():
    img_topic = f'/cam{args.cam}/color/image_raw'

if not img_topic:
    with open('error_log.txt', 'a') as error_file:
        error_file.write(f"{args.bag}\n")
    sys.exit(1)

print('now', args.bag)

for topic, msg, t in bag.read_messages(topics=[img_topic]):
    if args.check:
        if os.path.exists(save_path+'cam'+args.cam+'_'+str(t)+".jpg"):
            continue
        else:
            print('Check error!')
        continue
    np_arr = np.fromstring(msg.data, np.uint8)
    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    cv2.imwrite(save_path+'cam'+args.cam+'_'+str(t)+".jpg", cv_image)

bag.close()
