import cv2
import numpy as np
import rosbag
import pandas as pd
import argparse
from cv_bridge import CvBridge
import os

parser = argparse.ArgumentParser()
parser.add_argument('--frame', type=int)
parser.add_argument('--bag', type=str)
parser.add_argument('--cam', type=str)
parser.add_argument('--save', type=str)
parser.add_argument('--num', type=str)

args = parser.parse_args()
save_path = "/lab/tmpig10b/kiran/bag_dump/"+args.save+"/" + args.num + "/all_imgs/"
os.makedirs(save_path, exist_ok=True)
bag = rosbag.Bag(args.bag)
bridge = CvBridge()

print(args.bag)
for topic, msg, t in bag.read_messages(topics=[f'/cam{args.cam}/color/image_raw/compressed']):
    np_arr = np.fromstring(msg.data, np.uint8)
    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    cv2.imwrite(save_path+'cam'+args.cam+'_'+str(t)+".jpg", cv_image)

bag.close()
