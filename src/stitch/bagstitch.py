import rosbag
from autostitcher import autostitch
import cv2
from cv_bridge import CvBridge


# Initialize variables
bag = rosbag.Bag("merged.bag")
#imgOrder = ["1", "4", "3", "5", "2"]
colorImgs = [[], [], [], [], []]
depthImgs = [[], [], [], [], []]
colorCount = 0
depthCount = 0
bridge = CvBridge()

# Read merged bag file
for cam in range(0, 5):
    colorTopic = "/cam_" + str(cam + 1) + "/color/image_raw"
    depthTopic = "/cam_" + str(cam + 1) + "/depth/image_rect_raw"
    for topic, msg, t in bag.read_messages(topics=[colorTopic, depthTopic]):
        if topic == colorTopic:
            colorCount += 1
            colorImgs[cam].append(bridge.imgmsg_to_cv2(msg))
        elif topic == depthTopic:
            depthCount += 1
            depthImgs[cam].append(bridge.imgmsg_to_cv2(msg))

print(colorCount, depthCount)
print(len(colorImgs[0]), len(depthImgs[0]))

# Run stitcher on all depth image sets
for i in range(0, len(depthImgs[0])):
    for cam in range(1, 6):
        cv2.imwrite("hello/cam" + str(cam) + ".png", depthImgs[cam - 1][i])
    autostitch("hello/depthpano" + str(i) + ".png")

# Run stitcher on all color image sets
for i in range(0, len(colorImgs[0])):
    for cam in range(1, 6):
        cv2.imwrite("hello/cam" + str(cam) + ".png", colorImgs[cam - 1][i])
    autostitch("hello/colorpano" + str(i) + ".png")

