from pypcd4 import PointCloud
import numpy as np
import os

folder_path = '/lab/tmpig23b/navisim/data/bag_dump/2023_03_11/0/all_pcl'

# Loop through each file in the folder
for filename in os.listdir(folder_path)[:10]:
    # Create the full path to the file
    file_path = os.path.join(folder_path, filename)

    # Check if the current object is a file (to skip directories)
    if os.path.isfile(file_path):
        with open(file_path, 'rb') as f:
            pc = PointCloud.from_fileobj(f)
            print(pc.fields)


