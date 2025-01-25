## VISION_TOOLKIT
Contains code to extract images, pcd, gpt and legoloam, pcl, odom and text_gpt

### More about Vision Toolkit
| Component | Description |
| ---- | --- |
|BlockPcl|Contains codes and implementation necessary to extract files into blocks to read pcl files|
|BagDump|Contains codes and implemenation to extract necessary files in bag dump|
Usually, VisionTool is used to either:
* To extract files into bag_dump folder
* To preprocess files to run bag_dump

### How to use 
To run the entire pipeline use the following script
```
extract_bag_dump.sh $date
```

### LegoLoam Blocks
LegoLoam : Due to memory constraints, we divide the bag files into blocks of 10 minutes before running LegoLoam After running LegoLoam, the blocks are deleted (implementation can be found [here](https://github.com/klekkala/vision_toolkit/blob/main/scripts/split_blocks/run_LeGO_with_blocks.sh)
```
run_LeGO_with_blocks.sh $date
```

sync_data.json, text_gpt4 
Uses implementation from 3d2d_ann. 
Needs to redefine if location of 3d2d_ann project changes
# extract bagfile code

This program extracts images from bag files and organizes them into structured folders. To run the program, use the following command:
```
./bag_.sh 2023_06_30
```
This command will extract images from the bag files located in /data/. Simply pass the folder name (e.g., 2023_06_30), assuming it contains subfolders named cam1, cam2, cam3, cam4, and cam5. The program will generate an output folder with the same name (e.g., 2023_06_30/imgs).

To process all dates, you can run:
```
./bag_.sh 2023_06_30
```

# All_sessions.txt
This file contains the mapping of each session to its corresponding bag files. For example:
```
2023_08_10/0	test_2023-08-10-02-23-01.bag	test_2023-08-09-19-21-48.bag	test_2023-08-09-19-21-51.bag	test_2023-03-02-05-03-14.bag	test_2023-03-02-05-03-04.bag	
```


# divide session by sector

To divide a session into sectors for COLMAP, run:
```
./run_divide.sh 2023_06_30
```

This command divides the session into sectors. To process all dates, use:
```
./parallel_divide.sh
```

# SLAM

To run LeGO-LOAM for a bag file and save the data in the corresponding folder, use:
```
./run_LeGO_parameter.sh bag_path session date
```
To process all dates, run:
```
./all_LeGO_24.sh
```

# COLMAP
To run COLMAP Structure-from-Motion (SfM) for a specific date with a specified number of threads, use:
```
./sub_colmap/run.py date threads
```
