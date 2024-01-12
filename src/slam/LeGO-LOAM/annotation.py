import os

PCD_DIR = "/home2/carla/"
ODOM_FILE = "/tmp/odometry.txt"
OUTFILE = "result.txt"

time_stamps = []
file_names = []
for fn in os.listdir(PCD_DIR):
    if ".pcd" in fn:
        file_names.append(fn)
        num = round(float(fn[:-6]), 6)
        time_stamps.append(f"{num:.6f}")
time_stamps.sort()
file_names.sort()

poses = []
with open(ODOM_FILE, "r") as f:
    for line in f:
        poses.append(line.strip().split(", "))
        for i in range(6):
            poses[-1][i] = float(poses[-1][i])

all_poses = [poses[0][:-1]]
all_poses[-1].append(file_names[time_stamps.index(poses[0][-1])])
for i in range(len(poses)-1):
    time0 = poses[i][-1]
    time1 = poses[i+1][-1]

    f0 = time_stamps.index(time0)
    f1 = time_stamps.index(time1)

    N = f1 - f0

    for j in range(1, N):
        pose = []
        pose.append(poses[i + 1][0] * j / N + poses[i][0] * (1 - j / N))
        pose.append(poses[i + 1][1] * j / N + poses[i][1] * (1 - j / N))
        pose.append(poses[i + 1][2] * j / N + poses[i][2] * (1 - j / N))
        pose.append(poses[i + 1][3] * j / N + poses[i][3] * (1 - j / N))
        pose.append(poses[i + 1][4] * j / N + poses[i][4] * (1 - j / N))
        pose.append(poses[i + 1][5] * j / N + poses[i][5] * (1 - j / N))
        pose.append(file_names[f0+j])
        all_poses.append(pose)

    all_poses.append(poses[i + 1][:-1])
    all_poses[-1].append(file_names[f1])

with open(OUTFILE, "w") as f:
    for line in all_poses:
        # Format each element to one decimal place, join with a comma and write to the file
        formatted_list = [f"{float(elem):.6f}" for elem in line[:-1]]
        formatted_list.append(line[-1])
        f.write(','.join(formatted_list) + '\n')
