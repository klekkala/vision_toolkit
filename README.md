# 3dobject_tracking

Build instructions:
(From root directory of the repo, ie. /path/to/3dobject_tracking/)

```
mkdir build
cd build
```
Then:
(CMake):
```
cmake ../
make
```

or
(Ninja):
```
cmake -G Ninja ../
ninja
```

ROS version: http://wiki.ros.org/melodic/Installation (Recommended for Ubuntu 18.04 which is what I have)

# removeNoise
Removes noise with a value of 0 from a depth image.

With the noisy image named "depthimage.png" in the same directory as the program, run the following command:
```
python removeNoise.py
```

# autostitcher
Stitches together 5 images into a panorama.

The images are located in a directory named "hello" and are named "cam1.png", "cam2.png", "cam3.png", "cam4.png", "cam5.png". Run the following command in the terminal:
```
python autostitcher
```
The resulting panorama is outputted with the default file name "testpanorama.png".
