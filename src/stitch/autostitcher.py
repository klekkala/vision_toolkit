# Stitches 5 images from the RL project into a panorama
# Image order: 1, 4, 3, 5, 2

import cv2
import numpy as np
from math import sqrt
from PIL import Image
import wand.image
import matplotlib.pyplot as plt


# INPUT
images = []
paths = ["hello/cam1_Color.png", "hello/cam4_Color.png", "hello/cam3_Color.png",
         "hello/cam5_Color.png", "hello/cam2_Color.png"]
for path in paths:
    newImg = cv2.imread(path)
    images.append(newImg)


# INITIALIZATION
h, w = images[0].shape[:2]
# Intrinsic F parameters for each image
fx = [664, 519, 1357, 446, 645]
fy = [661, 532, 1314, 451, 622]
barrelDistort = (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.1, 0.5)

# Rotation and translation parameters for each image
rotXval = 90
rotYval = [65, 67, 90, 100, 123]
rotZval = 90
distXval = [1618, 1575, 500, 40, -700]
distYval = 500
distZval = 500

# Points used to transpose each rotated image to the flat 1280x720 canvas
srcPts = [[[500, 2], [500, 719], [1200, 559], [1200, 164]],
          [[540, 0], [540, 720], [1188, 543], [1188, 180]],
          [],
          [[1, 98], [1, 625], [1092, 720], [1092, 0]],
          [[210, 190], [210, 532], [798, 710], [798, 12]]]
outputPts = np.float32([[0, 0],
                        [0, 720],
                        [1280, 720],
                        [1280, 0]])


# TRANPOSE IMAGES
dst = []
# Rotate each image according to the parameters and then flatten it
for i in range(0, len(images)):
    # Convert parameters to the appropriate values
    rotX = (rotXval - 90)*np.pi/180
    rotY = (rotYval[i] - 90)*np.pi/180
    rotZ = (rotZval - 90)*np.pi/180
    distX = distXval[i] - 500
    distY = distYval - 500
    distZ = distZval - 500

    # Camera intrinsic matrix
    f = sqrt((fx[i] * fx[i]) + (fy[i] * fy[i]))
    K = np.array([[fx[i], 0, w/2, 0],
                [0, fy[i], h/2, 0],
                [0, 0,   1, 0]])
    # K inverse
    Kinv = np.zeros((4,3))
    Kinv[:3,:3] = np.linalg.inv(K[:3,:3])*f
    Kinv[-1,:] = [0, 0, 1]

    # Rotation matrices around the X,Y,Z axis
    RX = np.array([[1,           0,            0, 0],
                    [0,np.cos(rotX),-np.sin(rotX), 0],
                    [0,np.sin(rotX),np.cos(rotX) , 0],
                    [0,           0,            0, 1]])
    RY = np.array([[ np.cos(rotY), 0, np.sin(rotY), 0],
                    [            0, 1,            0, 0],
                    [ -np.sin(rotY), 0, np.cos(rotY), 0],
                    [            0, 0,            0, 1]])
    RZ = np.array([[ np.cos(rotZ), -np.sin(rotZ), 0, 0],
                    [ np.sin(rotZ), np.cos(rotZ), 0, 0],
                    [            0,            0, 1, 0],
                    [            0,            0, 0, 1]])
    R = np.linalg.multi_dot([ RX , RY , RZ ])

    # Translation matrix
    T = np.array([[1,0,0,distX],
                    [0,1,0,distY],
                    [0,0,1,distZ],
                    [0,0,0,1]])

    # Apply calculated homography matrix to get the image with new perspective
    H = np.linalg.multi_dot([K, R, T, Kinv])
    midImg = np.zeros_like(images[i])
    cv2.warpPerspective(images[i], H, (w, h), midImg, cv2.INTER_NEAREST,
                        cv2.BORDER_CONSTANT, 0)

    # Middle image that doesn't need further adjustment
    if i == 2:
        distImg = wand.image.Image.from_array(midImg)
        distImg.distort("barrel", barrelDistort)
        dst.append(np.array(distImg))
        continue

    # Flatten the image back to a 1280x720 sized image
    finalImg = np.zeros_like(images[i])
    inputPts = np.float32([srcPts[i][0], srcPts[i][1], srcPts[i][2], srcPts[i][3]])
    M = cv2.getPerspectiveTransform(inputPts, outputPts)
    cv2.warpPerspective(midImg, M, (1280, 720), finalImg, cv2.INTER_LINEAR)

    # Apply barrel distortion
    distImg = wand.image.Image.from_array(finalImg)
    distImg.distort("barrel", barrelDistort)
    finalImg = np.array(distImg)

    dst.append(finalImg)
    #cv2.imshow("Image " + str(i), finalImg)
    #cv2.waitKey(0)


# BLEND (using laplacian pyramids)
"""
# Generate Laplacian pyramids for each image
lps = []
print("Original size:", dst[0].shape)
print("PyrDown sizes:")
for img in range(len(dst)):
    # Gaussian pyramid
    imgCopy = dst[img].copy()
    imgGs = [imgCopy]
    for i in range(6):
        imgCopy = cv2.pyrDown(imgCopy)
        imgGs.append(imgCopy)
        if img == 0:
            print(imgCopy.shape)
    
    if img == 0:
        print("PyrUp Sizes:")
    # Laplacian pyramid
    imgCopy = imgGs[5].copy()
    imgLs = [imgCopy]
    for i in range(5, 0, -1):
        expandedGaussian = cv2.pyrUp(imgGs[i])
        if expandedGaussian.shape[0] == 46:
            expandedGaussian = expandedGaussian[:45, :, :]
        imgLs.append(cv2.subtract(imgGs[i - 1], expandedGaussian))
        if img == 0:
            print(expandedGaussian.shape)
    lps.append(imgLs)

# Blend the images together
testBlend = []
n = 0
for test1Lap, test2Lap in zip(lps[0], lps[1]):
    n += 1
    cols, rows, ch = test1Lap.shape
    lapl = np.hstack((test1Lap, test2Lap))
    testBlend.append(lapl)

testReconstruct = testBlend[0].copy()
for i in range(1, 6):
    testReconstruct = cv2.pyrUp(testReconstruct)
    if testReconstruct.shape[0] == 46:
            testReconstruct = testReconstruct[:45, :, :]
    print(testReconstruct.shape, testBlend[i].shape)
    col, row, ch = testReconstruct.shape
    testReconstruct = cv2.add(testBlend[i][:col, :row, :], 
                                testReconstruct)
print(testReconstruct.shape)
cv2.imwrite("test.png", testReconstruct[:, :, ::-1])
"""


# BLEND (using exponential value averaging)
shifts = [1140, 1010, 1110, 1170]
blendedDst = []
for i in range(0, len(dst) - 1):
    # Fade in the next image based on column number
    temp = dst[i].copy()
    for col in range(w):
        if col < shifts[i]:
            continue
        else:
            alpha = (col - shifts[i]) / (1280 - shifts[i])
            newCol = (temp[:, col, :] * (1.0 - alpha)) + \
                        (dst[i + 1][:, col - shifts[i], :] * (alpha))
            temp[:, col, :] = newCol
    blendedDst.append(temp)
blendedDst.append(dst[-1])


# STITCH IMAGES
panorama = Image.new("RGB", size=(5710, h), color=(0, 0, 0))
# Shifts each image over the right amount
newShifts = [(0, 0), (1140, 0), (2150, 0), (3260, 0), (4430, 0)]
# Adds each image to panorama cameras starting from the outside images
for i in range(len(blendedDst) - 1, -1, -1):
    insertImg = Image.fromarray(cv2.cvtColor(blendedDst[i], cv2.COLOR_BGR2RGB))
    panorama.paste(insertImg, newShifts[i])

panorama.save("testpanorama.png")
