# Steps:
# 1. Go through all the black pixels in the image.
# 2. For each, search outward from the pixel to find a nonblack pixel value and
#    replace itself with that value.
# 4. Output the new image.


import cv2
import numpy as np
import math


# Checks if a coordinate is within the image borders
def checkInside(row, col):
    if row < 0 or row >= image.shape[0]:
        return False
    if col < 0 or col >= image.shape[1]:
        return False
    return True

# Finds nearest pixel value greater than 0
def findNearestVal(row, col):
    pixelDist = 1
    insideEdge = True
    while insideEdge:
        insideEdge = False
        rowVals = [pixelDist, pixelDist * -1]
        colVals = [pixelDist, pixelDist * -1]
        for x in rowVals:
            for i in range((pixelDist * -1), pixelDist + 1):
                if checkInside(row + x, col + i):
                    insideEdge = True
                    if image[row + x][col + i]:
                        return image[row + x][col + i]
        for y in colVals:
            for i in range((pixelDist * -1), pixelDist + 1):
                if checkInside(row + i, col + y):
                    insideEdge = True
                    if image[row + i][col + y]:
                        return image[row + i][col + y]

        pixelDist += 1

    return 255


# -----MAIN-----
image = cv2.imread("depthimg.png", 0)
newImg = image.copy()

for i in range(image.shape[0]):
    for j in range(image.shape[1]):
        if image[i][j] == 0:
            newImg[i][j] = findNearestVal(i, j)

cv2.imwrite("test.png", newImg)