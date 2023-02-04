# Image order: 1 4 3 5 2
# Image 4 - rotYval 50, distXval 1295, [A = [90, 0]
#                                       B = [90, 480]
#                                       C = [477, 386]
#                                       D = [477, 96]]
# Image 1 - rotYval 113, distXval 0, [A = [45, 61]
#                                     B = [45, 421]
#                                     C = [557, 480]
#                                     D = [557, 0]]
# Image 5 - rotYval 120, distXval -=700, [A = [68, 81]
#                                         B = [68, 401]
#                                         C = [494, 480]
#                                         D = [494, 0]]
# Image 2 - rotYval 147, distXval -=1000, [A = [314, 114]
#                                          B = [314, 369]
#                                          C = [640, 470]
#                                          D = [640, 10]]

import cv2
import numpy as np
from math import sqrt

f = 500
rotXval = 90
rotYval = 90
rotZval = 90
distXval = 500
distYval = 500
distZval = 500

def onFchange(val):
    global f
    f = val
def onRotXChange(val):
    global rotXval
    rotXval = val
def onRotYChange(val):
    global rotYval
    rotYval = val
def onRotZChange(val):
    global rotZval
    rotZval = val
def onDistXChange(val):
    global distXval
    distXval = val
def onDistYChange(val):
    global distYval
    distYval = val
def onDistZChange(val):
    global distZval
    distZval = val

if __name__ == '__main__':

    #Read input image, and create output image
    src = cv2.imread('hello/cam2_Color.png')
    #src = cv2.resize(src,(640,480))
    dst = np.zeros_like(src)
    h, w = src.shape[:2]

    #Create user interface with trackbars that will allow to modify the parameters of the transformation
    wndname1 = "Source:"
    wndname2 = "WarpPerspective: "
    cv2.namedWindow(wndname1, 1)
    cv2.namedWindow(wndname2, 1)
    #cv2.createTrackbar("f", wndname2, f, 1000, onFchange)
    fx = 645
    fy = 622
    f = sqrt((fx * fx) + (fy * fy))
    cv2.createTrackbar("Rotation X", wndname2, rotXval, 180, onRotXChange)
    cv2.createTrackbar("Rotation Y", wndname2, rotYval, 180, onRotYChange)
    rotYval = 123
    cv2.createTrackbar("Rotation Z", wndname2, rotZval, 180, onRotZChange)
    cv2.createTrackbar("Distance X", wndname2, distXval, 2500, onDistXChange)
    distXval = -700
    cv2.createTrackbar("Distance Y", wndname2, distYval, 1000, onDistYChange)
    cv2.createTrackbar("Distance Z", wndname2, distZval, 1000, onDistZChange)

    #Show original image
    cv2.imshow(wndname1, src)

    k = -1
    while k != 27:

        if f <= 0: f = 1
        rotX = (rotXval - 90)*np.pi/180
        rotY = (rotYval - 90)*np.pi/180
        rotZ = (rotZval - 90)*np.pi/180
        distX = distXval - 500
        distY = distYval - 500
        distZ = distZval - 500

        # Camera intrinsic matrix
        K = np.array([[fx, 0, w/2, 0],
                    [0, fy, h/2, 0],
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

        # Composed rotation matrix with (RX,RY,RZ)
        R = np.linalg.multi_dot([ RX , RY , RZ ])

        # Translation matrix
        T = np.array([[1,0,0,distX],
                    [0,1,0,distY],
                    [0,0,1,distZ],
                    [0,0,0,1]])

        # Overall homography matrix
        H = np.linalg.multi_dot([K, R, T, Kinv])

        # Apply matrix transformation
        cv2.warpPerspective(src, H, (w, h), dst, cv2.INTER_NEAREST, cv2.BORDER_CONSTANT, 0)

        # Show the image
        cv2.imshow(wndname2, dst)
        k = cv2.waitKey(1)

    newDst = np.zeros_like(src)
    A = [210, 190]
    B = [210, 532]
    C = [798, 710]
    D = [798, 12]
    inputPts = np.float32([A, B, C, D])
    outputPts = np.float32([[0, 0],
                            [0, 720],
                            [1280, 720],
                            [1280, 0]])
    M = cv2.getPerspectiveTransform(inputPts, outputPts)
    cv2.warpPerspective(dst, M, (1280, 720), newDst, cv2.INTER_LINEAR)
    cv2.imwrite("Cam2.png", newDst)