# -*- coding: utf-8 -*-
import cv2
import time
from cv2 import cornerHarris
import numpy as np
from numpy import random,mat
 
AUTO = False  # 自动拍照，或手动按s键拍照
C = False  # 自动拍照，或手动按s键拍照
INTERVAL = 2 # 自动拍照间隔
criteria =(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# Prepare object points
objp = np.zeros((8*6,3), np.float32)
objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all images
objpoints= []   # 3d points in real world space
imgpointsR= []  # 2d points in image plane   
imgpointsL= []   
 
cv2.namedWindow("left")
cv2.namedWindow("right")
camera = cv2.VideoCapture(0)
 
# 设置分辨率 左右摄像机同一频率，同一设备ID；左右摄像机总分辨率1280x480；分割为两个640x480、640x480
camera.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
 
counter = 0
utc = time.time()
folder = "D:\\CQUniversity\\1_billion\\camcapture\\codes\\SaveImage\\" # 拍照文件目录
 
def shot(pos, frame):
    global counter
    path = folder + pos + str(counter) + ".jpg"
 
    cv2.imwrite(path, frame)
    print("snapshot saved into: " + path)
 
while True:
    ret, frame = camera.read()
    # 裁剪坐标为[y0:y1, x0:x1] HEIGHT*WIDTH
    leftFrame = frame[0:480, 0:640]
    rightFrame = frame[0:480, 640:1280]

    #转换 array-> matrix
    #leftFrame=mat(leftFrame)
    #rightFrame=mat(rightFrame)


    #（采集灰度左右图像）
    grayL=cv2.cvtColor(leftFrame,cv2.COLOR_BGR2GRAY)
    grayR=cv2.cvtColor(rightFrame,cv2.COLOR_BGR2GRAY)


    # Find the chess board corners
    ret, cornersR = cv2.findChessboardCorners(grayR,(8,5),None)  # Define the number of chess corners we are looking for
    ret, cornersL = cv2.findChessboardCorners(grayL,(8,5),None)  # Left side


    #cv2.imshow("left", leftFrame)
    #cv2.imshow("right", rightFrame)
 
    now = time.time()
    if AUTO and now - utc >= INTERVAL:
        shot("left", leftFrame)
        shot("right", rightFrame)
        counter += 1
        utc = now
 
 # If found, add object points, image points (after refining them)
    if (True == ret) & (False == C):
        objpoints.append(objp)
        corners2R= cv2.cornerSubPix(grayR,cornersR,(11,11),(-1,-1),criteria)    # Refining the Position
        imgpointsR.append(cornersR)
        corners2L= cv2.cornerSubPix(grayL,cornersL,(11,11),(-1,-1),criteria)
        imgpointsL.append(cornersL)

        # Draw and display the corners
        cv2.drawChessboardCorners(rightFrame,(8,5),corners2R,ret)
        cv2.drawChessboardCorners(leftFrame,(8,5),corners2L,ret)
        cv2.imshow('right',rightFrame)
        cv2.imshow('left',leftFrame)

#        key=cv2.waitKey(1)
#        if key == ord('s'):   # Push "s" to save the images and "c" if u want to not save the images
#            t= str(i)
#            print('Saved'+t)
#            shot('chessboard-R'+t+'.png',rightFrame)
#            shot('chessboard-L'+t+'.png',leftFrame)
#            #cv2.imwrite('chessboard-R'+t+'.png',rightFrame) # Save the image in the file where this Programm is located
#            #cv2.imwrite('chessboard-L'+t+'.png',leftFrame)
#            i=i+1
#        elif key==ord(" "):
#            print('canceled')
#            break 


        key = cv2.waitKey(1)
        if key == ord(" "):
            break
        elif key == ord("s"):
            shot("chessboard-L", leftFrame)
            shot("chessboard-R", rightFrame)
            counter += 1




    
camera.release()
cv2.destroyWindow("left")
cv2.destroyWindow("right")