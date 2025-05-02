#!/usr/bin/env python3

import cv2
import numpy as np
# ========================= Student's code starts here =========================

# Params for camera calibration
theta = 0
beta = 760.0
tx = 0.22
ty = 0.07

# Function that converts image coord to world coord
def IMG2W(col, row):
    o_r = 240
    o_c = 320
    x_c = (row - o_r) / beta
    y_c = (col - o_c) / beta
    x_c2 = x_c + tx
    y_c2 = y_c + ty
    xw = (x_c2 * np.cos(theta) - y_c2 * np.sin(theta))
    yw = (x_c2 * np.sin(theta) + y_c2 * np.cos(theta)) + 0.01
    
    return (xw, yw)


# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    # Filter by Color
    params.filterByColor = False

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 100

    # Filter by Circularity
    params.filterByCircularity = False
    params.minCircularity = 0.6
    params.maxCircularity = 1

    # Filter by Inerita
    params.filterByInertia = False
    params.minInertiaRatio = 0.6

    # Filter by Convexity
    params.filterByConvexity = False

    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    # ========================= Student's code starts here =========================
    
    if color == "green":
        lower = (30,50,60)    # green lower
        upper = (80,255,255)   # green upper

        mask_image = cv2.inRange(hsv_image, lower, upper)
    elif color == "yellow":
        lower = (20,50,60)    # yellow lower
        upper = (40,255,255)   # yellow upper

        mask_image = cv2.inRange(hsv_image, lower, upper)
    else:
        lower2 = (140,150,150)    # pink lower
        upper2 = (179,255,255)   # pink upper
        
        lower3 = (0,150,150)    # pink lower
        upper3 = (10,255,255)   # pink upper
        
        mask_image2 = cv2.inRange(hsv_image, lower2, upper2)
        mask_image3 = cv2.inRange(hsv_image, lower3, upper3)
        
        mask_image = cv2.bitwise_or(mask_image2, mask_image3)
    
    # ========================= Student's code ends here ===========================

    keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))

    # ========================= Student's code starts here =========================

    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, 0, (255,0,0), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # ========================= Student's code ends here ===========================

    xw_yw = []

    #if(num_blobs == 0):
    #    print("No block found!")
    #else:
        # Convert image coordinates to global world coordinate using IM2W() function
    for i in range(num_blobs):
        xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))
    #print(blob_image_center)


    cv2.namedWindow("Camera View")
    cv2.imshow("Camera View", image_raw)
    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", mask_image)
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)

    cv2.waitKey(2)

    return xw_yw
