# This script will process the sensor data coming from the host and send back a response
# including the adjustments that need to be made to the user input

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2
import glob
import os

def birdeye_view(img):
    img_size = (img.shape[1], img.shape[0])
    offset = 450

    # Source points taken from images with straight lane lines, these are to become parallel after the warp transform
    # Needs to be updated with accurate lane coordinates
    src = np.array([
        (190, 720), # bottom-left corner
        (596, 447), # top-left corner
        (685, 447), # top-right corner
        (1125, 720) # bottom-right corner
    ], dtype='f')
    dst = np.array([
        [offset, img_size[1]],             # bottom-left corner
        [offset, 0],                       # top-left corner
        [img_size[0]-offset, 0],           # top-right corner
        [img_size[0]-offset, img_size[1]]  # bottom-right corner
    ], dtype='f')

    # Calculate the transformation matrix and it's inverse transformation
    M = cv2.getPerspectiveTransform(src, dst)
    M_inv = cv2.getPerspectiveTransform(dst, src)
    warped = cv2.warpPerspective(img, M, img_size)
   
    return warped, M_inv

def binary_threshold(img):
    # Transform to gray scale
    gray = cv2.cvtColor(img, cv2.COLOR_BAYER_BGGR2GRAY)
    # Apply sobel in x direction (detect vertical lines)
    sobelx = cv2.Sobel(gray, 6, 1, 0)
    abs_sobelx = np.absolute(sobelx)

    scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx))
    sx_binary = np.zeros_like(scaled_sobel)
    
    sx_binary[(scaled_sobel >= 30) & (scaled_sobel <= 255)] = 1

    # Detect white pixels
    white_binary = np.zeros_like(gray)
    white_binary[(gray > 200) & (gray <= 255)] = 1

    # Convert to HLS
    hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
    H = hls[:,:,0]
    S = hls[:,:,2]

    # Detect high saturation
    sat_binary = np.zeros_like(S)
    sat_binary[(S > 90) & (S <= 255)] = 1

    # Detect yellow pixels
    hue_binary = np.zeros_like(H)
    hue_binary[(H > 10) & (H <= 25)] = 1

    # Combine results
    binary_1 = cv2.bitwise_or(sx_binary, white_binary)
    binary_2 = cv2.bitwise_or(hue_binary, sat_binary)
    
    return cv2.bitwise_or(binary_1, binary_2)

def detect_lane_lines(binary_birdeye):
    # Make histogram of bottom half of img
    histogram = np.sum(binary_birdeye[binary_birdeye.shape[0]//2:,:], axis=0)

    # Find lanes starting points
    midpoint = np.int32(histogram.shape[0]//2)
    left_base = np.argmax(histogram[:midpoint])
    right_base = np.argmax(histogram[midpoint:]) + midpoint

    nwindows = 9
    margin = 150
    minpix = 75

    window_h = np.int32(binary_birdeye.shape[0]//nwindows)

    nonzero = binary_birdeye.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])

    left_curr = left_base
    right_curr = right_base

    left_lane = []
    right_lane = []

    for window in range(nwindows):
        win_y_low = binary_birdeye.shape[0] - (window + 1) * window_h
        win_y_high = binary_birdeye.shape[0] - window * window_h
        win_left_low = left_curr - margin
        win_left_high = left_curr + margin
        win_right_low = right_curr - margin
        win_right_high = right_curr + margin

        # Identiry nonzero pixels within the window
        good_left_lane = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high)
                          & (nonzerox >= win_left_low) & (nonzerox < win_left_high)).nonzero()[0]
        good_right_lane = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high)
                           & (nonzerox >= win_right_low) & (nonzerox < win_right_high)).nonzero()[0]
        
        left_lane.append(good_left_lane)
        right_lane.append(good_right_lane)

        # Recenter window if needed
        if len(good_right_lane) > minpix:
            left_curr = np.int32(np.mean(nonzerox[good_left_lane]))
        if len(good_right_lane) > minpix:
            right_curr = np.int32(np.mean(nonzerox[good_right_lane]))

    # Not needed?
    # try:
    #     left_lane = np.concatenate(left_lane)
    #     right_lane = np.concatenate(right_lane)
    # except ValueError:
    #     pass

    return nonzerox[left_lane], nonzeroy[left_lane], nonzerox[right_lane], nonzeroy[right_lane]

def fit_poly(binary_birdeye, leftx, lefty, rightx, righty):
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)

    # Generate values for plotting
    ploty = np.linspace(0, binary_birdeye.shape[0] - 1, binary_birdeye.shape[0])
    try:
        left_fitx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]
        right_fitx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]
    except TypeError:
        print("Failed to fit lane!")
        left_fitx = ploty**2 + ploty
        right_fitx = ploty**2 + ploty

    return left_fit, right_fit, left_fitx, right_fitx, ploty
# todo

img = cv2.imread('C:\\Users\\RUI1SF\\Pictures\\highway_hood.png')
birdeye_view(img)